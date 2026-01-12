/*
 * ESP32-S2 Safety Node - TWAI CAN Bus Implementation
 * 
 * Implements Safety Node Protocol (CAN ID: 0x005)
 * - Device ID 0x01: Safety Device 1 (e.g., Left/Yaw Limit)
 * - Device ID 0x02: Safety Device 2 (e.g., Up/Pitch Limit)
 * 
 * Board: Wemos/Lolin S2 Mini (ESP32-S2FN4R2)
 * Framework: PlatformIO (Arduino Core)
 * CAN: Internal TWAI (Two-Wire Automotive Interface)
 */

#include <Arduino.h>
#include "driver/twai.h"

// CAN Configuration
#define CAN_ID_SAFETY       0x005
#define DEVICE_ID_1         0x01
#define DEVICE_ID_2         0x02
#define STATUS_MIN_LIMIT    0x10
#define STATUS_MAX_LIMIT    0x20
#define STATUS_LIMIT1_FIND  0x11
#define STATUS_LIMIT2_FIND  0x12

// CAN Bitrate: 500 kbps (matching Orin NX)
#define CAN_BITRATE         TWAI_TIMING_CONFIG_500KBITS()

// Heartbeat interval: 5000 ms (5 seconds)
#define HEARTBEAT_INTERVAL  5000

// Debounce time: 50ms (stable state required)
#define DEBOUNCE_TIME       50

// Watchdog timeout: 2 seconds
#define WDT_TIMEOUT_MS      2000

// Hall sensor thresholds for LED control
#define HALL_RED_BLINK_THRESHOLD     2460   // ADC < 2460 - red blink limit1 find  send STATUS_LIMIT1_FIND  0x11
#define HALL_RED_ON_THRESHOLD        2160   // ADC < 2160 - red on constantly border limit1
#define HALL_GREEN_BLINK_THRESHOLD  2860   // ADC > 2860 - green blink limit2 find  send STATUS_LIMIT2_FIND  0x12
#define HALL_GREEN_ON_THRESHOLD      3360   // ADC > 3360 - green on constantly border limit2

// LED blink interval
#define LED_BLINK_INTERVAL   500  // 500ms blink period

// Device configuration (set via build flags)
#ifndef DEVICE_ID
#define DEVICE_ID           DEVICE_ID_1  // Default to Device 1
#endif

// Pin Configuration (ESP32-S2)
#define CAN_TX_PIN          5   // GPIO 5
#define CAN_RX_PIN          4   // GPIO 4
#define HALL_SENSOR_PIN     1   // GPIO 1 (A0, ADC)
#define LED_GREEN_PIN       39  // GPIO 39 (Green LED)
#define LED_RED_PIN         40  // GPIO 40 (Red LED)

// State variables
uint8_t current_device_id = DEVICE_ID;
unsigned long last_heartbeat = 0;
unsigned long last_hall_read = 0;
unsigned long last_led_update = 0;

// Hall sensor state
int last_hall_value = 0;
bool red_led_state = false;
bool green_led_state = false;
bool red_blink_mode = false;
bool green_blink_mode = false;
bool limit1_find_sent = false;  // Track if Limit1 Find was sent
bool limit2_find_sent = false;  // Track if Limit2 Find was sent

// TWAI message structure
twai_message_t tx_message;

/**
 * Initialize TWAI (CAN) bus
 */
void setupTWAI() {
  // Configure TWAI pins
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_PIN,
    (gpio_num_t)CAN_RX_PIN,
    TWAI_MODE_NORMAL
  );
  
  // Configure timing (500 kbps)
  twai_timing_config_t t_config = CAN_BITRATE;
  
  // Configure filter to accept all messages (we only send, but need filter for init)
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  
  // Install TWAI driver
  esp_err_t result = twai_driver_install(&g_config, &t_config, &f_config);
  if (result != ESP_OK) {
    Serial.printf("Failed to install TWAI driver: %s\n", esp_err_to_name(result));
    while (1) {
      delay(1000);
    }
  }
  
  // Start TWAI driver
  result = twai_driver_start();
  if (result != ESP_OK) {
    Serial.printf("Failed to start TWAI driver: %s\n", esp_err_to_name(result));
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("TWAI (CAN) bus initialized successfully");
}

/**
 * Check and recover from Bus-Off state
 */
void checkBusRecovery() {
  twai_status_info_t status_info;
  twai_get_status_info(&status_info);
  
  if (status_info.state == TWAI_STATE_BUS_OFF) {
    Serial.println("CAN Bus-Off detected, attempting recovery...");
    twai_initiate_recovery();
    delay(100);  // Wait for recovery
  }
}

/**
 * Send CAN message via TWAI
 */
bool sendCANMessage(uint32_t can_id, uint8_t* data, uint8_t data_len) {
  // Check bus state and recover if needed
  checkBusRecovery();
  
  // Prepare message
  tx_message.identifier = can_id;
  tx_message.data_length_code = data_len;
  tx_message.flags = TWAI_MSG_FLAG_NONE;
  tx_message.self = 0;
  tx_message.reserved = 0;
  
  // Copy data
  for (int i = 0; i < data_len && i < 8; i++) {
    tx_message.data[i] = data[i];
  }
  
  // Send message
  esp_err_t result = twai_transmit(&tx_message, pdMS_TO_TICKS(100));
  
  if (result == ESP_OK) {
    Serial.printf("Sent: CAN ID 0x%03X [", can_id);
    for (int i = 0; i < data_len; i++) {
      Serial.printf("0x%02X", data[i]);
      if (i < data_len - 1) Serial.print(" ");
    }
    Serial.println("]");
    return true;
  } else {
    Serial.printf("Failed to send CAN message: %s\n", esp_err_to_name(result));
    return false;
  }
}

/**
 * Send heartbeat message (1 byte: DeviceID only)
 */
void sendHeartbeat() {
  uint8_t payload[1] = {current_device_id};
  sendCANMessage(CAN_ID_SAFETY, payload, 1);
  // Note: LED feedback removed - using Hall sensor LEDs instead
}

/**
 * Send limit switch trigger message (based on Hall sensor)
 */
void sendLimitSwitchMessage(uint8_t device_id, uint8_t status) {
  uint8_t payload[2] = {device_id, status};
  sendCANMessage(CAN_ID_SAFETY, payload, 2);
}

/**
 * Read Hall sensor ADC value
 */
int readHallSensor() {
  return analogRead(HALL_SENSOR_PIN);
}

/**
 * Update LED states based on Hall sensor value (non-blocking)
 */
void updateLEDs(int hall_value) {
  unsigned long current_time = millis();
  bool led_update_due = (current_time - last_led_update >= LED_BLINK_INTERVAL / 2);
  
  // Determine LED states based on thresholds
  if (hall_value < HALL_RED_ON_THRESHOLD) {
    // Red LED on constantly
    red_blink_mode = false;
    red_led_state = true;
    green_led_state = false;
    green_blink_mode = false;
  } else if (hall_value < HALL_RED_BLINK_THRESHOLD) {
    // Red LED blink
    red_blink_mode = true;
    green_led_state = false;
    green_blink_mode = false;
    if (led_update_due) {
      red_led_state = !red_led_state;  // Toggle for blink
      last_led_update = current_time;
    }
  } else if (hall_value > HALL_GREEN_ON_THRESHOLD) {
    // Green LED on constantly
    green_blink_mode = false;
    green_led_state = true;
    red_led_state = false;
    red_blink_mode = false;
  } else if (hall_value > HALL_GREEN_BLINK_THRESHOLD) {
    // Green LED blink
    green_blink_mode = true;
    red_led_state = false;
    red_blink_mode = false;
    if (led_update_due) {
      green_led_state = !green_led_state;  // Toggle for blink
      last_led_update = current_time;
    }
  } else {
    // Normal range - both LEDs off
    red_led_state = false;
    green_led_state = false;
    red_blink_mode = false;
    green_blink_mode = false;
  }
  
  // Apply LED states
  digitalWrite(LED_RED_PIN, red_led_state ? HIGH : LOW);
  digitalWrite(LED_GREEN_PIN, green_led_state ? HIGH : LOW);
}

/**
 * Check Hall sensor and send CAN messages if thresholds crossed
 */
void checkHallSensor() {
  int hall_value = readHallSensor();
  unsigned long current_time = millis();
  
  // Debounce: only check if enough time has passed
  if (current_time - last_hall_read < 50) {  // 50ms debounce
    return;
  }
  last_hall_read = current_time;
  
  // Check if value crossed thresholds and send CAN messages
  // ADC value ranges (according to thresholds):
  //   < 2160: Red zone (STATUS_MIN_LIMIT 0x10) - border limit1
  //   2160 <= ADC < 2460: Red blink zone (STATUS_LIMIT1_FIND 0x11) - approaching min limit
  //   2460 <= ADC <= 2860: Normal zone
  //   2860 < ADC <= 3360: Green blink zone (STATUS_LIMIT2_FIND 0x12) - approaching max limit
  //   > 3360: Green zone (STATUS_MAX_LIMIT 0x20) - border limit2
  
  // Min limit (red zone: ADC < 2160) - border limit1, send STATUS_MIN_LIMIT 0x10
  if (hall_value < HALL_RED_ON_THRESHOLD) {
    if (last_hall_value >= HALL_RED_ON_THRESHOLD) {
      // Just entered red zone (min limit fully triggered)
      sendLimitSwitchMessage(current_device_id, STATUS_MIN_LIMIT);
      limit1_find_sent = false;  // Reset when min limit is reached
    }
  }
  
  // Limit1 Find (red blink zone: 2160 <= ADC < 2460) - send STATUS_LIMIT1_FIND 0x11
  if (hall_value >= HALL_RED_ON_THRESHOLD && hall_value < HALL_RED_BLINK_THRESHOLD) {
    if (last_hall_value < HALL_RED_ON_THRESHOLD || last_hall_value >= HALL_RED_BLINK_THRESHOLD) {
      // Just entered red blink zone (approaching min limit)
      if (!limit1_find_sent) {
        sendLimitSwitchMessage(current_device_id, STATUS_LIMIT1_FIND);
        limit1_find_sent = true;
      }
    }
  } else {
    // Reset flag when leaving red blink zone
    if (hall_value < HALL_RED_ON_THRESHOLD || hall_value >= HALL_RED_BLINK_THRESHOLD) {
      limit1_find_sent = false;
    }
  }
  
  // Limit2 Find (green blink zone: 2860 < ADC <= 3360) - send STATUS_LIMIT2_FIND 0x12
  if (hall_value > HALL_GREEN_BLINK_THRESHOLD && hall_value <= HALL_GREEN_ON_THRESHOLD) {
    if (last_hall_value <= HALL_GREEN_BLINK_THRESHOLD || last_hall_value > HALL_GREEN_ON_THRESHOLD) {
      // Just entered green blink zone (approaching max limit)
      if (!limit2_find_sent) {
        sendLimitSwitchMessage(current_device_id, STATUS_LIMIT2_FIND);
        limit2_find_sent = true;
      }
    }
  } else {
    // Reset flag when leaving green blink zone
    if (hall_value <= HALL_GREEN_BLINK_THRESHOLD || hall_value > HALL_GREEN_ON_THRESHOLD) {
      limit2_find_sent = false;
    }
  }
  
  // Max limit (green zone: ADC > 3360) - border limit2, send STATUS_MAX_LIMIT 0x20
  if (hall_value > HALL_GREEN_ON_THRESHOLD) {
    if (last_hall_value <= HALL_GREEN_ON_THRESHOLD) {
      // Just entered green zone (max limit fully triggered)
      sendLimitSwitchMessage(current_device_id, STATUS_MAX_LIMIT);
      limit2_find_sent = false;  // Reset when max limit is reached
    }
  }
  
  last_hall_value = hall_value;
  
  // Update LEDs
  updateLEDs(hall_value);
}


/**
 * Setup function
 */
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("ESP32-S2 Safety Node Starting...");
  Serial.printf("Device ID: 0x%02X\n", current_device_id);
  
  // Configure pins
  pinMode(HALL_SENSOR_PIN, INPUT);  // ADC input
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  
  // Initialize ADC
  analogReadResolution(12);  // 12-bit ADC (0-4095)
  
  // Initialize TWAI (CAN) bus
  setupTWAI();
  
  // Enable hardware watchdog timer (2 second timeout)
  esp_task_wdt_init(WDT_TIMEOUT_MS / 1000, true);
  esp_task_wdt_add(NULL);
  
  // Read initial Hall sensor value
  last_hall_value = readHallSensor();
  Serial.printf("Initial Hall sensor value: %d\n", last_hall_value);
  
  // Send initial heartbeat
  sendHeartbeat();
  last_heartbeat = millis();
  last_hall_read = millis();
  last_led_update = millis();
  
  Serial.println("Safety Node ready");
}

/**
 * Main loop (non-blocking)
 */
void loop() {
  // Feed watchdog
  esp_task_wdt_reset();
  
  unsigned long current_time = millis();
  
  // Task 1: Heartbeat Generator (every 5 seconds)
  if (current_time - last_heartbeat >= HEARTBEAT_INTERVAL) {
    sendHeartbeat();
    last_heartbeat = current_time;
  }
  
  // Task 2: Hall Sensor Monitor (with LED control)
  checkHallSensor();
  
  // Non-blocking: yield to other tasks
  // No delay() calls - fully non-blocking loop
}

