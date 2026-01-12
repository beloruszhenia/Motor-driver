# ESP32-S2 Safety Node - TWAI CAN Bus Implementation

ESP32-S2 based safety node implementing the Safety Node Protocol (CAN ID: 0x005) for limit switch monitoring and safety-critical applications.

## Overview

This project implements a safety monitoring node that:

- Monitors limit switches (min/max limits) with software debounce
- Sends periodic heartbeat messages on CAN bus (every 5 seconds)
- Reports safety status changes immediately upon detection
- Supports two device IDs for multiple safety zones
- Includes hardware watchdog timer and CAN bus recovery

## Hardware

- **Board:** Wemos/Lolin S2 Mini (ESP32-S2FN4R2)
- **Framework:** PlatformIO (Arduino Core)
- **CAN Interface:** Internal TWAI (Two-Wire Automotive Interface) - requires external transceiver
- **Inputs:** 2x Mechanical Limit Switches (Internal Pull-up, Active Low)
- **Status LED:** Onboard LED (GPIO 15) for heartbeat visualization

## Safety Node Protocol

- **CAN ID:** `0x005` (Standard 11-bit, highest priority)
- **Bitrate:** 500,000 bps (500 kbps) - matches Orin NX setting

### Packet Structure

#### Heartbeat Message (Task 1)

- **Frequency:** Every 5000 ms (5 seconds)
- **Format:** `ID: 0x005` | Payload: `[DeviceID]` (Length: 1 byte)
- **Example:** Device 1 heartbeat = `0x005 [0x01]`

#### Limit Switch Trigger (Task 2)

- **Format:** `ID: 0x005` | Payload: `[DeviceID, Status]` (Length: 2 bytes)
- **Trigger Logic:**
  - Switch 1 goes LOW (Active) → Send `[DeviceID, 0x10]` (Min Limit)
  - Switch 2 goes LOW (Active) → Send `[DeviceID, 0x20]` (Max Limit)
- **Priority:** Sent immediately upon detection (bypasses heartbeat timer)

### Device IDs

- `0x01` = Safety Device 1 (e.g., Left/Yaw Limit)
- `0x02` = Safety Device 2 (e.g., Up/Pitch Limit)

### Status Codes

- `0x10` = Min Limit / Warning 1 (Switch 1 triggered)
- `0x20` = Max Limit / Warning 2 (Switch 2 triggered)

## Pin Configuration

| Function       | GPIO Pin | Notes                             |
| -------------- | -------- | --------------------------------- |
| CAN TX         | GPIO 5   | TWAI transmit                     |
| CAN RX         | GPIO 3   | TWAI receive                      |
| Limit Switch 1 | GPIO 7   | Active LOW (pull-up enabled)      |
| Limit Switch 2 | GPIO 9   | Active LOW (pull-up enabled)      |
| Status LED     | GPIO 15  | Onboard LED (blinks on heartbeat) |

_Note: ESP32-S2 allows PIN matrix remapping, but pins are strictly defined in code constants._

## Software Components

### 1. ESP32-S2 Firmware (`src/main.cpp`)

Arduino-based firmware using native TWAI driver that:

- Initializes TWAI (CAN) bus at 500 kbps
- Monitors limit switches with 50ms software debounce
- Sends heartbeat every 5 seconds (1-byte payload)
- Immediately reports limit switch triggers (2-byte payload)
- Implements hardware watchdog timer (2 second timeout)
- Auto-recovers from CAN Bus-Off state
- Non-blocking loop (no `delay()` calls)

**Key Features:**

- **Debounce:** 50ms stable state required before trigger
- **Watchdog:** Hardware WDT restarts if loop hangs > 2s
- **Bus Recovery:** Auto-recovery from Bus-Off after 100ms
- **LED Feedback:** Blinks LED briefly on successful heartbeat send

**Build Configuration:**

- Device ID is set via build flag `DEVICE_ID` in `platformio.ini`
- Default: Device 1 (0x01)
- Change to Device 2: Set `-D DEVICE_ID=0x02` in build flags

### 2. Python Emulator (`safety_emu.py`)

Testing tool for validating safety logic without hardware:

**Features:**

- Automatic heartbeat generation (Device 1 or 2)
- Interactive keyboard control for fault injection
- Non-blocking key detection (no Enter key required)
- Real-time CAN message logging

**Usage:**

```bash
# Install dependencies
pip install -r requirements.txt

# Run as Device 1 (heartbeat source)
python safety_emu.py 1

# Run as Device 2 (heartbeat source)
python safety_emu.py 2

# Custom CAN interface
python safety_emu.py 1 -i can1 -b 500000
```

**Keyboard Controls:**

- `1` - Trigger Device 1 Min Limit (`0x005 [0x01, 0x10]`)
- `2` - Trigger Device 1 Max Limit (`0x005 [0x01, 0x20]`)
- `3` - Trigger Device 2 Min Limit (`0x005 [0x02, 0x10]`)
- `4` - Trigger Device 2 Max Limit (`0x005 [0x02, 0x20]`)
- `q` - Quit

## Building and Flashing

### Prerequisites

1. Install PlatformIO:

   ```bash
   pip install platformio
   ```

2. Connect ESP32-S2 Mini via USB

### Build and Upload

```bash
# Build firmware
pio run

# Upload to ESP32-S2
pio run -t upload

# Monitor serial output
pio device monitor
```

### Configure Device ID

Edit `platformio.ini` and modify the build flag:

```ini
build_flags =
    -D DEVICE_ID=0x01  # Change to 0x02 for Device 2
```

## Testing

### 1. Hardware Testing

1. Connect ESP32-S2 to CAN bus via transceiver
2. Connect limit switches to GPIO 7 and GPIO 9
3. Monitor CAN bus:
   ```bash
   candump can0
   ```
4. Observe heartbeat messages every 5 seconds
5. Trigger limit switches to verify immediate status messages

### 2. Emulator Testing

1. Start CAN interface (if needed):

   ```bash
   sudo ip link set can0 up type can bitrate 500000
   ```

2. Run emulator:

   ```bash
   python safety_emu.py 1
   ```

3. Monitor CAN bus in another terminal:

   ```bash
   candump can0
   ```

4. Press keys 1-4 to inject faults and verify messages

## Project Structure

```
safety_ESP32-CAN/
├── .cursorrules          # Project specification
├── platformio.ini        # PlatformIO configuration
├── requirements.txt      # Python dependencies
├── README.md            # This file
├── safety_emu.py        # Python emulator tool
└── src/
    └── main.cpp         # ESP32-S2 firmware
```

## Firmware Logic Rules

### Non-Blocking Implementation

- Main loop uses no `delay()` calls
- All timing uses `millis()` comparisons
- Fully non-blocking for responsive limit switch detection

### CAN Configuration

- **Library:** Native ESP-IDF `driver/twai.h` (built-in)
- **Bitrate:** 500 kbps (matches Orin NX)
- **Mode:** Normal Mode

### Failsafe Features

- **Watchdog:** Hardware WDT enabled (2 second timeout)
- **Bus Recovery:** Automatic recovery from Bus-Off state (100ms delay)
- **Debounce:** Software debounce (50ms stable state) prevents false triggers

## Integration Notes

- **Priority:** Safety Node Protocol has highest priority (ID: 0x005)
- **Heartbeat:** 5 second interval reduces bus traffic while maintaining safety
- **Status Changes:** Reported immediately with debounce protection
- **Multiple Devices:** Use different Device IDs (0x01, 0x02) for different safety zones
- **CAN Transceiver:** External transceiver required (e.g., MCP2551, SN65HVD230)
- **Termination:** CAN bus requires 120Ω termination resistors at each end

## Troubleshooting

### CAN Bus Issues

- Verify CAN transceiver connections (TX/RX to GPIO 5/3)
- Check termination resistors (120Ω at each end)
- Ensure bitrate matches (500 kbps)
- Use `candump` to verify bus activity
- Check for Bus-Off state in serial monitor

### Limit Switch Issues

- Verify GPIO pin assignments (GPIO 7 and GPIO 9)
- Check pull-up configuration (internal pull-ups enabled)
- Switches are Active LOW (LOW = triggered)
- Verify debounce timing (50ms stable state)

### Watchdog Issues

- If device resets frequently, check for blocking operations
- Ensure `esp_task_wdt_reset()` is called in main loop
- Verify loop execution time < 2 seconds

### Emulator Issues

- Ensure CAN interface is up: `ip link set can0 up`
- Check permissions: may need `sudo` or add user to `dialout` group
- Verify python-can installation: `pip install python-can`

## License

See project root for license information.
