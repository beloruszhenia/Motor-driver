# ESP32-S2 Safety Node - Electrical Datasheet

## Hardware Overview

**Board:** Wemos/Lolin S2 Mini (ESP32-S2FN4R2)  
**Function:** Safety monitoring node with Hall sensor and CAN bus communication  
**CAN Protocol:** Safety Node Protocol (ID: 0x005) at 500 kbps

---

## Pin Configuration

### CAN Bus Interface (TWAI)

| Function | GPIO Pin | Direction | Notes             |
| -------- | -------- | --------- | ----------------- |
| CAN TX   | GPIO 5   | Output    | TWAI transmit pin |
| CAN RX   | GPIO 4   | Input     | TWAI receive pin  |

**CAN Transceiver Required:** External CAN transceiver (e.g., MCP2551, SN65HVD230, TJA1051)

### Hall Sensor Input

| Function    | GPIO Pin    | Type         | Notes               |
| ----------- | ----------- | ------------ | ------------------- |
| Hall Sensor | GPIO 1 (A0) | Analog Input | 12-bit ADC (0-4095) |

### LED Outputs

| Function  | GPIO Pin | Type           | Notes            |
| --------- | -------- | -------------- | ---------------- |
| Green LED | GPIO 39  | Digital Output | Status indicator |
| Red LED   | GPIO 40  | Digital Output | Status indicator |

---

## Electrical Specifications

### Power Supply

- **Voltage:** 3.3V (via USB or external 3.3V regulator)
- **Current:** ~80-150mA typical (depends on CAN transceiver and LEDs)
- **USB:** 5V input, onboard regulator to 3.3V

### GPIO Electrical Characteristics

- **Logic High:** 2.4V minimum (3.3V nominal)
- **Logic Low:** 0.4V maximum
- **Input Voltage Range:** 0V to 3.3V (GPIO 1 ADC: 0V to 3.3V)
- **Output Current:** 12mA per GPIO (40mA max per GPIO)
- **ADC Resolution:** 12-bit (4096 steps, 0-3.3V range)
- **ADC Input Impedance:** ~1MΩ

### CAN Bus Specifications

- **Bitrate:** 500,000 bps (500 kbps)
- **Protocol:** CAN 2.0A (Standard 11-bit ID)
- **Termination:** 120Ω resistor required at each end of CAN bus
- **Differential Voltage:** ±1.5V minimum (CAN_H - CAN_L)

### Hall Sensor Interface

- **Input Range:** 0V to 3.3V (ADC: 0-4095)
- **Thresholds:**
  - Red LED ON: ADC < 2160 (~1.74V)
  - Red LED BLINK: ADC < 2460 (~1.99V)
  - Green LED BLINK: ADC > 2860 (~2.31V)
  - Green LED ON: ADC > 3360 (~2.72V)

### LED Specifications

- **Type:** External LEDs (Green and Red)
- **Forward Voltage:** ~2.0-2.2V (typical)
- **Forward Current:** 10-20mA recommended
- **Current Limiting:** External resistor required (220Ω-470Ω recommended)

---

## Circuit Diagram

### Block Diagram

```
                    ┌─────────────────────────────────────┐────────────────┐
                    │      ESP32-S2 Mini Board            │                │
                    │                                     │                │
    ┌───────────────┼───────────────┐                     │                │
    │               │               │                     │                │
┌───▼───┐      ┌────▼────┐     ┌────▼────┐          ┌─────▼─────┐  ┌───────▼────┐
│GPIO 5 │      │ GPIO 4  │     │ GPIO 1  │          │  GPIO 39  │  │   GPIO 40  │
│CAN TX │      │ CAN RX  │     │  (A0)   │          │  (LED G)  │  │   (LED R)  │
└───┬───┘      └────┬────┘     └────┬────┘          └─────┬─────┘  └─────┬──────┘
    │               │               │
    │   ┌───────────           ┌────▼─────┐
    │   │                      │   Hall   │
    │   │                      │  Sensor  │
    │   │                      │          │
    │   │                      │  Signal ─┼
┌───▼───▼──────┐               │  VCC     │
│   CAN        │               │  GND     │
│ Transceiver  │               └──────────┘
│              │
│  TXD ◄───────┘
│  RXD ────────►
│  CANH ───────┐
│  CANL ───────┤
└───────┬──────┘
        │
        │
   ┌────▼─────┐
   │   CAN    │
   │   Bus    │
   │          │
   │ CAN_H    │
   │ CAN_L    │
   │ 3.3V     │
   │ Gnd      │
   └──────────┘

```

### Detailed Wiring Diagram

#### CAN Bus Connection

```
ESP32-S2                    CAN Transceiver              CAN Bus
┌─────────┐                 ┌──────────────┐            ┌─────────┐
│ GPIO 5  │───────────────► │ TXD          │            │         │
│ (CAN TX)│                 │              │            │   CAN   │
│         │                 │              │            │   Bus   │
│ GPIO 4  │◄──────────────  │ RXD          │            │         │
│ (CAN RX)│                 │              │            │         │
│         │                 │              │            │         │
│  GND    │─────────────────│ GND          │            │   GND   │
│         │                 │              │            │         │
│ 3.3V    │─────────────────│ VCC          │            │         │
│         │                 │              │            │         │
│         │                 │ CANH ────────┼───────────►│ CAN_H   │
│         │                 │              │            │         │
│         │                 │ CANL ────────┼───────────►│ CAN_L   │
└─────────┘                 └──────────────┘            └─────────┘
                                                              │
                                                         120Ω │
                                                          ────┘
                                                         (Termination)
```

#### Hall Sensor Connection

```
ESP32-S2                    Hall Sensor
┌─────────┐                 ┌──────────────┐
│ GPIO 1  │◄────────────────│ Signal Out   │
│  (A0)   │                 │              │
│         │                 │              │
│  GND    │─────────────────│ GND          │
│         │                 │              │
│ 3.3V    │─────────────────│ VCC          │
└─────────┘                 └──────────────┘

Note: Hall sensor output typically 0-5V or 0-3.3V
      May require voltage divider if sensor > 3.3V
```

#### LED Connections

```
ESP32-S2                    Green LED                    Red LED
┌─────────┐                 ┌─────────┐                 ┌─────────┐
│ GPIO 39 │─────────[R]─────│  Anode  │                 │         │
│ (LED G) │     (220-470Ω)  │         │                 │         │
│         │                 │         │                 │         │
│         │                 │  Cathode│─────────────────│  GND    │
│         │                 └─────────┘                 │         │
│         │                                             │         │
│ GPIO 40 │─────────────────────────────────────────────│  Anode  │
│ (LED R) │                                             │         │
│         │                                             │         │
│  GND    │─────────────────────────────────────────────│  Cathode│
└─────────┘                                             └─────────┘
```

**LED Resistor Calculation:**

- Supply: 3.3V
- LED Forward Voltage: ~2.0V
- Desired Current: 10-15mA
- Resistor: R = (3.3V - 2.0V) / 0.015A ≈ 87Ω
- **Recommended:** 220Ω (safer, ~6mA) or 150Ω (~8.7mA)

---

## Component Specifications

### CAN Transceiver Options

#### Option 1: MCP2551

- **Supply Voltage:** 4.5V to 5.5V (requires 5V supply)
- **Interface:** TTL/CMOS compatible
- **Speed:** Up to 1 Mbps
- **Package:** DIP-8, SOIC-8

#### Option 2: SN65HVD230 (Recommended for 3.3V)

- **Supply Voltage:** 3.0V to 3.6V (3.3V compatible)
- **Interface:** 3.3V CMOS/TTL compatible
- **Speed:** Up to 1 Mbps
- **Package:** SOIC-8

#### Option 3: TJA1051

- **Supply Voltage:** 4.75V to 5.25V (requires 5V supply)
- **Interface:** TTL/CMOS compatible
- **Speed:** Up to 1 Mbps
- **Package:** SOIC-8

### Hall Sensor Options

- **Analog Output:** 0-5V or 0-3.3V
- **Supply Voltage:** 3.3V or 5V (check datasheet)
- **Output Type:** Linear analog (proportional to magnetic field)
- **Examples:** A1302, A1324, SS49E

**Note:** If sensor output > 3.3V, use voltage divider:

```
Sensor Out ──[R1]──┬── ESP32 GPIO 1
                   │
                  [R2]
                   │
                  GND

R1 = 10kΩ, R2 = 20kΩ (for 5V sensor → 3.3V max)
```

### LED Specifications

- **Type:** Standard 5mm or 3mm LEDs
- **Green LED:** Forward voltage ~2.0-2.2V
- **Red LED:** Forward voltage ~1.8-2.0V
- **Current:** 10-20mA typical
- **Viewing Angle:** 30° to 60°

---

## Power Supply Connections

### USB Power (Default)

```
USB Connector ──► ESP32-S2 Mini ──► 3.3V Regulator ──► VCC (3.3V)
                                      │
                                      ├──► CAN Transceiver VCC
                                      ├──► Hall Sensor VCC
                                      └──► LED Anodes (via resistors)
```

### External Power (Optional)

```
External 3.3V ──► ESP32-S2 VIN ──► 3.3V Regulator ──► VCC (3.3V)
                                      │
                                      ├──► CAN Transceiver VCC
                                      ├──► Hall Sensor VCC
                                      └──► LED Anodes (via resistors)
```

**Note:** If using 5V CAN transceiver (MCP2551, TJA1051), provide separate 5V supply.

---

## CAN Bus Termination

CAN bus requires 120Ω termination resistors at each end of the bus:

```
Device 1 ──[120Ω]── CAN_H ──┬── CAN_H ──[120Ω]── Device 2
                            │
                            │ (Bus length)
                            │
Device 1 ─────────── CAN_L ─┼── CAN_L ──────────── Device 2
```

**Termination Resistor:**

- **Value:** 120Ω ±1%
- **Power Rating:** 0.25W minimum
- **Placement:** One at each physical end of the bus

---

## Complete Wiring Table

| ESP32-S2 Pin | Connected To         | Notes                 |
| ------------ | -------------------- | --------------------- |
| GPIO 5       | CAN Transceiver TXD  | CAN transmit          |
| GPIO 4       | CAN Transceiver RXD  | CAN receive           |
| GPIO 1 (A0)  | Hall Sensor Signal   | Analog input (0-3.3V) |
| GPIO 39      | Green LED (via 220Ω) | Status indicator      |
| GPIO 40      | Red LED (via 220Ω)   | Status indicator      |
| 3.3V         | CAN Transceiver VCC  | Power (if 3.3V type)  |
| 3.3V         | Hall Sensor VCC      | Power                 |
| GND          | CAN Transceiver GND  | Ground                |
| GND          | Hall Sensor GND      | Ground                |
| GND          | LED Cathodes         | Ground                |

---

## Safety Considerations

1. **Voltage Levels:**

   - Never exceed 3.3V on any GPIO pin
   - Use voltage divider for sensors > 3.3V
   - Check CAN transceiver voltage compatibility

2. **Current Limits:**

   - Maximum 12mA per GPIO (40mA absolute max)
   - Use current-limiting resistors for LEDs
   - Do not exceed total board current rating

3. **CAN Bus:**

   - Always use proper termination (120Ω at each end)
   - Use twisted-pair cable for CAN_H/CAN_L
   - Keep bus length reasonable (< 40m for 500 kbps)

4. **ESD Protection:**
   - Consider ESD protection on CAN bus lines
   - Use TVS diodes for industrial environments

---

## Pin Assignment Summary

```
ESP32-S2 Mini Pinout (Relevant Pins Only)

┌─────────────────────────────────────┐
│                                     │
│  [USB]                              │
│                                     │
│  1  [GPIO 1]  A0  ──► Hall Sensor   │
│  4  [GPIO 4]      ──► CAN RX        │
│  5  [GPIO 5]      ──► CAN TX        │
│  39 [GPIO 39]     ──► Green LED     │
│  40 [GPIO 40]     ──► Red LED       │
│                                     │
│  [3.3V] ──► Power                   │
│  [GND]  ──► Ground                  │
│                                     │
└─────────────────────────────────────┘
```

---

## Revision History

| Version | Date | Changes                                          |
| ------- | ---- | ------------------------------------------------ |
| 1.0     | 2024 | Initial release with Hall sensor and LED control |

---

## References

- ESP32-S2 Datasheet: https://www.espressif.com/sites/default/files/documentation/esp32-s2_datasheet_en.pdf
- TWAI Controller: ESP-IDF TWAI Driver Documentation
- CAN 2.0 Specification: ISO 11898
