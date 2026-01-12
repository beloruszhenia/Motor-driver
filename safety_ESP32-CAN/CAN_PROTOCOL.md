# Safety Node CAN Protocol

ESP32-S2 Safety Node - передача інформації про граничні положення через CAN шину.

---

## CAN Configuration

- **CAN ID:** `0x005`
- **Bitrate:** `500 kbps` (default, configurable: 125, 250, 500, 800, 1000 kbps)
- **Device IDs:** `0x01` (Device 1), `0x02` (Device 2)

### Bitrate Options

**Зміна бітрейту:** В `platformio.ini` змініть `-D CAN_SPEED_KBPS=500` на потрібне значення

---

## Message Formats

### 1. Heartbeat (Alive Signal)
```
CAN ID: 0x005
DLC: 1
Data: [Device_ID]
```
**Частота:** кожні 5 секунд

**Приклад:** `0x005 [0x01]`

---

### 2. Limit Events
```
CAN ID: 0x005
DLC: 2
Data: [Device_ID, Status_Code]
```

**Status Codes:**

| Code | Name | ADC Range | LED | Опис |
|------|------|-----------|-----|------|
| `0x10` | MIN_LIMIT | < 2160 | 🔴 ON | Досягнуто мінімальний ліміт |
| `0x11` | LIMIT1_FIND | 2160-2459 | 🔴 BLINK | Наближення до min ліміту |
| `0x12` | LIMIT2_FIND | 2861-3360 | 🟢 BLINK | Наближення до max ліміту |
| `0x20` | MAX_LIMIT | > 3360 | 🟢 ON | Досягнуто максимальний ліміт |

### CAN Error Indication

**При помилці передачі CAN (3+ невдалих спроб):**
- LED: 🔴🟢 **ALTERNATE** (почергово червоний/зелений, 2 Hz)
- Pattern: RED ON 250ms → GREEN ON 250ms → repeat
- Авто-відновлення: При успішній відправці повертається нормальний режим

**Приклади:**
- `0x005 [0x01 0x10]` - Device 1, MIN_LIMIT
- `0x005 [0x02 0x12]` - Device 2, LIMIT2_FIND

---

## Hall Sensor Zones

```
ADC:  0        2160      2460  2860      3360        4095
      │─────────┼──────────┼─────┼──────────┼──────────│
      │  0x10   │   0x11   │ OK  │   0x12   │   0x20   │
LED:  │ RED ON  │ RED BLINK│ OFF │GRN BLINK │ GREEN ON │
```

**ADC:** 12-bit (0-4095), зчитування 20 Hz (кожні 50 мс)

---

## Transmission Logic

- ✅ Повідомлення відправляються **ТІЛЬКИ при переході між зонами** (edge detection)
- ✅ Anti-spam: LIMIT1_FIND та LIMIT2_FIND надсилаються 1 раз при вході в зону
- ✅ Heartbeat: незалежно від стану Hall sensor

---

## Timing

| Parameter | Value |
|-----------|-------|
| ADC Sampling | 20 Hz (50 ms) |
| Heartbeat | 5 seconds |
| CAN Timeout | 100 ms |

---

## Python Integration

```python
import can

bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)

for msg in bus:
    if msg.arbitration_id == 0x005:
        device_id = msg.data[0]
        
        if msg.dlc == 1:
            print(f"Device {device_id:02X} alive")
        elif msg.dlc == 2:
            status = msg.data[1]
            events = {0x10: "MIN_LIMIT", 0x11: "Approaching Min", 
                     0x12: "Approaching Max", 0x20: "MAX_LIMIT"}
            print(f"Device {device_id:02X}: {events.get(status, 'Unknown')}")
```

---

## LED Status Indicators

| LED Pattern | Стан | Опис |
|-------------|------|------|
| 🔴 ON | MIN_LIMIT | Досягнуто мінімальний ліміт |
| 🔴 BLINK (500ms) | LIMIT1_FIND | Наближення до мінімуму |
| 🟢 BLINK (500ms) | LIMIT2_FIND | Наближення до максимуму |
| 🟢 ON | MAX_LIMIT | Досягнуто максимальний ліміт |
| OFF | NORMAL | Нормальна зона |
| 🔴🟢 ALTERNATE (250ms) | **CAN ERROR** | Помилка CAN шини (3+ failures) |

---

## Hardware Pins (ESP32-S2)

| GPIO | Function |
|------|----------|
| 5 | CAN TX |
| 4 | CAN RX |
| 1 | Hall Sensor (ADC) |
| 39 | Green LED |
| 40 | Red LED |

**CAN Transceiver:** SN65HVD230 або TJA1050 (120Ω termination)

