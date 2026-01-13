# GIM8115 Motor Driver for SocketCAN

Python driver for controlling GIM8115 motors via Linux SocketCAN interface, implementing the SteadyWin GIM Protocol Specification.

## Features

- **SocketCAN Integration**: Native Linux CAN bus support
- **Full Protocol Implementation**: Position, velocity, and torque control modes
- **Type-Safe**: Strict type hints and dataclasses with `__slots__`
- **Memory Efficient**: Pre-allocated buffers, no dynamic allocation in control loops
- **Error Handling**: Comprehensive exception handling for motor errors
- **Feedback Parsing**: Automatic decoding of motor status frames

## Requirements

- Python 3.7+
- Linux with SocketCAN support
- `python-can` library
- CAN interface configured (e.g., `can0`)

## Installation

```bash
pip install -r requirements.txt
```

## Quick Start

```python
from gim8115_driver import GIM8115Driver
import math

# Initialize driver
motor = GIM8115Driver(
    interface="can0",
    can_id=0x01,
    torque_constant=0.1,  # N⋅m/A (from motor datasheet)
    gear_ratio=10.0       # Gear ratio
)

# Connect to CAN bus
motor.connect()

try:
    # Start motor
    motor.start_motor()

    # Move to 45 degrees in 2 seconds
    motor.send_position(angle_rad=math.pi/4, duration_ms=2000)

    # Receive feedback
    response = motor._receive_frame(timeout=2.5)
    if response:
        status = motor.parse_feedback(response)
        print(f"Position: {math.degrees(status.position_rad):.2f}°")
        print(f"Speed: {status.speed_rads:.2f} rad/s")

    # Stop motor
    motor.stop_motor()
finally:
    motor.disconnect()
```

## Context Manager Usage

```python
with GIM8115Driver(interface="can0", can_id=0x01) as motor:
    motor.start_motor()
    motor.send_position(math.pi/2, duration_ms=1000)
    # Automatic disconnect on exit
```

## API Reference

### GIM8115Driver

#### Constructor Parameters

- `interface` (str): CAN interface name (default: `"can0"`)
- `can_id` (int): CAN ID for the motor (default: `0x01`)
- `bitrate` (int): CAN bus bitrate in bits/s (default: `1000000`)
- `torque_constant` (float): Motor torque constant KT in N⋅m/A
- `gear_ratio` (float): Gear ratio (output/input)

#### Methods

- `connect()`: Connect to CAN bus
- `disconnect()`: Disconnect from CAN bus
- `start_motor()`: Start the motor
- `stop_motor()`: Stop the motor
- `send_position(angle_rad: float, duration_ms: int)`: Position control
  - `duration_ms`: Total time from start to stop (acceleration + deceleration). Example: 100ms = 50ms accel + 50ms decel
- `send_velocity(speed_rads: float, duration_ms: int)`: Velocity control
  - `duration_ms`: Total time from start to stop (acceleration + deceleration). Example: 100ms = 50ms accel + 50ms decel
- `send_torque(torque_nm: float, duration_ms: int)`: Torque control
  - `duration_ms`: Total time from start to stop (acceleration + deceleration). Example: 100ms = 50ms accel + 50ms decel
- `set_zero_position()`: Set current position as zero
- `parse_feedback(data: bytes, check_result: bool = True) -> MotorStatus`: Parse feedback frame
- `_receive_frame(timeout: float = 1.0) -> Optional[bytes]`: Receive CAN frame
- `_send_frame(payload: bytes)`: Send CAN frame

### MotorStatus

Dataclass containing motor feedback:

- `command_echo` (int): Echoed command byte
- `result_code` (int): Result code (0x00 = success)
- `temperature` (int): Temperature in Celsius
- `position_rad` (float): Position in radians (-12.5 to +12.5)
- `speed_rads` (float): Speed in rad/s (-65 to +65)
- `torque_nm` (float): Torque in N⋅m (calculated using KT and gear ratio)
- `torque_raw` (float): Raw torque in Amperes (-225 to +225)

### Exceptions

- `GIM8115Error`: Base exception for driver errors
- `GIM8115ResultError`: Raised when motor returns non-success result code

## Protocol Details

### Position Control Frame

- **Byte 0**: `0x95` (Command)
- **Bytes 1-4**: Target position (float, little-endian, radians)
- **Bytes 5-7**: Duration (24-bit unsigned int, little-endian, milliseconds)
  - Total time from start to stop (acceleration + deceleration)
  - Example: 100ms = 50ms acceleration + 50ms deceleration
  - Use 0 for immediate execution (max acceleration)

### Feedback Frame

- **Byte 0**: Command echo
- **Byte 1**: Result code (`0x00` = success)
- **Byte 2**: Temperature (int8)
- **Bytes 3-4**: Position (uint16, compressed)
- **Bytes 5-7**: Speed & Torque (compressed 12-bit values)

### Decoding Formulas

**Position:**

```
pos_rad = (uint16_pos * 25.0 / 65535.0) - 12.5
```

**Speed:**

```
speed_int = (ST0 << 4) | (ST1 >> 4)
speed_rads = (speed_int * 130.0 / 4095.0) - 65.0
```

**Torque:**

```
torque_int = ((ST1 & 0x0F) << 8) | ST2
torque_nm = (torque_int * (450 * KT * GEAR) / 4095.0) - (225 * KT * GEAR)
```

## CAN Bus Setup

Before using the driver, ensure your CAN interface is configured:

```bash
# Bring up CAN interface
sudo ip link set can0 up type can bitrate 1000000

# Check interface status
ip link show can0
```

## Examples

See `example_usage.py` for comprehensive examples including:

- Position control
- Velocity control
- Torque control
- Continuous control loops
- Feedback parsing

## License

This implementation follows the SteadyWin GIM Protocol Specification.
