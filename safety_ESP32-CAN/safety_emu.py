#!/usr/bin/env python3
"""
Safety Node Emulator - CAN Bus Testing Tool

Emulates Safety Node Protocol (CAN ID: 0x005) for testing without hardware.
Supports automatic heartbeat and interactive fault injection via keyboard.

Usage:
    python safety_emu.py [1|2]
    
    Where 1 or 2 specifies which device acts as the active heartbeat source.

Keyboard Controls:
    '1' - Trigger Device 1 Min Limit (0x005 [0x01, 0x10])
    '2' - Trigger Device 1 Max Limit (0x005 [0x01, 0x20])
    '3' - Trigger Device 1 Limit1 Find (0x005 [0x01, 0x11])
    '4' - Trigger Device 1 Limit2 Find (0x005 [0x01, 0x12])

    '5' - Trigger Device 2 Min Limit (0x005 [0x02, 0x10])
    '6' - Trigger Device 2 Max Limit (0x005 [0x02, 0x20])
    '7' - Trigger Device 2 Limit1 Find (0x005 [0x02, 0x11])
    '8' - Trigger Device 2 Limit2 Find (0x005 [0x02, 0x12])

    'q' - Quit
"""

import sys
import time
import select
import termios
import tty
import can
import argparse
import subprocess

# CAN Configuration
CAN_ID_SAFETY = 0x005
DEVICE_ID_1 = 0x01
DEVICE_ID_2 = 0x02
STATUS_OK = 0x00
STATUS_MIN_LIMIT = 0x10
STATUS_MAX_LIMIT = 0x20
STATUS_LIMIT1_FIND = 0x11
STATUS_LIMIT2_FIND = 0x12

# Heartbeat interval: 5 seconds (matches firmware)
HEARTBEAT_INTERVAL = 5.0

# CAN interface (default: can0, matching motor driver)
CAN_INTERFACE = "can0"
CAN_BITRATE = 500000


class SafetyEmulator:
    """Safety Node Emulator for CAN bus testing"""
    
    def __init__(self, device_id: int, interface: str = CAN_INTERFACE, bitrate: int = CAN_BITRATE):
        """
        Initialize the safety emulator
        
        Args:
            device_id: Device ID (1 or 2) for heartbeat source
            interface: CAN interface name (e.g., "can0")
            bitrate: CAN bus bitrate in bits/s
        """
        if device_id not in [1, 2]:
            raise ValueError("Device ID must be 1 or 2")
        
        self.device_id = device_id
        self.interface = interface
        self.bitrate = bitrate
        self.bus = None
        self.running = False
        
        # Save original terminal settings for restoration
        self.old_settings = None
    
    def connect(self):
        """Connect to CAN bus"""
        # Check if interface exists and is up (optional check)
        try:
            result = subprocess.run(
                ["ip", "link", "show", self.interface],
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.returncode != 0:
                print(f"WARNING: CAN interface '{self.interface}' not found.", file=sys.stderr)
                print(f"  Try: sudo ip link set {self.interface} up type can bitrate {self.bitrate}", file=sys.stderr)
            elif "state DOWN" in result.stdout:
                print(f"WARNING: CAN interface '{self.interface}' is DOWN.", file=sys.stderr)
                print(f"  Try: sudo ip link set {self.interface} up", file=sys.stderr)
        except (FileNotFoundError, subprocess.TimeoutExpired):
            # Continue anyway - interface check is optional
            pass
        except Exception:
            # Ignore interface check errors, try connecting anyway
            pass
        
        # Connect to CAN bus
        try:
            self.bus = can.Bus(
                interface="socketcan",
                channel=self.interface,
                bitrate=self.bitrate
            )
            print(f"Connected to CAN bus: {self.interface} at {self.bitrate} bps")
        except can.CanError as e:
            raise RuntimeError(
                f"Failed to connect to CAN bus '{self.interface}': {e}\n"
                f"  Make sure the interface is up: sudo ip link set {self.interface} up type can bitrate {self.bitrate}\n"
                f"  Check permissions: you may need sudo or add user to 'dialout' group"
            ) from e
        except Exception as e:
            raise RuntimeError(f"Failed to connect to CAN bus '{self.interface}': {e}") from e
    
    def disconnect(self):
        """Disconnect from CAN bus"""
        if self.bus:
            self.bus.shutdown()
            self.bus = None
    
    def send_heartbeat(self, device_id: int):
        """
        Send heartbeat message on CAN bus (1 byte: DeviceID only)
        
        Args:
            device_id: Device ID (0x01 or 0x02)
        """
        if not self.bus:
            raise RuntimeError("Not connected to CAN bus")
        
        # Heartbeat payload: [DeviceID] (1 byte)
        payload = bytes([device_id])
        
        # Create and send CAN message
        msg = can.Message(
            arbitration_id=CAN_ID_SAFETY,
            data=payload,
            is_extended_id=False
        )
        
        try:
            self.bus.send(msg, timeout=0.1)
            print(f"Sent: 0x{CAN_ID_SAFETY:03X} [{payload.hex(' ').upper()}]")
        except can.CanError as e:
            print(f"ERROR: Failed to send heartbeat: {e}", file=sys.stderr)
            raise
        except Exception as e:
            print(f"ERROR: Unexpected error sending heartbeat: {e}", file=sys.stderr)
            raise
    
    def send_limit_switch_message(self, device_id: int, status: int):
        """
        Send limit switch trigger message on CAN bus (2 bytes: DeviceID, Status)
        
        Args:
            device_id: Device ID (0x01 or 0x02)
            status: Status code (0x10, 0x20, 0x11, or 0x12)
        """
        if not self.bus:
            raise RuntimeError("Not connected to CAN bus")
        
        # Limit switch payload: [DeviceID, Status] (2 bytes)
        payload = bytes([device_id, status])
        
        # Create and send CAN message
        msg = can.Message(
            arbitration_id=CAN_ID_SAFETY,
            data=payload,
            is_extended_id=False
        )
        
        try:
            self.bus.send(msg, timeout=0.1)
            print(f"Sent: 0x{CAN_ID_SAFETY:03X} [{payload.hex(' ').upper()}]")
        except can.CanError as e:
            print(f"ERROR: Failed to send limit switch message: {e}", file=sys.stderr)
            raise
        except Exception as e:
            print(f"ERROR: Unexpected error sending limit switch message: {e}", file=sys.stderr)
            raise
    
    def setup_keyboard(self):
        """Setup terminal for non-blocking keyboard input"""
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
    
    def restore_keyboard(self):
        """Restore original terminal settings"""
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def get_key(self) -> str:
        """
        Get a key press without blocking (non-blocking)
        
        Returns:
            Key character if pressed, None otherwise
        """
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None
    
    def run(self):
        """Run the emulator main loop"""
        print(f"\nSafety Node Emulator - Device {self.device_id}")
        print("=" * 50)
        print("Keyboard Controls:")
        print("  Device 1:")
        print("    '1' - Trigger Device 1 Min Limit (0x005 [0x01, 0x10])")
        print("    '2' - Trigger Device 1 Max Limit (0x005 [0x01, 0x20])")
        print("    '3' - Trigger Device 1 Limit1 Find (0x005 [0x01, 0x11])")
        print("    '4' - Trigger Device 1 Limit2 Find (0x005 [0x01, 0x12])")
        print("  Device 2:")
        print("    '5' - Trigger Device 2 Min Limit (0x005 [0x02, 0x10])")
        print("    '6' - Trigger Device 2 Max Limit (0x005 [0x02, 0x20])")
        print("    '7' - Trigger Device 2 Limit1 Find (0x005 [0x02, 0x11])")
        print("    '8' - Trigger Device 2 Limit2 Find (0x005 [0x02, 0x12])")
        print("  'q' - Quit")
        print("=" * 50)
        print(f"\nSending heartbeat every {HEARTBEAT_INTERVAL}s as Device {self.device_id}")
        print("Press keys to inject faults...\n")
        
        self.running = True
        self.setup_keyboard()
        last_heartbeat = time.time()
        
        try:
            while self.running:
                current_time = time.time()
                
                # Send heartbeat if interval elapsed
                if current_time - last_heartbeat >= HEARTBEAT_INTERVAL:
                    device_id_byte = DEVICE_ID_1 if self.device_id == 1 else DEVICE_ID_2
                    self.send_heartbeat(device_id_byte)
                    last_heartbeat = current_time
                
                # Check for key press
                key = self.get_key()
                if key:
                    if key == 'q' or key == '\x1b':  # 'q' or ESC
                        print("\nQuitting...")
                        self.running = False
                    # Device 1 controls
                    elif key == '1':
                        self.send_limit_switch_message(DEVICE_ID_1, STATUS_MIN_LIMIT)
                    elif key == '2':
                        self.send_limit_switch_message(DEVICE_ID_1, STATUS_MAX_LIMIT)
                    elif key == '3':
                        self.send_limit_switch_message(DEVICE_ID_1, STATUS_LIMIT1_FIND)
                    elif key == '4':
                        self.send_limit_switch_message(DEVICE_ID_1, STATUS_LIMIT2_FIND)
                    # Device 2 controls
                    elif key == '5':
                        self.send_limit_switch_message(DEVICE_ID_2, STATUS_MIN_LIMIT)
                    elif key == '6':
                        self.send_limit_switch_message(DEVICE_ID_2, STATUS_MAX_LIMIT)
                    elif key == '7':
                        self.send_limit_switch_message(DEVICE_ID_2, STATUS_LIMIT1_FIND)
                    elif key == '8':
                        self.send_limit_switch_message(DEVICE_ID_2, STATUS_LIMIT2_FIND)
                
                # Small sleep to prevent excessive CPU usage
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self.restore_keyboard()
            self.disconnect()


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description="Safety Node Emulator for CAN bus testing",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        "device_id",
        type=int,
        choices=[1, 2],
        help="Device ID for active heartbeat source (1 or 2)"
    )
    parser.add_argument(
        "-i", "--interface",
        type=str,
        default=CAN_INTERFACE,
        help=f"CAN interface name (default: {CAN_INTERFACE})"
    )
    parser.add_argument(
        "-b", "--bitrate",
        type=int,
        default=CAN_BITRATE,
        help=f"CAN bus bitrate in bits/s (default: {CAN_BITRATE})"
    )
    
    args = parser.parse_args()
    
    emulator = SafetyEmulator(
        device_id=args.device_id,
        interface=args.interface,
        bitrate=args.bitrate
    )
    
    try:
        emulator.connect()
        emulator.run()
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()

