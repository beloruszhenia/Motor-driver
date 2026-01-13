"""
Status Monitoring Example

This script demonstrates how to monitor motor status (position, speed, error) at 10 Hz.

Usage:
    python3 status_monitor_example.py
"""

import time
import math
from gim8115_driver import (
    GIM8115Driver, 
    GIM8115Error,
    MotorStatus
)


# Global variable to access motor instance in callback
_motor_instance = None

def status_callback(status: MotorStatus):
    """
    Callback function called 10 times per second with motor status
    
    Args:
        status: MotorStatus object containing position, speed, error, etc.
    """
    global _motor_instance
    
    # Calculate relative position (relative to zero) using position_offset
    # Position from status is absolute, we need to subtract offset to get relative position
    if _motor_instance is not None:
        position_abs = status.position_rad
        position_offset = _motor_instance.get_position_offset()
        position_rel = position_abs - position_offset
    else:
        position_rel = status.position_rad  # Fallback to absolute if motor not available
    
    # Check for errors (result_code from status)
    error_str = "OK" if status.result_code == 0x00 else f"Error: 0x{status.result_code:02X}"
    
    # Print status with relative position
    print(f"Status: Pos={math.degrees(position_rel):7.2f}° (abs: {math.degrees(status.position_rad):7.2f}°) | "
          f"Speed={status.speed_rads:6.2f} rad/s ({math.degrees(status.speed_rads):6.2f}°/s) | "
          f"{error_str}")


def main():
    """
    Example of monitoring motor status at 10 Hz
    """
    print("=" * 60)
    print("Motor Status Monitoring Example (10 Hz)")
    print("=" * 60)
    
    # Motor parameters (adjust based on your motor specifications)
    TORQUE_CONSTANT = 1  # N⋅m/A (example value, check motor datasheet)
    GEAR_RATIO = 36  # Gear ratio
    
    global _motor_instance
    
    try:
        with GIM8115Driver(
            interface="can0",
            can_id=0x0A,
            bitrate=500000,
            torque_constant=TORQUE_CONSTANT,
            gear_ratio=GEAR_RATIO
        ) as motor:
            _motor_instance = motor  # Store for use in callback
            
            print("\n1. Starting motor...")
            motor.start_motor()
            time.sleep(0.3)
            print("   ✓ Motor started")
            
            print("\n2. Starting status monitor at 10 Hz...")
            print("   Monitoring: position, speed, error status")
            print("   (Press Ctrl+C to stop)")
            print()
            motor.start_status_monitor(rate_hz=10.0, callback=status_callback)
            print("   ✓ Status monitor started")
            print("   Status updates will appear below (10 times per second):\n")
            
            # Keep running to receive continuous status updates
            # The status monitor runs in a background thread and calls the callback
            try:
                loop_count = 0
                while True:
                    time.sleep(1.0)  # Sleep 1 second at a time
                    loop_count += 1
                    # Print a heartbeat every 10 seconds to show we're still running
                    if loop_count % 10 == 0:
                        print(f"   [Heartbeat: Still monitoring... ({loop_count}s elapsed)]")
            except KeyboardInterrupt:
                print("\n\n   Stopping status monitor...")
            
            print("\n3. Stopping status monitor...")
            motor.stop_status_monitor()
            print("   ✓ Status monitor stopped")
            
            print("\n4. Stopping motor...")
            motor.stop_motor()
            print("   ✓ Motor stopped")
            
            print("\n✓ Status monitoring completed!")
            
    except GIM8115Error as e:
        print(f"\n✗ GIM8115 Error: {e}")
        print("   Make sure:")
        print("   - Motor is connected and powered")
        print("   - CAN bus is properly configured")
    except KeyboardInterrupt:
        print("\n\n✗ Interrupted by user")
        print("   Status monitor should be stopped automatically")
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()


def manual_polling_example():
    """
    Alternative example: Manual polling at 10 Hz (without background thread)
    """
    print("=" * 60)
    print("Manual Status Polling Example (10 Hz)")
    print("=" * 60)
    
    TORQUE_CONSTANT = 1
    GEAR_RATIO = 36
    
    try:
        with GIM8115Driver(
            interface="can0",
            can_id=0x0A,
            bitrate=500000,
            torque_constant=TORQUE_CONSTANT,
            gear_ratio=GEAR_RATIO
        ) as motor:
            
            print("\n1. Starting motor...")
            motor.start_motor()
            time.sleep(0.3)
            
            print("\n2. Polling status manually at 10 Hz...")
            print("   (Press Ctrl+C to stop)")
            print()
            
            poll_interval = 0.1  # 10 Hz = 100ms = 0.1 seconds
            start_time = time.time()
            
            try:
                while True:
                    # Get status
                    status = motor.get_motor_status(timeout=0.05)
                    
                    if status is not None:
                        position_rel = status.position_rad
                        error_str = "OK" if status.result_code == 0x00 else f"Error: 0x{status.result_code:02X}"
                        
                        elapsed = time.time() - start_time
                        print(f"[{elapsed:6.2f}s] Pos={math.degrees(position_rel):6.2f}° | "
                              f"Speed={status.speed_rads:6.2f} rad/s | "
                              f"Temp={status.temperature:3d}°C | "
                              f"{error_str}")
                    else:
                        print("   No status received")
                    
                    time.sleep(poll_interval)
                    
            except KeyboardInterrupt:
                print("\n   Polling stopped by user")
            
            motor.stop_motor()
            print("\n✓ Manual polling example completed!")
            
    except GIM8115Error as e:
        print(f"\n✗ GIM8115 Error: {e}")


if __name__ == "__main__":
    # Run callback-based example
    main()
    
    # Uncomment to run manual polling example instead:
    # manual_polling_example()

