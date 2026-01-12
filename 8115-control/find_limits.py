"""
Find Position Limits Example

This script demonstrates how to automatically find position limits by rotating
slowly until limit switch events are detected.

Usage:
    python3 find_limits.py
"""

import time
import math
from gim8115_driver import (
    GIM8115Driver, 
    GIM8115Error, 
    GIM8115ResultError
)


def main():
    """
    Automatically find position limits by rotating slowly until limit events
    
    This example demonstrates how to use find_position_limits() to automatically
    calibrate the physical limits of the motor system.
    """
    print("=" * 60)
    print("Finding Position Limits Automatically")
    print("=" * 60)
    
    # Motor parameters (adjust based on your motor specifications)
    TORQUE_CONSTANT = 1  # N⋅m/A (example value, check motor datasheet)
    GEAR_RATIO = 10.0  # Example gear ratio
    
    try:
        with GIM8115Driver(
            interface="can0",
            can_id=0x0A,
            bitrate=500000,
            torque_constant=TORQUE_CONSTANT,
            gear_ratio=GEAR_RATIO
        ) as motor:
            
            print("\n1. Setting zero position...")
            motor.set_zero_position()
            print("   ✓ Zero position set")
            
            print("\n2. Starting safety listener...")
            print("   This monitors CAN ID 0x005 for limit switch events")
            motor.start_safety_listener(auto_stop=True)
            print("   ✓ Safety listener started")
            
            print("\n3. Finding position limits...")
            print("   The motor will:")
            print("   - Rotate left until safety limit1 (0x11) is detected")
            print("   - Then rotate right until safety limit2 (0x12) is detected")
            print("   - Apply ±5° safety shift to create safe operating limits")
            print("   - Limits will be automatically saved to config")
            print("   Starting now...")
            
            # Find limits using configured speed from config file
            # Speed can be configured via: motor.set_limit_find_speed(speed_rads)
            print(f"   Using rotation speed: {motor.get_limit_find_speed():.2f} rad/s (~{math.degrees(motor.get_limit_find_speed()):.1f} deg/s)")
            min_limit, max_limit = motor.find_position_limits(
                timeout_seconds=60.0,     # Max 60 seconds per limit
                check_interval=0.05       # Check safety messages every 50ms (20 Hz)
            )
            
            print("\n4. Limits found and saved!")
            print(f"   Minimum limit: {math.degrees(min_limit):.2f}° ({min_limit:.4f} rad)")
            print(f"   Maximum limit: {math.degrees(max_limit):.2f}° ({max_limit:.4f} rad)")
            print(f"   Total range: {math.degrees(max_limit - min_limit):.2f}°")
            
            # Verify limits
            saved_min, saved_max = motor.get_position_limits()
            print(f"\n   Verified limits from config:")
            print(f"   Min: {math.degrees(saved_min):.2f}°")
            print(f"   Max: {math.degrees(saved_max):.2f}°")
            
            print("\n5. Testing limits...")
            print("   Moving to minimum limit...")
            motor.stop_motor()
            motor.start_motor()
            time.sleep(0.1)            
            motor.send_position(min_limit, duration_ms=300)
            time.sleep(2)
            
            print("   Moving to maximum limit...")
            motor.send_position(max_limit, duration_ms=300)
            time.sleep(2)
            
            print("   Moving back to center...")
            center = (min_limit + max_limit) / 2
            motor.send_position(center, duration_ms=300)
            time.sleep(2)
            
            motor.stop_motor()
            print("\n✓ Limit finding completed successfully!")
            
    except GIM8115Error as e:
        print(f"\n✗ GIM8115 Error: {e}")
        print("   Make sure:")
        print("   - Motor is connected and powered")
        print("   - CAN bus is properly configured")
        print("   - Safety limit switches are connected and working")
        print("   - Motor can move freely in both directions")
    except KeyboardInterrupt:
        print("\n\n✗ Interrupted by user")
        print("   Motor should be stopped automatically")
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

