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
            
            # Verify limits from driver
            saved_min, saved_max = motor.get_position_limits()
            print(f"\n   Verified limits from driver:")
            print(f"   Min: {math.degrees(saved_min):.2f}° ({saved_min:.4f} rad)")
            print(f"   Max: {math.degrees(saved_max):.2f}° ({saved_max:.4f} rad)")
            
            # Check if limits match
            if abs(saved_min - min_limit) > 0.001 or abs(saved_max - max_limit) > 0.001:
                print(f"   ⚠️  Warning: Limits don't match! Using returned values.")
                print(f"      Returned: min={math.degrees(min_limit):.2f}°, max={math.degrees(max_limit):.2f}°")
                print(f"      Saved: min={math.degrees(saved_min):.2f}°, max={math.degrees(saved_max):.2f}°")
            else:
                print(f"   ✓ Limits match!")                                   
            
            
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

