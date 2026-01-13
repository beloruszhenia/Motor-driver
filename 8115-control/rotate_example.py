"""
Motor Rotation Example

This script demonstrates how to rotate the motor using position and velocity control.

Usage:
    python3 rotate_example.py
"""

import time
import math
from gim8115_driver import (
    GIM8115Driver, 
    GIM8115Error
)


def main():
    """
    Example of rotating the motor using different methods
    """
    print("=" * 60)
    print("Motor Rotation Example")
    print("=" * 60)
    
    # Motor parameters (adjust based on your motor specifications)
    TORQUE_CONSTANT = 1  # N⋅m/A (example value, check motor datasheet)
    GEAR_RATIO = 36  # Example gear ratio
    
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
            time.sleep(0.3)  # Wait for motor to start
            print("   ✓ Motor started")
                        
            # Example 2: Continuous rotation using velocity control
            print("\n4. Example 2: Velocity Control (Continuous Rotation)")
            print("   Rotating left at 1 rad/s (~28.6 deg/s)...")
            motor.send_velocity(30, duration_ms=100)  # Rotate left for 3 seconds
            time.sleep(5)
            

            
            # Example 3: Slow rotation for limit finding
            print("\n5. Example 3: Slow Rotation (for limit finding)")
            print("   Rotating slowly left at 0.1 rad/s (~5.7 deg/s)...")
            motor.send_velocity(-0.1, duration_ms=50)  # Rotate left for 5 seconds
            time.sleep(5.5)
            
            print("   Rotating slowly right at 0.1 rad/s (~5.7 deg/s)...")
            motor.send_velocity(0.1, duration_ms=50)  # Rotate right for 5 seconds
            time.sleep(5.5)
            
           
            # Stop motor
            print("\n7. Stopping motor...")
            motor.stop_motor()
            print("   ✓ Motor stopped")
            
            print("\n✓ Rotation examples completed successfully!")
            
    except GIM8115Error as e:
        print(f"\n✗ GIM8115 Error: {e}")
        print("   Make sure:")
        print("   - Motor is connected and powered")
        print("   - CAN bus is properly configured")
        print("   - Motor can move freely")
    except KeyboardInterrupt:
        print("\n\n✗ Interrupted by user")
        print("   Motor should be stopped automatically")
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

