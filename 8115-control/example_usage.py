"""
Example usage of GIM8115 Driver

This script demonstrates how to use the GIM8115Driver to control a motor.
# python3 example_usage.py
"""

import time
import math
from gim8115_driver import (
    GIM8115Driver, 
    GIM8115Error, 
    GIM8115ResultError,
    INDIID_MEC_ANGLE_SHAFT,
    INDIID_SPEED_SHAFT
)


def main():
    """Example motor control sequence"""
    
    # Motor parameters (adjust based on your motor specifications)
    TORQUE_CONSTANT = 1 # N⋅m/A (example value, check motor datasheet)
    GEAR_RATIO = 36  # Example gear ratio
    
    # Initialize driver
    # Using context manager for automatic connection/disconnection
    try:
        with GIM8115Driver(
            interface="can0",
            can_id=0x0A,  # Default CAN ID per .cursorrules
            bitrate=500000,  # 500 kbps per .cursorrules
            torque_constant=TORQUE_CONSTANT,
            gear_ratio=GEAR_RATIO
        ) as motor:

            # print("\n--- Set Zero Position ---")
            # print("Setting current position as zero...")
            # motor.set_zero_position()
            # time.sleep(0.1)
            
            print("Connected to GIM8115 motor")
            print("Current position: ", motor.get_current_position())
            print("Current speed: ", motor.retrieve_indicator(INDIID_SPEED_SHAFT))
            
            # Start safety listener to monitor CAN ID 0x005 for limit switches
            print("\nStarting safety listener (CAN ID 0x005)...")
            motor.start_safety_listener(auto_stop=True)  # Automatically stop motor on limit trigger
            
            # Start the motor
            print("\nStarting motor...")
            motor.start_motor()
            time.sleep(0.3)  # Wait for motor to start
            
            # # Example 1: Position control
            print("\n--- Position 60 degrees ---")
            target_angle = 2 * math.pi /360 * 60  
            duration = 100  # 100ms seconds
            print(f"Moving to {math.degrees(target_angle):.1f} degrees in {duration}ms")          
            motor.send_position(target_angle, duration)     
            time.sleep(2)
            

            print("\n--- Position -60 degrees ---")
            target_angle = 2 * math.pi /360 * -60   
            duration = 100  # 100ms seconds
            print(f"Moving to {math.degrees(target_angle):.1f} degrees in {duration}ms")            
            motor.send_position(target_angle, duration)     
            time.sleep(2)

            # # Example 1: Position control
            print("\n--- Position 120 degrees ---")
            target_angle = 2 * math.pi /360 * 120  
            duration = 100  # 100ms seconds
            print(f"Moving to {math.degrees(target_angle):.1f} degrees in {duration}ms")          
            motor.send_position(target_angle, duration)     
            time.sleep(2)

            print("\n--- Position 0 degrees ---")
            target_angle = 2 * math.pi /360 * 0   
            duration = 100  # 100ms seconds
            print(f"Moving to {math.degrees(target_angle):.1f} degrees in {duration}ms")            
            motor.send_position(target_angle, duration)     
            time.sleep(3)

            print("\n--- Position 0 degrees ---")
            target_angle = 2 * math.pi /360 * -2
            duration = 100  # 100ms seconds
            print(f"Moving to {math.degrees(target_angle):.1f} degrees in {duration}ms")            
            motor.send_position(target_angle, duration)              
            time.sleep(0.1)

            print("\n--- Position 0 degrees ---")
            target_angle = 2 * math.pi /360 * 2
            duration = 100  # 100ms seconds
            print(f"Moving to {math.degrees(target_angle):.1f} degrees in {duration}ms")            
            motor.send_position(target_angle, duration)              
            time.sleep(0.1)



            print("\n--- Position 0 degrees ---")
            target_angle = 2 * math.pi /360 * 0   
            duration = 100  # 100ms seconds
            print(f"Moving to {math.degrees(target_angle):.1f} degrees in {duration}ms")            
            motor.send_position(target_angle, duration)     
            time.sleep(3)

            # print("\n--- Position Control to Zero ---")
            # target_angle = 0.0  # 0.0 radians
            # duration = 1000  # 100ms seconds
            # print(f"Moving to {math.degrees(target_angle):.1f} degrees in {duration}ms")
            # motor.send_position(target_angle, duration)
            # time.sleep(1)

            # Wait for response
            # try:
            #     response = motor._receive_frame(timeout=2.5)
            #     if response:
            #         status = motor.parse_feedback(response)
            #         print(f"Position: {math.degrees(status.position_rad):.2f}°")
            #         print(f"Speed: {status.speed_rads:.2f} rad/s")
            #         print(f"Torque: {status.torque_nm:.3f} N⋅m")
            #         print(f"Temperature: {status.temperature}°C")
            #     else:
            #         print("No response received")
            # except GIM8115Error as e:
            #     print(f"Error: {e}")
            
            # time.sleep(2.5)  # Wait for movement to complete
            
            # Example 2: Velocity control
            # print("\n--- Velocity Control Example ---")
            # target_speed = 100  # 0.1 rad/s
            # print(f"Setting speed to {target_speed} rad/s")
            
            # motor.send_velocity(target_speed, duration_ms=0)  # 0 = immediate
            
            # time.sleep(1.0)

            # print("\n--- Position Control to Zero ---")
            # target_angle = 0.0  # 0.0 radians
            # duration = 100  # 100ms seconds
            # print(f"Moving to {math.degrees(target_angle):.1f} degrees in {duration}ms")
            # motor.send_position(target_angle, duration)
            # time.sleep(2)
            
            # Example 3: Torque control
            # print("\n--- Torque Control Example ---")
            # target_torque = 0.5  # 0.5 N⋅m
            # print(f"Setting torque to {target_torque} N⋅m")
            
            # motor.send_torque(target_torque, duration_ms=0)
            
            # time.sleep(2.0)
            
            # Example 4: Set zero position
            # print("\n--- Set Zero Position ---")
            # print("Setting current position as zero...")
            # motor.set_zero_position()
            # time.sleep(2)
            
            # Example 5: Continuous position control loop
            # print("\n--- Continuous Control Loop ---")
            # print("Moving motor in sine wave pattern...")
            
            # start_time = time.time()
            # duration_loop = 10.0  # Run for 10 seconds
            
            # while time.time() - start_time < duration_loop:
            #     # Generate sine wave position command
            #     t = time.time() - start_time
            #     angle = math.sin(2 * math.pi * 0.5 * t) * (math.pi / 4)  # ±45 degrees at 0.5 Hz
                
            #     motor.send_position(angle, duration_ms=100)  # 100ms duration
                
            #     # Try to receive feedback (non-blocking)
            #     try:
            #         response = motor._receive_frame(timeout=0.05)
            #         if response:
            #             status = motor.parse_feedback(response, check_result=False)
            #             print(f"t={t:.2f}s: pos={math.degrees(status.position_rad):.1f}°, "
            #                   f"speed={status.speed_rads:.2f} rad/s")
            #     except GIM8115Error:
            #         pass  # No response yet, continue
                
            #     time.sleep(0.1)  # 10 Hz control loop
            
            # Stop motor
            print("\nStopping motor...")
            motor.stop_motor()
            time.sleep(0.1)
            
            print("\nExample completed successfully!")
            
    except GIM8115Error as e:
        print(f"GIM8115 Error: {e}")
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Unexpected error: {e}")


def simple_position_example():
    """Simplest possible example"""
    motor = GIM8115Driver(interface="can0", can_id=0x0A)
    motor.connect()
    
    try:
        motor.start_motor()
        motor.send_position(angle_rad=math.pi / 2, duration_ms=1000)  # 90 degrees in 1 second
        time.sleep(1.5)
        motor.stop_motor()
    finally:
        motor.disconnect()


if __name__ == "__main__":
    main()

