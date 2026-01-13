"""
GIM8115 Motor Driver for SocketCAN (Linux)
Implements SteadyWin GIM Protocol Specification
"""

import struct
import time
import can
import json
import os
import threading
from dataclasses import dataclass
from typing import Optional, Callable
import math


# Command codes
CMD_START_MOTOR = 0x91
CMD_STOP_MOTOR = 0x92
CMD_TORQUE_CONTROL = 0x93
CMD_VELOCITY_CONTROL = 0x94
CMD_POSITION_CONTROL = 0x95
CMD_STOP_CONTROL = 0x97
CMD_MODIFY_CONFIGURATION = 0x83
CMD_REFRESH_CONFIGURATION = 0x82
CMD_RETRIEVE_INDICATOR = 0xB4

# Configuration IDs
CONFID_ZERO_POSITION = 0x14

# Indicator IDs
INDIID_MEC_ANGLE_SHAFT = 0x13  # Mechanical Angle of Output Shaft (RAD)
INDIID_SPEED_SHAFT = 0x14  # Speed of Output Shaft (RAD/s)

# Result codes
RES_SUCCESS = 0x00
RES_FAIL = 0x01
RES_FAIL_UNKNOWN_CMD = 0x02
RES_FAIL_UNKNOWN_ID = 0x03
RES_FAIL_RO_REG = 0x04
RES_FAIL_UNKNOWN_REG = 0x05
RES_FAIL_STR_FORMAT = 0x06
RES_FAIL_DATA_FORMAT = 0x07
RES_FAIL_WO_REG = 0x0B
RES_FAIL_NOT_CONNECTED = 0x80

# Result code descriptions
RESULT_CODE_NAMES = {
    RES_SUCCESS: "Success",
    RES_FAIL: "Fail",
    RES_FAIL_UNKNOWN_CMD: "Unknown Command",
    RES_FAIL_UNKNOWN_ID: "Unknown ID",
    RES_FAIL_RO_REG: "Read-Only Register",
    RES_FAIL_UNKNOWN_REG: "Unknown Register",
    RES_FAIL_STR_FORMAT: "String Format Error",
    RES_FAIL_DATA_FORMAT: "Data Format Error",
    RES_FAIL_WO_REG: "Write-Only Register",
    RES_FAIL_NOT_CONNECTED: "Not Connected",
}

# Safety CAN ID and status codes
CAN_ID_SAFETY = 0x005
SAFETY_DEVICE_1 = 0x01
SAFETY_STATUS_MIN_LIMIT = 0x10  # Border limit1 (hard stop)
SAFETY_STATUS_MAX_LIMIT = 0x20  # Border limit2 (hard stop)
SAFETY_STATUS_LIMIT1_FIND = 0x11  # Safety limit1 (approaching, used for limit finding)
SAFETY_STATUS_LIMIT2_FIND = 0x12  # Safety limit2 (approaching, used for limit finding)

# Safety limit shift: ±5 degrees added to found limits
SAFETY_LIMIT_SHIFT_DEG = 5.0
SAFETY_LIMIT_SHIFT_RAD = math.radians(SAFETY_LIMIT_SHIFT_DEG)


class GIM8115Error(Exception):
    """Base exception for GIM8115 driver errors"""
    pass


class GIM8115ResultError(GIM8115Error):
    """Exception raised when motor returns non-success result code"""
    def __init__(self, result_code: int, message: str = ""):
        self.result_code = result_code
        self.message = message or RESULT_CODE_NAMES.get(result_code, f"Unknown error code: 0x{result_code:02X}")
        super().__init__(f"Motor error: {self.message} (0x{result_code:02X})")


@dataclass
class MotorStatus:
    """Motor status feedback data"""
    __slots__ = ('command_echo', 'result_code', 'temperature', 'position_rad', 
                 'speed_rads', 'torque_nm', 'torque_raw')
    
    command_echo: int
    result_code: int
    temperature: int  # int8, in Celsius
    position_rad: float  # Position in radians (-12.5 to +12.5)
    speed_rads: float  # Speed in rad/s (-65 to +65)
    torque_nm: float  # Torque in N⋅m (requires KT and gear ratio)
    torque_raw: float  # Raw torque value in Amperes (-225 to +225)


class GIM8115Driver:
    """
    GIM8115 Motor Driver for SocketCAN
    
    Implements the SteadyWin GIM Protocol for controlling GIM8115 motors
    over Linux SocketCAN interface.
    """
    
    # Default CAN ID (Standard 11-bit)
    CAN_ID = 0x0A
    
    # Frame size is always 8 bytes
    FRAME_SIZE = 8
    
    def __init__(
        self,
        interface: str = "can0",
        can_id: int = 0x0A,
        bitrate: int = 500000,
        torque_constant: float = 1.0,
        gear_ratio: float = 1.0,
        config_file: str = "gim8115_config.json"
    ):
        """
        Initialize GIM8115 driver
        
        Args:
            interface: CAN interface name (e.g., "can0")
            can_id: CAN ID for the motor (default 0x0A / 10)
            bitrate: CAN bus bitrate in bits/s (default 500,000 bps / 500 kbps)
            torque_constant: Motor torque constant KT (N⋅m/A)
            gear_ratio: Gear ratio (output/input)
            config_file: Path to configuration file for storing offset (default: "gim8115_config.json")
        """
        self.interface = interface
        self.can_id = can_id
        self.bitrate = bitrate
        self.torque_constant = torque_constant
        self.gear_ratio = gear_ratio
        self.config_file = config_file
        
        # Pre-allocate buffers (no dynamic allocation in loops)
        self._tx_buffer = bytearray(self.FRAME_SIZE)
        self._rx_buffer = bytearray(self.FRAME_SIZE)
        
        # CAN bus interface
        self._bus: Optional[can.Bus] = None
        
        # Position offset (loaded from config file)
        self._position_offset: float = 0.0
        
        # Position limits (relative to offset/zero position)
        # Default: +/- 60 degrees from zero position
        DEFAULT_LIMIT_DEG = 60.0
        DEFAULT_LIMIT_RAD = math.radians(DEFAULT_LIMIT_DEG)
        self._position_min_limit: float = -DEFAULT_LIMIT_RAD
        self._position_max_limit: float = DEFAULT_LIMIT_RAD
        self._position_limits_enabled: bool = True
        
        # Position border limits (relative to offset/zero position)
        # These are the physical border limits detected by 0x11/0x12 (before safety shift)
        self._position_border_min_limit: Optional[float] = None
        self._position_border_max_limit: Optional[float] = None
        
        # Limit finding speed (rad/s)
        self._limit_find_speed_rads: float = 0.5  # Default: 0.5 rad/s = ~28.6 deg/s
        
        self.load_config()
        
        # Safety listener state
        self._safety_listener_thread: Optional[threading.Thread] = None
        self._safety_listener_running: bool = False
        self._safety_callback: Optional[Callable[[int, int], None]] = None
        self._auto_stop_on_limit: bool = True  # Automatically stop motor on limit trigger
        
        # Status monitoring state
        self._status_monitor_thread: Optional[threading.Thread] = None
        self._status_monitor_running: bool = False
        self._status_callback: Optional[Callable[[MotorStatus], None]] = None
        self._status_monitor_rate: float = 10.0  # Hz (10 times per second)
        
    def load_config(self) -> None:
        """
        Load configuration from file (position offset and limits)
        """
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    config = json.load(f)
                    self._position_offset = config.get('position_offset', 0.0)
                    
                    # Load position limits (in radians)
                    if 'position_min_limit' in config:
                        self._position_min_limit = config['position_min_limit']
                    if 'position_max_limit' in config:
                        self._position_max_limit = config['position_max_limit']
                    if 'position_limits_enabled' in config:
                        self._position_limits_enabled = config['position_limits_enabled']
                    
                    # Load position border limits (in radians)
                    if 'position_border_min_limit' in config:
                        self._position_border_min_limit = config['position_border_min_limit']
                    if 'position_border_max_limit' in config:
                        self._position_border_max_limit = config['position_border_max_limit']
                    
                    # Load limit finding speed
                    if 'limit_find_speed_rads' in config:
                        self._limit_find_speed_rads = config['limit_find_speed_rads']
            except (json.JSONDecodeError, IOError) as e:
                # If config file is corrupted, use defaults
                self._position_offset = 0.0
        else:
            # Config file doesn't exist, use defaults
            self._position_offset = 0.0
            
    def save_config(self) -> None:
        """
        Save configuration to file (position offset and limits)
        """
        config = {
            'position_offset': self._position_offset,
            'position_min_limit': self._position_min_limit,
            'position_max_limit': self._position_max_limit,
            'position_limits_enabled': self._position_limits_enabled,
            'limit_find_speed_rads': self._limit_find_speed_rads,
            'can_id': self.can_id,
            'interface': self.interface
        }
        # Add border limits if they are set
        if self._position_border_min_limit is not None:
            config['position_border_min_limit'] = self._position_border_min_limit
        if self._position_border_max_limit is not None:
            config['position_border_max_limit'] = self._position_border_max_limit
        try:
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=2)
        except IOError as e:
            raise GIM8115Error(f"Failed to save config file: {e}") from e
            
    def get_position_offset(self) -> float:
        """
        Get the current position offset
        
        Returns:
            Position offset in radians
        """
        return self._position_offset
        
    def set_position_offset(self, offset_rad: float) -> None:
        """
        Set position offset manually
        
        Args:
            offset_rad: Position offset in radians
        """
        self._position_offset = offset_rad
        self.save_config()
    
    def get_position_limits(self) -> tuple[float, float]:
        """
        Get current position limits (left/right) - safety limits after shift
        
        Returns:
            Tuple of (min_limit, max_limit) in radians relative to zero position
        """
        return (self._position_min_limit, self._position_max_limit)
    
    def get_position_border_limits(self) -> Optional[tuple[float, float]]:
        """
        Get position border limits (left/right) - physical border limits before shift
        
        These are the positions where 0x11 and 0x12 were detected during limit finding.
        
        Returns:
            Tuple of (border_min_limit, border_max_limit) in radians relative to zero position,
            or None if border limits have not been set yet
        """
        if self._position_border_min_limit is None or self._position_border_max_limit is None:
            return None
        return (self._position_border_min_limit, self._position_border_max_limit)
    
    def set_position_limits(self, min_limit_rad: float, max_limit_rad: float) -> None:
        """
        Set position limits (left/right) relative to zero position
        
        Args:
            min_limit_rad: Minimum position (left limit) in radians (negative value)
            max_limit_rad: Maximum position (right limit) in radians (positive value)
        """
        if min_limit_rad >= max_limit_rad:
            raise GIM8115Error("Minimum limit must be less than maximum limit")
        self._position_min_limit = min_limit_rad
        self._position_max_limit = max_limit_rad
        self.save_config()
    
    def set_position_limits_degrees(self, min_limit_deg: float, max_limit_deg: float) -> None:
        """
        Set position limits in degrees (convenience method)
        
        Args:
            min_limit_deg: Minimum position (left limit) in degrees (negative value)
            max_limit_deg: Maximum position (right limit) in degrees (positive value)
        """
        self.set_position_limits(math.radians(min_limit_deg), math.radians(max_limit_deg))
    
    def enable_position_limits(self, enabled: bool = True) -> None:
        """
        Enable or disable position limit checking
        
        Args:
            enabled: True to enable limits, False to disable
        """
        self._position_limits_enabled = enabled
        self.save_config()
    
    def is_position_limits_enabled(self) -> bool:
        """
        Check if position limits are enabled
        
        Returns:
            True if limits are enabled, False otherwise
        """
        return self._position_limits_enabled
    
    def enforce_position_limits(self, duration_ms: int = 100) -> bool:
        """
        Check current motor position and move to nearest limit if beyond limits
        
        Args:
            duration_ms: Total time from start to stop in milliseconds (acceleration + deceleration).
                        Example: 100ms = 50ms acceleration + 50ms deceleration.
            
        Returns:
            True if motor was moved to limit, False if already within limits or limits disabled
        """
        if not self._position_limits_enabled:
            return False
        
        # Get current position (relative to zero)
        current_abs = self.get_current_position(timeout=0.5)
        if current_abs is None:
            return False
        
        # Convert absolute position to relative position (subtract offset)
        current_relative = current_abs - self._position_offset
        
        # Check if beyond limits
        if current_relative < self._position_min_limit:
            # Motor is beyond left limit, move to left limit
            self.send_position(self._position_min_limit, duration_ms)
            return True
        elif current_relative > self._position_max_limit:
            # Motor is beyond right limit, move to right limit
            self.send_position(self._position_max_limit, duration_ms)
            return True
        
        return False  # Already within limits
    
    def get_limit_find_speed(self) -> float:
        """
        Get the configured rotation speed for limit finding
        
        Returns:
            Rotation speed in rad/s
        """
        return self._limit_find_speed_rads
    
    def set_limit_find_speed(self, speed_rads: float) -> None:
        """
        Set the rotation speed for limit finding
        
        Args:
            speed_rads: Rotation speed in rad/s
        """
        if speed_rads <= 0:
            raise GIM8115Error("Limit find speed must be positive")
        self._limit_find_speed_rads = speed_rads
        self.save_config()
    
    def find_position_limits(
        self,
        speed_rads: Optional[float] = None,
        timeout_seconds: float = 60.0,
        check_interval: float = 0.05
    ) -> tuple[float, float]:
        """
        Automatically find position limits by rotating until limit events are detected
        
        Process:
        1. Rotate in negative direction until min limit safety event
        2. Record position as min limit
        3. Reverse direction and rotate until max limit safety event
        4. Record position as max limit
        5. Set limits based on found positions (relative to current zero)
        
        Args:
            speed_rads: Rotation speed in rad/s (default: uses configured limit_find_speed_rads from config)
            timeout_seconds: Maximum time to wait for each limit (default: 60 seconds)
            check_interval: Interval to check for safety messages (default: 0.05 seconds = 20 Hz)
            
        Returns:
            Tuple of (min_limit, max_limit) in radians relative to zero position
            
        Raises:
            GIM8115Error: If motor is not started, connection issues, or timeout
        """
        # Use configured speed if not specified
        if speed_rads is None:
            speed_rads = self._limit_find_speed_rads
        if self._bus is None:
            raise GIM8115Error("Not connected to CAN bus. Call connect() first.")
        
        # Temporarily disable position limits to allow free movement
        limits_were_enabled = self._position_limits_enabled
        self._position_limits_enabled = False
        
        # Temporarily stop safety listener to avoid race condition with CAN bus reading
        # Both the listener thread and limit finding code would try to read from CAN bus
        safety_listener_was_running = self._safety_listener_running
        if safety_listener_was_running:
            print("Temporarily stopping safety listener during limit finding...")
            self.stop_safety_listener()
            time.sleep(0.1)  # Give thread time to stop
        
        try:
            # Ensure motor is started
            self.start_motor()            
            
            min_limit_abs: Optional[float] = None
            max_limit_abs: Optional[float] = None
            
            # Step 1: Find safety limit1 (rotate negative/left)
            # Look for STATUS_LIMIT1_FIND (0x11), ignore border limits (0x10)
            print("Finding safety limit1 (rotating left, looking for 0x11)...")
            print(f"  Motor should be rotating left at {math.degrees(speed_rads):.1f}°/s")
            print(f"  Waiting for CAN message 0x005 [0x01, 0x11]...")
            self.send_velocity(-abs(speed_rads), duration_ms=0)  # Negative speed = left
            time.sleep(0.1)  # Give motor time to start
            
            start_time = time.time()
            last_status_time = start_time
            while (time.time() - start_time) < timeout_seconds:
                # Check for safety message
                result = self.check_safety_message(timeout=check_interval)
                if result is not None:
                    device_id, status = result
                    elapsed = time.time() - start_time
                    print(f"  Received CAN message: Device {device_id:02X}, Status {status:02X} (after {elapsed:.1f}s)")
                    
                    # Only use safety limit1 (0x11), ignore border limit (0x10)
                    if status == SAFETY_STATUS_LIMIT1_FIND:
                        # Safety limit1 detected, stop and record position
                        print(f"  Safety limit1 (0x11) detected! Stopping motor and recording position...")
                        self.stop_motor()
                        time.sleep(0.2)
                        
                        current_abs = self.get_current_position(timeout=1.0)
                        if current_abs is not None:
                            min_limit_abs = current_abs  # Store absolute position
                            min_limit_rel = current_abs - self._position_offset
                            print(f"✓ Safety limit1 found at {math.degrees(min_limit_rel):.2f}° (abs: {min_limit_abs:.4f} rad)")
                            break
                        else:
                            print(f"⚠️  Warning: Could not get current position after detecting limit1")
                            # Try again with longer timeout
                            time.sleep(0.5)
                            current_abs = self.get_current_position(timeout=2.0)
                            if current_abs is not None:
                                min_limit_abs = current_abs
                                min_limit_rel = current_abs - self._position_offset
                                print(f"✓ Safety limit1 found at {math.degrees(min_limit_rel):.2f}° (abs: {min_limit_abs:.4f} rad) - retry successful")
                                break
                            else:
                                raise GIM8115Error("Failed to get current position after detecting safety limit1 (0x11)")
                    elif status == SAFETY_STATUS_MIN_LIMIT:
                        # Border limit reached - continue but don't use for limit finding
                        print(f"⚠️  Border limit1 reached, continuing to find safety limit1...")
                
                # Print progress every 5 seconds
                elapsed = time.time() - start_time
                if elapsed - (last_status_time - start_time) >= 5.0:
                    print(f"  Still searching... ({elapsed:.1f}s elapsed, waiting for 0x11)")
                    last_status_time = time.time()
                
                # Check timeout
                if (time.time() - start_time) >= timeout_seconds:
                    raise GIM8115Error("Timeout waiting for safety limit1 event (0x11)")
            
            if min_limit_abs is None:
                raise GIM8115Error("Failed to find safety limit1 (0x11)")
            
            # Small delay before reversing
            time.sleep(0.5)
            
            # Step 2: Find safety limit2 (rotate positive/right)
            # Look for STATUS_LIMIT2_FIND (0x12), ignore border limits (0x20)
            print("Finding safety limit2 (rotating right, looking for 0x12)...")
            print(f"  Motor should be rotating right at {math.degrees(speed_rads):.1f}°/s")
            print(f"  Waiting for CAN message 0x005 [0x01, 0x12]...")
            self.start_motor()            
            time.sleep(0.1)  # Give motor time to start
            self.send_velocity(abs(speed_rads), duration_ms=0)  # Positive speed = right
            
            start_time = time.time()
            last_status_time = start_time
            while (time.time() - start_time) < timeout_seconds:
                # Check for safety message
                result = self.check_safety_message(timeout=check_interval)
                if result is not None:
                    device_id, status = result
                    elapsed = time.time() - start_time
                    print(f"  Received CAN message: Device {device_id:02X}, Status {status:02X} (after {elapsed:.1f}s)")
                    
                    # Only use safety limit2 (0x12), ignore border limit (0x20)
                    if status == SAFETY_STATUS_LIMIT2_FIND:
                        # Safety limit2 detected, stop and record position
                        print(f"  Safety limit2 (0x12) detected! Stopping motor and recording position...")
                        self.stop_motor()
                        time.sleep(0.2)
                        
                        current_abs = self.get_current_position(timeout=1.0)
                        if current_abs is not None:
                            max_limit_abs = current_abs  # Store absolute position
                            max_limit_rel = current_abs - self._position_offset
                            print(f"✓ Safety limit2 found at {math.degrees(max_limit_rel):.2f}° (abs: {max_limit_abs:.4f} rad)")
                            break
                        else:
                            print(f"⚠️  Warning: Could not get current position after detecting limit2")
                            # Try again with longer timeout
                            time.sleep(0.5)
                            current_abs = self.get_current_position(timeout=2.0)
                            if current_abs is not None:
                                max_limit_abs = current_abs
                                max_limit_rel = current_abs - self._position_offset
                                print(f"✓ Safety limit2 found at {math.degrees(max_limit_rel):.2f}° (abs: {max_limit_abs:.4f} rad) - retry successful")
                                break
                            else:
                                raise GIM8115Error("Failed to get current position after detecting safety limit2 (0x12)")
                    elif status == SAFETY_STATUS_MAX_LIMIT:
                        # Border limit reached - continue but don't use for limit finding
                        print(f"⚠️  Border limit2 reached, continuing to find safety limit2...")
                
                # Print progress every 5 seconds
                elapsed = time.time() - start_time
                if elapsed - (last_status_time - start_time) >= 5.0:
                    print(f"  Still searching... ({elapsed:.1f}s elapsed, waiting for 0x12)")
                    last_status_time = time.time()
                
                # Check timeout
                if (time.time() - start_time) >= timeout_seconds:
                    raise GIM8115Error("Timeout waiting for safety limit2 event (0x12)")
            
            if max_limit_abs is None:
                raise GIM8115Error("Failed to find safety limit2 (0x12)")
            
            # Validate limits
            if min_limit_abs >= max_limit_abs:
                raise GIM8115Error(f"Invalid limits: min ({min_limit_abs:.3f}) >= max ({max_limit_abs:.3f})")
                                    
            # Calculate relative positions before shift (for display) - using current offset
            min_limit_rel_before = min_limit_abs - self._position_offset
            max_limit_rel_before = max_limit_abs - self._position_offset
            
            # Apply safety limit shift: ±5 degrees to found border limits
            # Shift inward to create safety margin away from physical borders
            min_limit_abs_shifted = min_limit_abs + SAFETY_LIMIT_SHIFT_RAD  # Move inward (more positive)
            max_limit_abs_shifted = max_limit_abs - SAFETY_LIMIT_SHIFT_RAD  # Move inward (more negative)
            
            # Calculate center position in absolute coordinates (using shifted limits)
            center_abs = (min_limit_abs_shifted + max_limit_abs_shifted) / 2.0
            
            # Set new offset so that center becomes zero (relative position)
            # When user sends 0.0, we want: 0.0 + new_offset = center_abs
            # So: new_offset = center_abs
            old_offset = self._position_offset
            self._position_offset = center_abs
            
            # Calculate limits relative to new center (symmetric around zero)
            # Half range: (max_limit_abs_shifted - min_limit_abs_shifted) / 2
            half_range = abs(max_limit_abs_shifted - min_limit_abs_shifted) / 2.0
            self._position_min_limit = -half_range
            self._position_max_limit = half_range
            
            # Store border limits relative to new center (before shift)
            # Border limits are the positions where 0x11 and 0x12 were detected
            self._position_border_min_limit = min_limit_abs - self._position_offset
            self._position_border_max_limit = max_limit_abs - self._position_offset
            
            # Display results using new offset for relative positions
            print(f"  Applying ±{SAFETY_LIMIT_SHIFT_DEG}° safety shift to border limits:")
            print(f"    Border limit1 (0x11): {math.degrees(self._position_border_min_limit):.2f}° -> Safety limit: {math.degrees(self._position_min_limit):.2f}°")
            print(f"    Border limit2 (0x12): {math.degrees(self._position_border_max_limit):.2f}° -> Safety limit: {math.degrees(self._position_max_limit):.2f}°")
            
            # Restore position limits state
            self._position_limits_enabled = limits_were_enabled
            
            # Move motor to center position (safe position within new limits)
            # This ensures the motor is at a valid position after limit finding
            print("Moving motor to center position (safe position within new limits)...")
            center_position = 0.0  # Center is now at zero (relative to new offset)
            try:
                self.start_motor()
                time.sleep(0.05)
                self.send_position(center_position, duration_ms=100)  # Move to center over 2 seconds
                time.sleep(2.5)  # Wait for movement to complete
                self.stop_motor()
                print("✓ Motor moved to center position")
            except Exception as e:
                print(f"⚠️  Warning: Could not move motor to center: {e}")
                # Continue anyway - motor is already stopped
            
            self.save_config()
            
            print(f"✓ Limits set: Min={math.degrees(self._position_min_limit):.2f}°, Max={math.degrees(self._position_max_limit):.2f}°")
            print(f"✓ Zero position set to center: offset={self._position_offset:.4f} rad")
            
            # Restore safety listener after successful completion
            if safety_listener_was_running:
                print("Restarting safety listener...")
                self.start_safety_listener(auto_stop=True)
                time.sleep(0.1)  # Give thread time to start
            
            return (self._position_min_limit, self._position_max_limit)
            
        except Exception as e:
            # Ensure motor is stopped on error
            try:
                self.stop_motor()
            except:
                pass
            # Restore limits state
            self._position_limits_enabled = limits_were_enabled
            # Restore safety listener on error too
            if safety_listener_was_running:
                print("Restarting safety listener after error...")
                try:
                    self.start_safety_listener(auto_stop=True)
                    time.sleep(0.1)  # Give thread time to start
                except Exception as listener_error:
                    print(f"⚠️  Warning: Could not restart safety listener: {listener_error}")
            raise GIM8115Error(f"Error finding limits: {e}") from e
        
    def __enter__(self):
        """Context manager entry"""
        self.connect()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.stop_safety_listener()
        self.stop_status_monitor()
        self.disconnect()
        
    def connect(self) -> None:
        """Connect to CAN bus"""
        if self._bus is not None:
            return
            
        try:
            self._bus = can.Bus(
                interface="socketcan",
                channel=self.interface,
                bitrate=self.bitrate
            )
        except Exception as e:
            raise GIM8115Error(f"Failed to connect to CAN bus {self.interface}: {e}") from e
            
    def disconnect(self) -> None:
        """Disconnect from CAN bus"""
        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None
            
    def _send_frame(self, payload: bytes) -> None:
        """
        Send CAN frame
        
        Args:
            payload: 8-byte payload
        """
        if self._bus is None:
            raise GIM8115Error("Not connected to CAN bus. Call connect() first.")
            
        if len(payload) != self.FRAME_SIZE:
            raise GIM8115Error(f"Payload must be exactly {self.FRAME_SIZE} bytes")
            
        msg = can.Message(
            arbitration_id=self.can_id,
            data=payload,
            is_extended_id=False
        )
        
        try:
            self._bus.send(msg)
            # Debug: print sent frame (can be removed in production)
            # print(f"Sent CAN ID 0x{self.can_id:02X}: {payload.hex()}")
        except Exception as e:
            raise GIM8115Error(f"Failed to send CAN frame: {e}") from e
            
    def _receive_frame(self, timeout: float = 1.0, filter_can_id: Optional[int] = None) -> Optional[bytes]:
        """
        Receive CAN frame from motor
        
        Args:
            timeout: Timeout in seconds
            filter_can_id: CAN ID to filter by. If None, uses self.can_id. If -1, accepts any ID.
            
        Returns:
            8-byte payload or None if timeout
        """
        if self._bus is None:
            raise GIM8115Error("Not connected to CAN bus. Call connect() first.")
            
        # Determine which CAN ID to filter by
        if filter_can_id == -1:
            # Accept any CAN ID
            target_can_id = None
        elif filter_can_id is None:
            # Default: filter by configured CAN ID
            target_can_id = self.can_id
        else:
            # Filter by specified CAN ID
            target_can_id = filter_can_id
            
        try:
            start_time = time.time()
            while (time.time() - start_time) < timeout:
                msg = self._bus.recv(timeout=min(0.1, timeout - (time.time() - start_time)))
                if msg is None:
                    continue
                # Filter by CAN ID if specified (some motors respond on different ID)
                if target_can_id is not None and msg.arbitration_id != target_can_id:
                    continue
                if len(msg.data) != self.FRAME_SIZE:
                    continue
                return bytes(msg.data)
            return None
        except Exception as e:
            raise GIM8115Error(f"Failed to receive CAN frame: {e}") from e
            
    def _clear_tx_buffer(self) -> None:
        """Clear transmit buffer (set all bytes to 0x00)"""
        for i in range(self.FRAME_SIZE):
            self._tx_buffer[i] = 0x00
    
    def start_motor(self) -> None:
        """Start the motor"""
        self._clear_tx_buffer()
        self._tx_buffer[0] = CMD_START_MOTOR
        self._send_frame(bytes(self._tx_buffer))
        
    def stop_motor(self) -> None:
        """Stop the motor"""
        self._clear_tx_buffer()
        self._tx_buffer[0] = CMD_STOP_MOTOR
        self._send_frame(bytes(self._tx_buffer))
        
    def _clamp_position(self, angle_rad: float) -> float:
        """
        Clamp position to limits if enabled
        
        Args:
            angle_rad: Target position in radians
            
        Returns:
            Clamped position in radians
        """
        if not self._position_limits_enabled:
            return angle_rad
        
        if angle_rad < self._position_min_limit:
            return self._position_min_limit
        elif angle_rad > self._position_max_limit:
            return self._position_max_limit
        
        return angle_rad
    
    def _pack_duration(self, duration_ms: int) -> None:
        """
        Pack duration (24-bit unsigned int) into bytes 5-7 of tx_buffer
        
        Args:
            duration_ms: Total time from start to stop in milliseconds (acceleration + deceleration).
                        Example: 100ms = 50ms acceleration + 50ms deceleration.
        """
        duration_bytes = struct.pack('<I', duration_ms & 0xFFFFFF)
        self._tx_buffer[5] = duration_bytes[0]
        self._tx_buffer[6] = duration_bytes[1]
        self._tx_buffer[7] = duration_bytes[2]
    
    def send_position(self, angle_rad: float, duration_ms: int = 0) -> None:
        """
        Send position control command (automatically applies position offset and clamps to limits)
        
        Args:
            angle_rad: Target position in radians (relative to calibrated zero)
            duration_ms: Total time from start to stop in milliseconds (acceleration + deceleration).
                        Example: 100ms = 50ms acceleration + 50ms deceleration.
                        Use 0 for immediate execution (max acceleration, no deceleration planning).
            
        Note:
            If position limits are enabled, the position will be clamped to the nearest limit.
            If command exceeds limits, motor will move to the limit position (cannot exceed min or max limits).
        """
        # Clamp position to limits if enabled
        angle_rad = self._clamp_position(angle_rad)
        
        # Apply position offset
        actual_position = angle_rad + self._position_offset
        
        # Build command frame
        self._tx_buffer[0] = CMD_POSITION_CONTROL
        struct.pack_into('<f', self._tx_buffer, 1, actual_position)
        self._pack_duration(duration_ms)
        
        self._send_frame(bytes(self._tx_buffer))
        
    def send_velocity(self, speed_rads: float, duration_ms: int) -> None:
        """
        Send velocity control command
        
        Args:
            speed_rads: Target speed in rad/s
            duration_ms: Total time from start to stop in milliseconds (acceleration + deceleration).
                        Example: 100ms = 50ms acceleration + 50ms deceleration.
                        Use 0 for immediate execution (max acceleration, no deceleration planning).
        """
        self._tx_buffer[0] = CMD_VELOCITY_CONTROL
        struct.pack_into('<f', self._tx_buffer, 1, speed_rads)
        self._pack_duration(duration_ms)
        
        self._send_frame(bytes(self._tx_buffer))
        
    def send_torque(self, torque_nm: float, duration_ms: int) -> None:
        """
        Send torque control command
        
        Args:
            torque_nm: Target torque in N⋅m
            duration_ms: Total time from start to stop in milliseconds (acceleration + deceleration).
                        Example: 100ms = 50ms acceleration + 50ms deceleration.
                        Use 0 for immediate execution (max acceleration, no deceleration planning).
        """
        self._tx_buffer[0] = CMD_TORQUE_CONTROL
        struct.pack_into('<f', self._tx_buffer, 1, torque_nm)
        self._pack_duration(duration_ms)
        
        self._send_frame(bytes(self._tx_buffer))
        
    def refresh_configuration(self) -> None:
        """
        Refresh configuration - applies all previously modified configurations
        Command 0x82: Byte 0 = 0x82, Bytes 1-7 = NULL (0x00)
        """
        self._clear_tx_buffer()
        self._tx_buffer[0] = CMD_REFRESH_CONFIGURATION
        self._send_frame(bytes(self._tx_buffer))
        
    def retrieve_indicator(self, indi_id: int, timeout: float = 1.0) -> Optional[float]:
        """
        Retrieve indicator value from motor
        
        Args:
            indi_id: Indicator ID (e.g., INDIID_MEC_ANGLE_SHAFT = 0x13)
            timeout: Timeout in seconds
            
        Returns:
            Indicator value (float), or None if no response or error
        """
        self._clear_tx_buffer()
        self._tx_buffer[0] = CMD_RETRIEVE_INDICATOR
        self._tx_buffer[1] = indi_id
        self._send_frame(bytes(self._tx_buffer))
        
        # Wait for response (accept any CAN ID - motor may respond on different ID)
        response = self._receive_frame(timeout=timeout, filter_can_id=-1)
        if response is None:
            return None
        
        # Validate response format
        if (len(response) != self.FRAME_SIZE or
            response[0] != CMD_RETRIEVE_INDICATOR or
            response[1] != indi_id or
            response[2] != RES_SUCCESS):
            return None
        
        # Extract float from bytes 4-7 (little-endian)
        try:
            return struct.unpack('<f', response[4:8])[0]
        except struct.error:
            return None
            
    def get_current_position(self, timeout: float = 1.0) -> Optional[float]:
        """
        Get current position from motor using Retrieve Indicator command
        Uses command 0xB4 with IndID 0x13 (Mechanical Angle of Output Shaft)
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            Current absolute position in radians, or None if no response
        """
        return self.retrieve_indicator(INDIID_MEC_ANGLE_SHAFT, timeout=timeout)
    
    def get_current_speed(self, timeout: float = 1.0) -> Optional[float]:
        """
        Get current speed from motor using Retrieve Indicator command
        Uses command 0xB4 with IndID 0x14 (Speed of Output Shaft)
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            Current speed in rad/s, or None if no response
        """
        return self.retrieve_indicator(INDIID_SPEED_SHAFT, timeout=timeout)
    
    def get_motor_status(self, timeout: float = 0.1) -> Optional[MotorStatus]:
        """
        Get full motor status (position, speed, torque, error) efficiently
        
        Uses retrieve_indicator for position and speed, which doesn't affect motor state.
        This is safe for continuous 10 Hz monitoring.
        
        Args:
            timeout: Timeout in seconds (should be short for 10 Hz polling, default: 0.1s)
            
        Returns:
            MotorStatus object with position and speed data, or None if no response or error
            Note: Temperature and torque are set to 0 (not available via indicators)
            Error status (result_code) is RES_SUCCESS if data is available
        """
        try:
            # Get position and speed using retrieve_indicator (non-intrusive, doesn't affect motor)
            # This is the safest method for continuous monitoring
            # Use shorter timeout per call to ensure we don't block too long
            position = self.retrieve_indicator(INDIID_MEC_ANGLE_SHAFT, timeout=timeout/2)
            # Small delay to avoid race condition with CAN bus responses
            time.sleep(0.01)  # 10ms delay
            speed = self.retrieve_indicator(INDIID_SPEED_SHAFT, timeout=timeout/2)
            
            if position is None:
                return None  # Position is required
            
            # Speed might be None if motor is not responding, use 0.0 as default
            if speed is None:
                speed = 0.0
            
            # Construct MotorStatus object
            return MotorStatus(
                command_echo=CMD_RETRIEVE_INDICATOR,
                result_code=RES_SUCCESS,  # Indicators return success if data is available
                temperature=0,  # Not available via indicators
                position_rad=position,
                speed_rads=speed,
                torque_nm=0.0,  # Not available via indicators
                torque_raw=0.0  # Not available via indicators
            )
        except Exception as e:
            # Don't print here - let the monitor loop handle logging
            return None
        
    def set_zero_position(self) -> None:
        """
        Set current position as zero by saving it as offset
        Reads current position from motor feedback and saves it as offset
        """
        # Get current position from motor
        current_pos = self.get_current_position(timeout=1.0)
        
        if current_pos is None:
            raise GIM8115Error("Failed to get current position from motor. Cannot set zero.")
        
        # Save current position as offset (negative because we add it later)
        # When user sends 0.0, we want: 0.0 + offset = current_pos
        # So: offset = current_pos - 0.0 = current_pos
        self._position_offset = current_pos
        self.save_config()
        
    def parse_feedback(self, data: bytes, check_result: bool = True) -> MotorStatus:
        """
        Parse feedback frame from motor
        
        Args:
            data: 8-byte response frame
            check_result: If True, raise exception on non-success result code
            
        Returns:
            MotorStatus object with parsed data
            
        Raises:
            GIM8115ResultError: If check_result is True and result code != 0x00
        """
        if len(data) != self.FRAME_SIZE:
            raise GIM8115Error(f"Feedback data must be exactly {self.FRAME_SIZE} bytes")
            
        # Byte 0: Command echo
        command_echo = data[0]
        
        # Byte 1: Result code
        result_code = data[1]
        
        if check_result and result_code != RES_SUCCESS:
            raise GIM8115ResultError(result_code)
            
        # Byte 2: Temperature (int8)
        temperature = struct.unpack('<b', data[2:3])[0]
        
        # Bytes 3-4: Position (uint16, little-endian)
        pos_uint16 = struct.unpack('<H', data[3:5])[0]
        # Decode position: pos_rad = (uint16_pos * 25.0 / 65535.0) - 12.5
        position_rad = (pos_uint16 * 25.0 / 65535.0) - 12.5
        
        # Bytes 5-7: Speed & Torque (compressed 12-bit values)
        # Speed: ST0 (byte 5) is High 8 bits. ST1 bits [7:4] (byte 6, upper 4 bits) are Low 4 bits.
        speed_int = (data[5] << 4) | ((data[6] & 0xF0) >> 4)
        # Decode speed: speed_rads = (speed_int * 130.0 / 4095.0) - 65.0
        speed_rads = (speed_int * 130.0 / 4095.0) - 65.0
        
        # Torque: ST1[3:0] (byte 6, lower 4 bits) is high 4 bits, ST2 (byte 7) is low 8 bits
        torque_int = ((data[6] & 0x0F) << 8) | data[7]
        # Decode torque: torque_raw = (torque_int * 450.0 / 4095.0) - 225.0 (in Amperes)
        torque_raw = (torque_int * 450.0 / 4095.0) - 225.0
        # Convert to N⋅m: torque_nm = (torque_int * (450 * KT * GEAR) / 4095.0) - (225 * KT * GEAR)
        torque_nm = (torque_int * (450.0 * self.torque_constant * self.gear_ratio) / 4095.0) - \
                    (225.0 * self.torque_constant * self.gear_ratio)
        
        return MotorStatus(
            command_echo=command_echo,
            result_code=result_code,
            temperature=temperature,
            position_rad=position_rad,
            speed_rads=speed_rads,
            torque_nm=torque_nm,
            torque_raw=torque_raw
        )
        
    def send_and_receive(
        self,
        payload: bytes,
        timeout: float = 1.0,
        check_result: bool = True
    ) -> MotorStatus:
        """
        Send command and wait for response
        
        Args:
            payload: 8-byte command payload
            timeout: Receive timeout in seconds
            check_result: If True, raise exception on non-success result code
            
        Returns:
            MotorStatus object with feedback data
        """
        self._send_frame(payload)
        response = self._receive_frame(timeout=timeout)
        if response is None:
            raise GIM8115Error("Timeout waiting for motor response")
        return self.parse_feedback(response, check_result=check_result)
    
    def check_safety_message(self, timeout: float = 0.002) -> Optional[tuple[int, int]]:
        """
        Check for safety CAN messages (ID 0x005) - non-blocking polling method
        
        Safety message format (CAN ID 0x005):
        - Device 1 Border Limit1: [0x01, 0x10] - hard stop
        - Device 1 Border Limit2: [0x01, 0x20] - hard stop
        - Device 1 Safety Limit1: [0x01, 0x11] - approaching limit1 (used for limit finding)
        - Device 1 Safety Limit2: [0x01, 0x12] - approaching limit2 (used for limit finding)
        
        Args:
            timeout: Timeout in seconds (should be short for non-blocking)
            
        Returns:
            Tuple of (device_id, status) if limit triggered, None otherwise
            - device_id: 0x01 (Device 1)
            - status: 0x10 (Border Limit1), 0x20 (Border Limit2), 0x11 (Safety Limit1), or 0x12 (Safety Limit2)
        """
        if self._bus is None:
            return None
        
        try:
            msg = self._bus.recv(timeout=timeout)
            if msg is None:
                return None
            
            # Check if it's a safety message (CAN ID 0x005)
            if msg.arbitration_id != CAN_ID_SAFETY:
                return None
            
            # Parse safety message: [device_id, status]
            if len(msg.data) < 2:
                return None
            
            device_id = msg.data[0]
            status = msg.data[1]
            
            # Only process Device 1 (0x01) limit triggers
            # Accept all status codes: border limits (0x10, 0x20) and safety limits (0x11, 0x12)
            if device_id == SAFETY_DEVICE_1 and status in (
                SAFETY_STATUS_MIN_LIMIT, SAFETY_STATUS_MAX_LIMIT,
                SAFETY_STATUS_LIMIT1_FIND, SAFETY_STATUS_LIMIT2_FIND
            ):
                return (device_id, status)
            
            return None
        except Exception:
            # Ignore errors in non-blocking check
            return None
    
    def _set_thread_priority(self) -> None:
        """Set thread to highest priority (requires appropriate permissions)"""
        try:
            # Try real-time scheduling (requires root or CAP_SYS_NICE)
            try:
                max_priority = os.sched_get_priority_max(os.SCHED_FIFO)
                param = os.sched_param(max_priority)
                os.sched_setscheduler(0, os.SCHED_FIFO, param)
            except (OSError, AttributeError):
                # Fallback to nice priority
                try:
                    os.nice(-20)  # Highest priority (requires root or capabilities)
                except (OSError, AttributeError):
                    pass  # Continue without priority setting if not permitted
        except (ImportError, AttributeError):
            pass
    
    def _handle_safety_limit(self, device_id: int, status: int) -> None:
        """Handle safety limit trigger"""
        # Only stop motor on border limits (0x10, 0x20), not on safety limits (0x11, 0x12)
        if status == SAFETY_STATUS_MIN_LIMIT or status == SAFETY_STATUS_MAX_LIMIT:
            # Border limit - hard stop
            if self._auto_stop_on_limit:
                try:
                    self.stop_motor()
                    print(f"⚠️  Border limit triggered! Device {device_id:02X}, Status {status:02X} - Motor stopped")
                except Exception as e:
                    print(f"Error stopping motor on border limit: {e}")
        else:
            # Safety limit (0x11, 0x12) - just notify, don't stop
            print(f"ℹ️  Safety limit detected: Device {device_id:02X}, Status {status:02X}")
        
        # Call user callback if provided
        if self._safety_callback is not None:
            try:
                self._safety_callback(device_id, status)
            except Exception as e:
                print(f"Error in safety callback: {e}")
    
    def _safety_listener_loop(self):
        """
        Background thread loop for monitoring safety messages
        Runs at 200 Hz (5ms interval) with highest priority
        """
        self._set_thread_priority()
        
        # 200 Hz = 5ms interval
        CHECK_INTERVAL = 0.005  # 5ms = 200 checks per second
        SAFETY_TIMEOUT = 0.002  # 2ms timeout for 200Hz
        
        while self._safety_listener_running:
            try:
                result = self.check_safety_message(timeout=SAFETY_TIMEOUT)
                if result is not None:
                    device_id, status = result
                    self._handle_safety_limit(device_id, status)
            except Exception:
                # Continue running even if there's an error
                pass
            
            time.sleep(CHECK_INTERVAL)
    
    def start_safety_listener(
        self,
        auto_stop: bool = True,
        callback: Optional[Callable[[int, int], None]] = None
    ) -> None:
        """
        Start background thread to listen for safety CAN messages (ID 0x005)
        
        Args:
            auto_stop: If True, automatically stop motor when limit is triggered
            callback: Optional callback function(device_id, status) called when limit is triggered
        """
        if self._bus is None:
            raise GIM8115Error("Not connected to CAN bus. Call connect() first.")
        
        if self._safety_listener_running:
            # Already running, update settings
            self._auto_stop_on_limit = auto_stop
            self._safety_callback = callback
            return
        
        self._auto_stop_on_limit = auto_stop
        self._safety_callback = callback
        self._safety_listener_running = True
        
        self._safety_listener_thread = threading.Thread(
            target=self._safety_listener_loop,
            daemon=True,
            name="SafetyListener"
        )
        self._safety_listener_thread.start()
        print("Safety listener started (monitoring CAN ID 0x005 at 200 Hz, highest priority)")
    
    def stop_safety_listener(self) -> None:
        """Stop the background safety listener thread"""
        if self._safety_listener_running:
            self._safety_listener_running = False
            if self._safety_listener_thread is not None:
                self._safety_listener_thread.join(timeout=1.0)
                self._safety_listener_thread = None
            print("Safety listener stopped")
    
    def _status_monitor_loop(self):
        """
        Background thread loop for monitoring motor status at 10 Hz
        Reads position, speed, and error status
        """
        # 10 Hz = 100ms interval
        CHECK_INTERVAL = 1.0 / self._status_monitor_rate  # 0.1 seconds for 10 Hz
        
        consecutive_failures = 0
        max_failures = 10  # Warn after 10 consecutive failures
        
        while self._status_monitor_running:
            try:
                # Get motor status (position, speed, error)
                # Use longer timeout to ensure we get responses (80ms allows 40ms per indicator)
                status = self.get_motor_status(timeout=0.08)  # 80ms timeout
                if status is not None:
                    consecutive_failures = 0  # Reset failure counter on success
                    if self._status_callback is not None:
                        try:
                            self._status_callback(status)
                        except Exception as e:
                            print(f"Error in status callback: {e}")
                else:
                    # Status is None - might be timeout or error
                    consecutive_failures += 1
                    if consecutive_failures >= max_failures:
                        print(f"⚠️  Status monitor: {consecutive_failures} consecutive failures (timeout or no response)")
                        consecutive_failures = 0  # Reset to avoid spam
            except Exception as e:
                # Log exception but continue running
                consecutive_failures += 1
                if consecutive_failures >= max_failures:
                    print(f"⚠️  Status monitor error: {e}")
                    consecutive_failures = 0  # Reset to avoid spam
            
            time.sleep(CHECK_INTERVAL)
    
    def start_status_monitor(
        self,
        rate_hz: float = 10.0,
        callback: Optional[Callable[[MotorStatus], None]] = None
    ) -> None:
        """
        Start background thread to monitor motor status (position, speed, error) at specified rate
        
        Args:
            rate_hz: Monitoring rate in Hz (default: 10.0 = 10 times per second)
            callback: Optional callback function(status: MotorStatus) called when status is received
        """
        if self._bus is None:
            raise GIM8115Error("Not connected to CAN bus. Call connect() first.")
        
        if self._status_monitor_running:
            # Already running, update settings
            self._status_monitor_rate = rate_hz
            self._status_callback = callback
            return
        
        self._status_monitor_rate = rate_hz
        self._status_callback = callback
        self._status_monitor_running = True
        self._status_monitor_thread = threading.Thread(
            target=self._status_monitor_loop,
            daemon=True,
            name="StatusMonitor"
        )
        self._status_monitor_thread.start()
        print(f"Status monitor started (monitoring at {rate_hz} Hz)")
    
    def stop_status_monitor(self) -> None:
        """Stop the background status monitor thread"""
        if self._status_monitor_running:
            self._status_monitor_running = False
            if self._status_monitor_thread is not None:
                self._status_monitor_thread.join(timeout=1.0)
                self._status_monitor_thread = None
            print("Status monitor stopped")

