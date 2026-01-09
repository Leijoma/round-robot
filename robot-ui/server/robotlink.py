"""
RobotLink Protocol - Python Implementation
Provides binary protocol communication with ESP32 robot controller via UDP

Protocol Format:
  [SOF0] [SOF1] [TYPE] [LEN] [PAYLOAD...] [CRC_LO] [CRC_HI]

  SOF0, SOF1: Start-of-frame bytes (0xAA, 0x55)
  TYPE: Message type code
  LEN: Payload length (0-255 bytes)
  PAYLOAD: Message-specific data
  CRC: CRC16 checksum (little-endian)
"""

import socket
import struct
import time
from enum import IntEnum
from typing import Optional, Callable, Dict, Any
from dataclasses import dataclass

# Protocol Constants
SOF0 = 0xAA
SOF1 = 0x55
MAX_PAYLOAD = 255

class MessageType(IntEnum):
    """RobotLink message type codes"""
    # Odometry and Status (Robot -> Host)
    MSG_ODOM = 0x01
    MSG_CMD_VEL = 0x02  # Command velocity (v, w) - for Arduino
    MSG_STATUS = 0x03
    MSG_ERROR = 0x04

    # Control Commands (Host -> Robot)
    MSG_SET_VEL = 0x10  # Set motor velocities (left, right) - extended protocol
    MSG_SET_PID = 0x11
    MSG_SET_DEADBAND = 0x12
    MSG_SET_CONFIG = 0x13
    MSG_ENABLE_STREAM = 0x14
    MSG_GET_CONFIG = 0x15
    MSG_CONFIG_RESP = 0x16
    MSG_SAVE_CONFIG = 0x17
    MSG_LOAD_CONFIG = 0x18
    MSG_ZERO_ENCODERS = 0x19
    MSG_STOP = 0x1A  # FIXED: Was 0x15, should be 0x1A
    MSG_RESET_POSE = 0x1B  # Reset pose to origin without zeroing encoders
    MSG_SET_PID_PER_MOTOR = 0x1C  # Set per-motor PID parameters (left/right separate)
    MSG_SET_HEADING_HOLD_KP = 0x1D  # Set angular velocity feedback gain
    MSG_STATUS_EXTENDED = 0x1E  # Extended status with per-motor PID and heading hold Kp
    MSG_ACK = 0x1F  # Acknowledgment of received command

    # Lidar Messages
    MSG_LIDAR_SCAN = 0x20      # ESP32 -> Host
    MSG_LIDAR_STATUS = 0x21    # ESP32 -> Host
    MSG_LIDAR_ENABLE = 0x22    # Host -> ESP32
    MSG_LIDAR_SET_RPM = 0x23   # Host -> ESP32
    MSG_SET_ROBOT_PARAMS = 0x24  # Set robot geometry parameters

    # Ping/Pong and Error
    MSG_NACK = 0x7D  # Negative acknowledgment (error)
    MSG_PING = 0x7E
    MSG_PONG = 0x7F


# Payload Structures
@dataclass
class OdomPayload:
    """Odometry data from robot"""
    timestamp: int         # milliseconds
    delta_left: int        # encoder delta (ticks since last message)
    delta_right: int       # encoder delta (ticks since last message)
    x_mm: int              # X position in mm
    y_mm: int              # Y position in mm
    theta_mrad: int        # Heading in milliradians

    # Computed values (not in payload)
    encoder_left: int = 0      # total ticks (computed)
    encoder_right: int = 0     # total ticks (computed)
    vel_left: float = 0.0      # m/s (computed from delta)
    vel_right: float = 0.0     # m/s (computed from delta)
    pwm_left: int = 0          # Not available in this format
    pwm_right: int = 0         # Not available in this format

    @classmethod
    def unpack(cls, data: bytes) -> 'OdomPayload':
        """Unpack Arduino odometry format: <Iiiiih> (22 bytes)"""
        if len(data) == 22:
            # Arduino format (updated): timestamp, delta_L, delta_R, x, y, theta
            # All positions are SIGNED int32 to handle negative coordinates
            values = struct.unpack('<Iiiiih', data)
            return cls(
                timestamp=values[0],
                delta_left=values[1],
                delta_right=values[2],
                x_mm=values[3],
                y_mm=values[4],
                theta_mrad=values[5]
            )
        elif len(data) == 18:
            # Legacy Arduino format: timestamp, delta_L, delta_R, x, y, theta
            # Old format with int16 deltas (kept for backwards compatibility)
            values = struct.unpack('<Ihhiih', data)
            return cls(
                timestamp=values[0],
                delta_left=values[1],
                delta_right=values[2],
                x_mm=values[3],
                y_mm=values[4],
                theta_mrad=values[5]
            )
        elif len(data) == 24:
            # Legacy format: enc_L, enc_R, vel_L, vel_R, pwm_L, pwm_R, timestamp
            values = struct.unpack('<iiffhhI', data)
            return cls(
                timestamp=values[6],
                delta_left=0,
                delta_right=0,
                x_mm=0,
                y_mm=0,
                theta_mrad=0,
                encoder_left=values[0],
                encoder_right=values[1],
                vel_left=values[2],
                vel_right=values[3],
                pwm_left=values[4],
                pwm_right=values[5]
            )
        else:
            raise ValueError(f"Invalid odometry payload size: {len(data)} bytes (expected 18, 22, or 24)")


@dataclass
class StatusPayload:
    """Robot status information (legacy - only left motor PID)"""
    kp: float
    ki: float
    kd: float
    deadband: tuple  # (left_fwd, left_rev, right_fwd, right_rev)
    pid_enabled: bool
    stream_enabled: bool
    stream_interval: int
    frames_received: int
    frames_sent: int
    uptime: int

    @classmethod
    def unpack(cls, data: bytes) -> 'StatusPayload':
        values = struct.unpack('<fffffffBBHIII', data)
        return cls(
            kp=values[0],
            ki=values[1],
            kd=values[2],
            deadband=(values[3], values[4], values[5], values[6]),
            pid_enabled=bool(values[7]),
            stream_enabled=bool(values[8]),
            stream_interval=values[9],
            frames_received=values[10],
            frames_sent=values[11],
            uptime=values[12]
        )


@dataclass
class StatusExtendedPayload:
    """Extended robot status with per-motor PID"""
    left_kp: float
    left_ki: float
    left_kd: float
    right_kp: float
    right_ki: float
    right_kd: float
    deadband: tuple  # (left_fwd, left_rev, right_fwd, right_rev)
    heading_hold_kp: float
    pid_enabled: bool
    stream_enabled: bool
    stream_interval: int
    frames_received: int
    frames_sent: int
    uptime: int

    @classmethod
    def unpack(cls, data: bytes) -> 'StatusExtendedPayload':
        """Unpack StatusExtendedPayload: 12+12+16+4+2+2+12 = 60 bytes"""
        values = struct.unpack('<fffffffffffBBHIII', data)
        return cls(
            left_kp=values[0],
            left_ki=values[1],
            left_kd=values[2],
            right_kp=values[3],
            right_ki=values[4],
            right_kd=values[5],
            deadband=(values[6], values[7], values[8], values[9]),
            heading_hold_kp=values[10],
            pid_enabled=bool(values[11]),
            stream_enabled=bool(values[12]),
            stream_interval=values[13],
            frames_received=values[14],
            frames_sent=values[15],
            uptime=values[16]
        )


@dataclass
class AckPayload:
    """Acknowledgment of received command"""
    original_msg_type: int  # The message type being acknowledged
    status: int  # 0=success, non-zero=error code

    @classmethod
    def unpack(cls, data: bytes) -> 'AckPayload':
        """Unpack AckPayload: 4 bytes (BBxx)"""
        values = struct.unpack('<BB', data[:2])
        return cls(
            original_msg_type=values[0],
            status=values[1]
        )


@dataclass
class NackPayload:
    """Negative acknowledgment (error)"""
    original_msg_type: int  # The message type that failed
    error_code: int  # Error code

    @classmethod
    def unpack(cls, data: bytes) -> 'NackPayload':
        """Unpack NackPayload: 4 bytes (BBxx)"""
        values = struct.unpack('<BB', data[:2])
        return cls(
            original_msg_type=values[0],
            error_code=values[1]
        )


@dataclass
class ConfigPayload:
    """Robot configuration"""
    wheel_diameter: float
    wheelbase: float
    ticks_per_rev: float
    invert_left: bool
    invert_right: bool
    balance_enable: bool
    balance_gain: float

    @classmethod
    def unpack(cls, data: bytes) -> 'ConfigPayload':
        values = struct.unpack('<fffBBBxf', data)
        return cls(
            wheel_diameter=values[0],
            wheelbase=values[1],
            ticks_per_rev=values[2],
            invert_left=bool(values[3]),
            invert_right=bool(values[4]),
            balance_enable=bool(values[5]),
            balance_gain=values[6]
        )


@dataclass
class LidarStatusPayload:
    """Lidar status information"""
    current_rpm: int
    target_rpm: int
    motor_running: bool
    rpm_control_enabled: bool
    motor_speed: int
    packets_received: int
    packets_invalid: int
    scans_complete: int
    timestamp: int

    @classmethod
    def unpack(cls, data: bytes) -> 'LidarStatusPayload':
        values = struct.unpack('<HHBBBxIIII', data)
        return cls(
            current_rpm=values[0],
            target_rpm=values[1],
            motor_running=bool(values[2]),
            rpm_control_enabled=bool(values[3]),
            motor_speed=values[4],
            packets_received=values[5],
            packets_invalid=values[6],
            scans_complete=values[7],
            timestamp=values[8]
        )


@dataclass
class LidarReading:
    """Single lidar reading"""
    distance_mm: int
    signal_strength: int
    invalid: bool
    warning: bool


@dataclass
class LidarScanPayload:
    """Lidar scan data"""
    timestamp: int
    rpm: int
    start_angle: int
    readings: list  # List of LidarReading

    @classmethod
    def unpack(cls, data: bytes) -> 'LidarScanPayload':
        # Unpack header
        timestamp, rpm, start_angle, num_readings = struct.unpack('<IHHBxxx', data[:12])

        # Unpack readings
        readings = []
        offset = 12
        for i in range(num_readings):
            dist, strength, flags = struct.unpack('<HHB', data[offset:offset+5])
            readings.append(LidarReading(
                distance_mm=dist,
                signal_strength=strength,
                invalid=bool(flags & 0x01),
                warning=bool(flags & 0x02)
            ))
            offset += 5

        return cls(timestamp, rpm, start_angle, readings)


class RobotLink:
    """RobotLink protocol handler for UDP communication"""

    def __init__(self, host: str = '192.168.4.1', port: int = 5000):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 0))  # IMPORTANT: Bind to any available port to receive responses!
        self.sock.settimeout(0.1)  # 100ms timeout for non-blocking reads

        # Statistics
        self.frames_sent = 0
        self.frames_received = 0
        self.crc_errors = 0

        # Message callbacks
        self.callbacks: Dict[int, Callable] = {}

    def register_callback(self, msg_type: MessageType, callback: Callable):
        """Register a callback for a specific message type"""
        self.callbacks[int(msg_type)] = callback

    def _crc16(self, data: bytes) -> int:
        """Calculate CRC16-CCITT-FALSE checksum (polynomial 0x1021)"""
        crc = 0xFFFF
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
            crc &= 0xFFFF  # Keep it 16-bit
        return crc

    def send_frame(self, msg_type: MessageType, payload: bytes = b'') -> bool:
        """Send a RobotLink frame via UDP"""
        if len(payload) > MAX_PAYLOAD:
            print(f"Error: Payload too large ({len(payload)} > {MAX_PAYLOAD})")
            return False

        # Build frame header
        frame = bytearray([SOF0, SOF1, int(msg_type), len(payload)])
        frame.extend(payload)

        # Calculate CRC over TYPE, LEN, and PAYLOAD (skip SOF bytes)
        crc = self._crc16(frame[2:])
        frame.extend(struct.pack('<H', crc))

        # Send via UDP
        try:
            self.sock.sendto(frame, (self.host, self.port))
            self.frames_sent += 1
            return True
        except Exception as e:
            print(f"Send error: {e}")
            return False

    def receive_frame(self) -> Optional[tuple]:
        """Receive and parse a RobotLink frame from UDP
        Returns: (msg_type, payload) or None if no valid frame"""
        try:
            data, addr = self.sock.recvfrom(1024)
        except socket.timeout:
            return None
        except Exception as e:
            print(f"Receive error: {e}")
            return None

        # Validate minimum frame size
        if len(data) < 6:
            return None

        # Validate SOF
        if data[0] != SOF0 or data[1] != SOF1:
            return None

        msg_type = data[2]
        payload_len = data[3]

        # Validate frame length
        expected_len = 4 + payload_len + 2  # header + payload + crc
        if len(data) != expected_len:
            return None

        # Validate CRC (calculated over TYPE, LEN, PAYLOAD - not SOF bytes)
        rx_crc = struct.unpack('<H', data[-2:])[0]
        calc_crc = self._crc16(data[2:-2])  # Skip SOF bytes, exclude CRC bytes
        if rx_crc != calc_crc:
            self.crc_errors += 1
            return None

        # Extract payload
        payload = data[4:4+payload_len] if payload_len > 0 else b''

        self.frames_received += 1
        return (msg_type, payload)

    def process_messages(self, timeout: float = 0.1) -> int:
        """Process incoming messages and call registered callbacks
        Returns: Number of messages processed"""
        count = 0
        start_time = time.time()

        while time.time() - start_time < timeout:
            result = self.receive_frame()
            if result is None:
                continue

            msg_type, payload = result
            count += 1

            # Call registered callback if available
            if msg_type in self.callbacks:
                try:
                    self.callbacks[msg_type](payload)
                except Exception as e:
                    print(f"Callback error for message type {msg_type:#x}: {e}")

        return count

    # ===== Control Commands =====

    def set_velocity(self, vel_left: float, vel_right: float) -> bool:
        """Set motor velocities in m/s (extended protocol - MSG_SET_VEL 0x10)"""
        payload = struct.pack('<ff', vel_left, vel_right)
        return self.send_frame(MessageType.MSG_SET_VEL, payload)

    def cmd_vel(self, v_mps: float, w_radps: float) -> bool:
        """Command velocity using (v, w) format for Arduino (MSG_CMD_VEL 0x02)

        Args:
            v_mps: Linear velocity in meters per second
            w_radps: Angular velocity in radians per second

        Returns:
            True if frame was sent successfully
        """
        # Convert to Arduino format: int16 mm/s and int16 mrad/s
        v_mm_s = int(v_mps * 1000)  # m/s to mm/s
        w_mrad_s = int(w_radps * 1000)  # rad/s to mrad/s

        # Clamp to int16 range
        v_mm_s = max(-32768, min(32767, v_mm_s))
        w_mrad_s = max(-32768, min(32767, w_mrad_s))

        payload = struct.pack('<hh', v_mm_s, w_mrad_s)
        return self.send_frame(MessageType.MSG_CMD_VEL, payload)

    def stop(self) -> bool:
        """Emergency stop - immediately halt motors"""
        return self.send_frame(MessageType.MSG_STOP)

    def reset_pose(self) -> bool:
        """Reset Arduino pose to origin (0, 0, 0) without zeroing encoders"""
        return self.send_frame(MessageType.MSG_RESET_POSE)

    def zero_encoders(self) -> bool:
        """Zero encoder counts and reset pose to origin"""
        return self.send_frame(MessageType.MSG_ZERO_ENCODERS)

    def set_pid(self, kp: float, ki: float, kd: float) -> bool:
        """Set PID controller parameters (both motors)"""
        payload = struct.pack('<fff', kp, ki, kd)
        return self.send_frame(MessageType.MSG_SET_PID, payload)

    def set_pid_per_motor(self, left_kp: float, left_ki: float, left_kd: float,
                          right_kp: float, right_ki: float, right_kd: float) -> bool:
        """Set per-motor PID controller parameters"""
        payload = struct.pack('<ffffff', left_kp, left_ki, left_kd, right_kp, right_ki, right_kd)
        return self.send_frame(MessageType.MSG_SET_PID_PER_MOTOR, payload)

    def set_deadband(self, left_fwd: float, left_rev: float,
                     right_fwd: float, right_rev: float) -> bool:
        """Set motor deadband compensation"""
        payload = struct.pack('<ffff', left_fwd, left_rev, right_fwd, right_rev)
        return self.send_frame(MessageType.MSG_SET_DEADBAND, payload)

    def set_heading_hold_kp(self, kp: float) -> bool:
        """Set angular velocity feedback gain for heading hold"""
        payload = struct.pack('<f', kp)
        return self.send_frame(MessageType.MSG_SET_HEADING_HOLD_KP, payload)

    def set_robot_params(self, wheel_diameter: float, wheelbase: float, ticks_per_rev: float) -> bool:
        """Set robot geometry parameters (meters, meters, ticks)"""
        payload = struct.pack('<fff', wheel_diameter, wheelbase, ticks_per_rev)
        return self.send_frame(MessageType.MSG_SET_ROBOT_PARAMS, payload)

    def enable_stream(self, enable: bool, interval_ms: int = 100) -> bool:
        """Enable/disable continuous odometry streaming"""
        payload = struct.pack('<BH', 1 if enable else 0, interval_ms)
        return self.send_frame(MessageType.MSG_ENABLE_STREAM, payload)

    def request_status(self) -> bool:
        """Request current robot status (PID, deadband, etc.) - legacy format"""
        return self.send_frame(MessageType.MSG_STATUS)

    def request_status_extended(self) -> bool:
        """Request extended robot status with per-motor PID"""
        return self.send_frame(MessageType.MSG_STATUS_EXTENDED)

    def save_config(self) -> bool:
        """Save current configuration to EEPROM"""
        return self.send_frame(MessageType.MSG_SAVE_CONFIG)

    def load_config(self) -> bool:
        """Load configuration from EEPROM"""
        return self.send_frame(MessageType.MSG_LOAD_CONFIG)

    def get_config(self) -> bool:
        """Request robot configuration"""
        return self.send_frame(MessageType.MSG_GET_CONFIG)

    def set_config(self, wheel_diameter: float, wheelbase: float,
                   ticks_per_rev: float, invert_left: bool = False,
                   invert_right: bool = False, balance_enable: bool = False,
                   balance_gain: float = 0.0) -> bool:
        """Set robot configuration"""
        payload = struct.pack('<fffBBBxf',
                            wheel_diameter, wheelbase, ticks_per_rev,
                            1 if invert_left else 0,
                            1 if invert_right else 0,
                            1 if balance_enable else 0,
                            balance_gain)
        return self.send_frame(MessageType.MSG_SET_CONFIG, payload)

    # ===== Lidar Commands =====

    def lidar_enable(self, enable: bool) -> bool:
        """Enable/disable lidar motor"""
        payload = struct.pack('<B', 1 if enable else 0)
        return self.send_frame(MessageType.MSG_LIDAR_ENABLE, payload)

    def lidar_set_rpm(self, target_rpm: int) -> bool:
        """Set lidar target RPM (typically 200-300)"""
        payload = struct.pack('<H', target_rpm)
        return self.send_frame(MessageType.MSG_LIDAR_SET_RPM, payload)

    # ===== Utility =====

    def ping(self, timestamp: Optional[int] = None) -> bool:
        """Send ping message (for latency measurement)"""
        if timestamp is None:
            timestamp = int(time.time() * 1000)
        payload = struct.pack('<I', timestamp)
        return self.send_frame(MessageType.MSG_PING, payload)

    def close(self):
        """Close UDP socket"""
        self.sock.close()

    def get_stats(self) -> Dict[str, int]:
        """Get protocol statistics"""
        return {
            'frames_sent': self.frames_sent,
            'frames_received': self.frames_received,
            'crc_errors': self.crc_errors
        }


# Example usage and testing
if __name__ == '__main__':
    # Create RobotLink instance
    robot = RobotLink(host='192.168.4.1', port=5000)

    # Register callbacks for incoming messages
    def on_odom(payload):
        odom = OdomPayload.unpack(payload)
        print(f"Odom: L={odom.vel_left:.3f} m/s, R={odom.vel_right:.3f} m/s, "
              f"PWM=[{odom.pwm_left}, {odom.pwm_right}]")

    def on_lidar_status(payload):
        status = LidarStatusPayload.unpack(payload)
        print(f"Lidar: {status.current_rpm} RPM (target {status.target_rpm}), "
              f"Motor={'ON' if status.motor_running else 'OFF'}, "
              f"Packets={status.packets_received}, Invalid={status.packets_invalid}")

    def on_lidar_scan(payload):
        scan = LidarScanPayload.unpack(payload)
        valid_readings = [r for r in scan.readings if not r.invalid]
        print(f"Lidar Scan: {len(valid_readings)}/{len(scan.readings)} valid readings, "
              f"Start angle={scan.start_angle}°, RPM={scan.rpm}")

    robot.register_callback(MessageType.MSG_ODOM, on_odom)
    robot.register_callback(MessageType.MSG_LIDAR_STATUS, on_lidar_status)
    robot.register_callback(MessageType.MSG_LIDAR_SCAN, on_lidar_scan)

    print("RobotLink Test - Connecting to robot...")
    print(f"Target: {robot.host}:{robot.port}\n")

    # Enable lidar
    print("Enabling lidar motor...")
    robot.lidar_enable(True)
    robot.lidar_set_rpm(220)

    # Enable odometry streaming
    print("Enabling odometry stream (100ms interval)...")
    robot.enable_stream(True, 100)

    # Send a test velocity command
    print("Setting velocity: 0.1 m/s forward\n")
    robot.set_velocity(0.1, 0.1)

    # Process messages for 10 seconds
    try:
        print("Processing messages (Ctrl+C to stop)...")
        start_time = time.time()
        while time.time() - start_time < 10:
            robot.process_messages(timeout=0.5)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping...")

    # Stop robot
    robot.stop()

    # Print statistics
    stats = robot.get_stats()
    print(f"\n--- Statistics ---")
    print(f"Frames sent: {stats['frames_sent']}")
    print(f"Frames received: {stats['frames_received']}")
    print(f"CRC errors: {stats['crc_errors']}")

    robot.close()
