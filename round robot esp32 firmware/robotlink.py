"""
RobotLink - Python client library for binary framed protocol

Implements the same CRC16-CCITT-FALSE algorithm as the Arduino firmware
and provides a clean API for sending commands and receiving data.
"""

import struct
import serial
import time
from typing import Optional, Callable, Dict, Any
from dataclasses import dataclass
from enum import IntEnum


class MsgType(IntEnum):
    """Message type codes matching Arduino firmware"""
    MSG_ODOM = 0x01
    MSG_CMD_VEL = 0x02
    MSG_STATUS = 0x03

    MSG_SET_VEL = 0x10
    MSG_SET_PID = 0x11
    MSG_SET_DEADBAND = 0x12
    MSG_SET_CONFIG = 0x13
    MSG_ENABLE_STREAM = 0x14
    MSG_GET_CONFIG = 0x15
    MSG_CONFIG_RESP = 0x16
    MSG_SAVE_CONFIG = 0x17
    MSG_LOAD_CONFIG = 0x18
    MSG_ZERO_ENCODERS = 0x19
    MSG_STOP = 0x1A

    MSG_PING = 0x7E
    MSG_PONG = 0x7F


@dataclass
class OdomData:
    """Odometry data from robot"""
    encoder_left: int
    encoder_right: int
    vel_left: float
    vel_right: float
    pwm_left: int
    pwm_right: int
    timestamp: int


@dataclass
class StatusData:
    """Status data from robot"""
    Kp: float
    Ki: float
    Kd: float
    deadband: list  # [L_fwd, L_rev, R_fwd, R_rev]
    pid_enabled: bool
    stream_enabled: bool
    stream_interval: int
    frames_received: int
    frames_sent: int
    uptime: int


@dataclass
class ConfigData:
    """Robot configuration"""
    wheel_diameter: float
    wheelbase: float
    ticks_per_rev: float
    invert_left: bool
    invert_right: bool
    balance_enable: bool
    balance_gain: float


class RobotLink:
    """
    RobotLink binary protocol client

    Usage:
        with RobotLink('/dev/ttyUSB0', baudrate=115200) as robot:
            robot.set_velocity(0.3, 0.3)
            robot.enable_stream(True, 50)

            for i in range(100):
                odom = robot.read_odom(timeout=0.2)
                if odom:
                    print(f"Encoders: {odom.encoder_left}, {odom.encoder_right}")
    """

    SOF0 = 0xAA
    SOF1 = 0x55

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.1):
        """
        Initialize RobotLink connection

        Args:
            port: Serial port name (e.g., '/dev/ttyUSB0', 'COM3')
            baudrate: Baud rate (default 115200)
            timeout: Read timeout in seconds (default 0.1)
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None

        # Statistics
        self.frames_ok = 0
        self.frames_bad_crc = 0
        self.frames_bad_sof = 0
        self.bytes_dropped = 0

        # Receive state machine
        self._reset_rx()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def open(self):
        """Open serial connection"""
        if self.ser is None or not self.ser.is_open:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(0.5)  # Wait for Arduino reset
            # Flush any startup messages
            self.ser.reset_input_buffer()

    def close(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _reset_rx(self):
        """Reset receive state machine"""
        self._rx_state = 'WAIT_SOF0'
        self._rx_type = 0
        self._rx_len = 0
        self._rx_payload = bytearray()
        self._rx_crc = 0
        self._calc_crc = 0

    @staticmethod
    def _crc16_ccitt_false(data: bytes) -> int:
        """
        Calculate CRC16-CCITT-FALSE
        Poly: 0x1021, Init: 0xFFFF, XorOut: 0x0000, RefIn: False, RefOut: False
        """
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc

    def send_frame(self, msg_type: int, payload: bytes = b'') -> bool:
        """
        Send a frame with CRC

        Args:
            msg_type: Message type code
            payload: Payload bytes

        Returns:
            True if sent successfully
        """
        if not self.ser or not self.ser.is_open:
            return False

        # Build frame
        frame = bytearray()
        frame.append(self.SOF0)
        frame.append(self.SOF1)
        frame.append(msg_type)
        frame.append(len(payload))
        frame.extend(payload)

        # Calculate CRC over type, len, payload
        crc_data = bytes([msg_type, len(payload)]) + payload
        crc = self._crc16_ccitt_false(crc_data)

        frame.append(crc & 0xFF)  # CRC low byte
        frame.append((crc >> 8) & 0xFF)  # CRC high byte

        self.ser.write(frame)
        return True

    def _process_byte(self, byte: int) -> Optional[tuple]:
        """
        Process one received byte through state machine

        Returns:
            (type, payload) tuple if frame complete, None otherwise
        """
        if self._rx_state == 'WAIT_SOF0':
            if byte == self.SOF0:
                self._rx_state = 'WAIT_SOF1'
            else:
                self.bytes_dropped += 1

        elif self._rx_state == 'WAIT_SOF1':
            if byte == self.SOF1:
                self._rx_state = 'READ_TYPE'
            else:
                self.bytes_dropped += 1
                self._frames_bad_sof += 1
                self._reset_rx()

        elif self._rx_state == 'READ_TYPE':
            self._rx_type = byte
            self._calc_crc = 0xFFFF
            self._calc_crc = self._crc16_update(self._calc_crc, byte)
            self._rx_state = 'READ_LEN'

        elif self._rx_state == 'READ_LEN':
            self._rx_len = byte
            self._calc_crc = self._crc16_update(self._calc_crc, byte)
            self._rx_payload = bytearray()
            if self._rx_len == 0:
                self._rx_state = 'READ_CRC_LO'
            else:
                self._rx_state = 'READ_PAYLOAD'

        elif self._rx_state == 'READ_PAYLOAD':
            self._rx_payload.append(byte)
            self._calc_crc = self._crc16_update(self._calc_crc, byte)
            if len(self._rx_payload) >= self._rx_len:
                self._rx_state = 'READ_CRC_LO'

        elif self._rx_state == 'READ_CRC_LO':
            self._rx_crc = byte
            self._rx_state = 'READ_CRC_HI'

        elif self._rx_state == 'READ_CRC_HI':
            self._rx_crc |= (byte << 8)

            # Check CRC
            if self._rx_crc == self._calc_crc:
                self.frames_ok += 1
                result = (self._rx_type, bytes(self._rx_payload))
                self._reset_rx()
                return result
            else:
                self.frames_bad_crc += 1
                self._reset_rx()

        return None

    @staticmethod
    def _crc16_update(crc: int, byte: int) -> int:
        """Update CRC with one byte"""
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
        return crc

    def receive_frame(self, timeout: Optional[float] = None) -> Optional[tuple]:
        """
        Receive one complete frame

        Args:
            timeout: Timeout in seconds (None = use default)

        Returns:
            (type, payload) tuple if frame received, None on timeout
        """
        if not self.ser or not self.ser.is_open:
            return None

        if timeout is None:
            timeout = self.timeout

        start_time = time.time()

        while True:
            if time.time() - start_time > timeout:
                return None

            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)[0]
                result = self._process_byte(byte)
                if result is not None:
                    return result
            else:
                time.sleep(0.001)  # Small delay to prevent busy waiting

    # ========================================
    # High-level API
    # ========================================

    def ping(self, timestamp: Optional[int] = None) -> Optional[int]:
        """
        Send PING and wait for PONG

        Args:
            timestamp: Optional timestamp to echo back (uint32)

        Returns:
            Echoed timestamp if PONG received, None on timeout
        """
        if timestamp is None:
            timestamp = int((time.time() * 1000)) % (2**32)

        payload = struct.pack('<I', timestamp)
        self.send_frame(MsgType.MSG_PING, payload)

        frame = self.receive_frame(timeout=1.0)
        if frame and frame[0] == MsgType.MSG_PONG:
            return struct.unpack('<I', frame[1])[0] if len(frame[1]) >= 4 else None
        return None

    def set_velocity(self, left: float, right: float) -> bool:
        """
        Set motor velocities in m/s

        Args:
            left: Left wheel velocity (m/s)
            right: Right wheel velocity (m/s)
        """
        payload = struct.pack('<ff', left, right)
        return self.send_frame(MsgType.MSG_SET_VEL, payload)

    def set_pid(self, Kp: float, Ki: float, Kd: float) -> bool:
        """Set PID gains (applied to both motors)"""
        payload = struct.pack('<fff', Kp, Ki, Kd)
        return self.send_frame(MsgType.MSG_SET_PID, payload)

    def set_deadband(self, left_fwd: float, left_rev: float,
                     right_fwd: float, right_rev: float) -> bool:
        """Set deadband PWM offsets"""
        payload = struct.pack('<ffff', left_fwd, left_rev, right_fwd, right_rev)
        return self.send_frame(MsgType.MSG_SET_DEADBAND, payload)

    def set_config(self, wheel_diameter: float, wheelbase: float,
                   ticks_per_rev: float, invert_left: bool = False,
                   invert_right: bool = False, balance_enable: bool = True,
                   balance_gain: float = 0.02) -> bool:
        """Set robot configuration"""
        payload = struct.pack('<fffBBBBf',
                             wheel_diameter, wheelbase, ticks_per_rev,
                             int(invert_left), int(invert_right),
                             int(balance_enable), 0,  # reserved
                             balance_gain)
        return self.send_frame(MsgType.MSG_SET_CONFIG, payload)

    def get_config(self, timeout: float = 1.0) -> Optional[ConfigData]:
        """Request and receive configuration"""
        self.send_frame(MsgType.MSG_GET_CONFIG)

        frame = self.receive_frame(timeout=timeout)
        if frame and frame[0] == MsgType.MSG_CONFIG_RESP:
            data = struct.unpack('<fffBBBBf', frame[1])
            return ConfigData(
                wheel_diameter=data[0],
                wheelbase=data[1],
                ticks_per_rev=data[2],
                invert_left=bool(data[3]),
                invert_right=bool(data[4]),
                balance_enable=bool(data[5]),
                balance_gain=data[7]
            )
        return None

    def enable_stream(self, enable: bool, interval_ms: int = 50) -> bool:
        """Enable/disable continuous odometry streaming"""
        payload = struct.pack('<BH', int(enable), interval_ms)
        return self.send_frame(MsgType.MSG_ENABLE_STREAM, payload)

    def save_config(self) -> bool:
        """Save current configuration to EEPROM"""
        return self.send_frame(MsgType.MSG_SAVE_CONFIG)

    def load_config(self) -> bool:
        """Load configuration from EEPROM"""
        return self.send_frame(MsgType.MSG_LOAD_CONFIG)

    def zero_encoders(self) -> bool:
        """Zero encoder counts"""
        return self.send_frame(MsgType.MSG_ZERO_ENCODERS)

    def stop(self) -> bool:
        """Emergency stop"""
        return self.send_frame(MsgType.MSG_STOP)

    def read_odom(self, timeout: Optional[float] = None) -> Optional[OdomData]:
        """
        Read one odometry message

        Args:
            timeout: Timeout in seconds

        Returns:
            OdomData if received, None on timeout
        """
        frame = self.receive_frame(timeout=timeout)
        if frame and frame[0] == MsgType.MSG_ODOM:
            # int32 L, int32 R, float vL, float vR, int16 pwmL, int16 pwmR, uint32 time
            data = struct.unpack('<iiffhhI', frame[1])
            return OdomData(
                encoder_left=data[0],
                encoder_right=data[1],
                vel_left=data[2],
                vel_right=data[3],
                pwm_left=data[4],
                pwm_right=data[5],
                timestamp=data[6]
            )
        return None

    def read_status(self, timeout: Optional[float] = None) -> Optional[StatusData]:
        """Read status message"""
        frame = self.receive_frame(timeout=timeout)
        if frame and frame[0] == MsgType.MSG_STATUS:
            # 3 floats (Kp, Ki, Kd), 4 floats (deadband), 2 uint8, 1 uint16, 3 uint32
            data = struct.unpack('<fffffffBBHIII', frame[1])
            return StatusData(
                Kp=data[0],
                Ki=data[1],
                Kd=data[2],
                deadband=[data[3], data[4], data[5], data[6]],
                pid_enabled=bool(data[7]),
                stream_enabled=bool(data[8]),
                stream_interval=data[9],
                frames_received=data[10],
                frames_sent=data[11],
                uptime=data[12]
            )
        return None

    def get_stats(self) -> Dict[str, int]:
        """Get client-side statistics"""
        return {
            'frames_ok': self.frames_ok,
            'frames_bad_crc': self.frames_bad_crc,
            'frames_bad_sof': self.frames_bad_sof,
            'bytes_dropped': self.bytes_dropped
        }


if __name__ == '__main__':
    # Simple test
    import sys

    if len(sys.argv) < 2:
        print("Usage: python robotlink.py <port>")
        print("Example: python robotlink.py /dev/cu.usbmodem212201")
        sys.exit(1)

    port = sys.argv[1]

    print(f"Testing RobotLink on {port}...")

    with RobotLink(port, baudrate=115200) as robot:
        # Test PING
        print("\n1. Testing PING/PONG...")
        timestamp = int(time.time() * 1000) % (2**32)
        echo = robot.ping(timestamp)
        if echo == timestamp:
            print(f"   ✓ PING/PONG successful (latency: <{robot.timeout*1000:.0f}ms)")
        else:
            print(f"   ✗ PING/PONG failed (sent {timestamp}, got {echo})")

        # Get config
        print("\n2. Getting configuration...")
        config = robot.get_config(timeout=1.0)
        if config:
            print(f"   ✓ Config received:")
            print(f"     Wheel diameter: {config.wheel_diameter} m")
            print(f"     Wheelbase: {config.wheelbase} m")
            print(f"     Ticks/rev: {config.ticks_per_rev}")
            print(f"     Balance: {config.balance_enable} (gain={config.balance_gain})")
        else:
            print("   ✗ Failed to get config")

        # Test velocity command
        print("\n3. Testing velocity control (0.2 m/s for 2 seconds)...")
        robot.zero_encoders()
        robot.enable_stream(True, 200)  # Enable streaming at 200ms (5Hz)
        time.sleep(0.3)  # Wait for streaming to start
        robot.set_velocity(0.2, 0.2)

        time.sleep(2.0)

        # Read one odometry message
        odom = robot.read_odom(timeout=0.5)
        if odom:
            print(f"   ✓ Odometry received:")
            print(f"     Encoders: L={odom.encoder_left}, R={odom.encoder_right}")
            print(f"     Velocities: L={odom.vel_left:.3f}, R={odom.vel_right:.3f} m/s")
            print(f"     PWM: L={odom.pwm_left}, R={odom.pwm_right}")
        else:
            print("   ✗ No odometry received")

        # Stop
        robot.stop()

        # Stats
        print("\n4. Statistics:")
        stats = robot.get_stats()
        print(f"   Frames OK: {stats['frames_ok']}")
        print(f"   Bad CRC: {stats['frames_bad_crc']}")
        print(f"   Bytes dropped: {stats['bytes_dropped']}")

    print("\nTest complete!")
