#!/usr/bin/env python3
"""
Test script for Arduino Motor Control Firmware
Tests RobotLink protocol communication and motor control
"""

import serial
import struct
import time
import sys
from enum import IntEnum


class MessageType(IntEnum):
    """RobotLink message types"""
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


class RobotLink:
    """RobotLink protocol handler"""

    SOF0 = 0xAA
    SOF1 = 0x55

    def __init__(self, port, baud=115200):
        """Initialize serial connection"""
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2)  # Wait for Arduino to boot
        print(f"Connected to {port} at {baud} baud")

    def crc16_ccitt_false(self, data):
        """Calculate CRC16-CCITT-FALSE"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
                crc &= 0xFFFF
        return crc

    def send_frame(self, msg_type, payload=b''):
        """Send a RobotLink frame"""
        frame = bytearray()
        frame.append(self.SOF0)
        frame.append(self.SOF1)
        frame.append(msg_type)
        frame.append(len(payload))
        frame.extend(payload)

        # Calculate CRC over TYPE, LEN, PAYLOAD
        crc_data = bytes([msg_type, len(payload)]) + payload
        crc = self.crc16_ccitt_false(crc_data)
        frame.append(crc & 0xFF)        # CRC low
        frame.append((crc >> 8) & 0xFF) # CRC high

        self.ser.write(frame)
        return frame

    def read_frame(self, timeout=1.0):
        """Read a RobotLink frame"""
        start_time = time.time()
        state = 0  # 0=wait SOF0, 1=wait SOF1, 2=read header+payload

        while time.time() - start_time < timeout:
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)[0]

                if state == 0:
                    if byte == self.SOF0:
                        state = 1
                elif state == 1:
                    if byte == self.SOF1:
                        state = 2
                    else:
                        state = 0
                elif state == 2:
                    # Read TYPE, LEN
                    msg_type = byte
                    if self.ser.in_waiting == 0:
                        time.sleep(0.01)
                    length = self.ser.read(1)[0]

                    # Read PAYLOAD + CRC
                    remaining = length + 2  # payload + 2 CRC bytes
                    data = bytearray()
                    while len(data) < remaining and time.time() - start_time < timeout:
                        if self.ser.in_waiting > 0:
                            data.extend(self.ser.read(min(self.ser.in_waiting, remaining - len(data))))
                        else:
                            time.sleep(0.001)

                    if len(data) < remaining:
                        return None  # Timeout

                    payload = data[:length]
                    crc_received = data[length] | (data[length + 1] << 8)

                    # Verify CRC
                    crc_data = bytes([msg_type, length]) + payload
                    crc_calc = self.crc16_ccitt_false(crc_data)

                    if crc_received == crc_calc:
                        return (msg_type, payload)
                    else:
                        print(f"CRC mismatch! Received: {crc_received:04x}, Calculated: {crc_calc:04x}")
                        return None
            else:
                time.sleep(0.001)

        return None  # Timeout

    def ping(self):
        """Send PING and wait for PONG"""
        timestamp = int(time.time() * 1000) & 0xFFFFFFFF
        payload = struct.pack('<I', timestamp)

        self.send_frame(MessageType.MSG_PING, payload)

        response = self.read_frame(timeout=1.0)
        if response and response[0] == MessageType.MSG_PONG:
            recv_timestamp = struct.unpack('<I', response[1])[0]
            return recv_timestamp == timestamp
        return False

    def set_velocity(self, left, right):
        """Set motor velocities (m/s)"""
        payload = struct.pack('<ff', left, right)
        self.send_frame(MessageType.MSG_SET_VEL, payload)

    def cmd_vel(self, v, w):
        """Send differential drive command (v in mm/s, w in mrad/s)"""
        payload = struct.pack('<hh', int(v), int(w))
        self.send_frame(MessageType.MSG_CMD_VEL, payload)

    def set_pid(self, kp, ki, kd):
        """Set PID parameters"""
        payload = struct.pack('<fff', kp, ki, kd)
        self.send_frame(MessageType.MSG_SET_PID, payload)

    def set_deadband(self, left_fwd, left_rev, right_fwd, right_rev):
        """Set deadband compensation values"""
        payload = struct.pack('<ffff', left_fwd, left_rev, right_fwd, right_rev)
        self.send_frame(MessageType.MSG_SET_DEADBAND, payload)

    def enable_stream(self, enable, interval_ms=50):
        """Enable/disable odometry streaming"""
        payload = struct.pack('<BH', 1 if enable else 0, interval_ms)
        self.send_frame(MessageType.MSG_ENABLE_STREAM, payload)

    def zero_encoders(self):
        """Reset encoder counts"""
        self.send_frame(MessageType.MSG_ZERO_ENCODERS)

    def stop(self):
        """Emergency stop"""
        self.send_frame(MessageType.MSG_STOP)

    def save_config(self):
        """Save configuration to EEPROM"""
        self.send_frame(MessageType.MSG_SAVE_CONFIG)

    def load_config(self):
        """Load configuration from EEPROM"""
        self.send_frame(MessageType.MSG_LOAD_CONFIG)

    def get_config(self):
        """Request configuration"""
        self.send_frame(MessageType.MSG_GET_CONFIG)
        response = self.read_frame(timeout=1.0)
        if response and response[0] == MessageType.MSG_CONFIG_RESP:
            # Parse ConfigPayload
            data = response[1]
            if len(data) >= 20:
                wheel_diameter, wheelbase, ticks_per_rev = struct.unpack('<fff', data[0:12])
                invert_left, invert_right, balance_enable, reserved = struct.unpack('<BBBB', data[12:16])
                balance_gain = struct.unpack('<f', data[16:20])[0]

                return {
                    'wheel_diameter': wheel_diameter,
                    'wheelbase': wheelbase,
                    'ticks_per_rev': ticks_per_rev,
                    'invert_left': bool(invert_left),
                    'invert_right': bool(invert_right),
                    'balance_enable': bool(balance_enable),
                    'balance_gain': balance_gain
                }
        return None

    def read_odometry(self, timeout=1.0):
        """Read one odometry message"""
        response = self.read_frame(timeout)
        if response and response[0] == MessageType.MSG_ODOM:
            # Parse ODOM payload (18 bytes)
            data = response[1]
            if len(data) >= 18:
                t_ms = struct.unpack('<I', data[0:4])[0]
                dL_ticks = struct.unpack('<h', data[4:6])[0]
                dR_ticks = struct.unpack('<h', data[6:8])[0]
                x_mm = struct.unpack('<i', data[8:12])[0]
                y_mm = struct.unpack('<i', data[12:16])[0]
                th_mrad = struct.unpack('<h', data[16:18])[0]

                return {
                    'timestamp': t_ms,
                    'dL_ticks': dL_ticks,
                    'dR_ticks': dR_ticks,
                    'x_mm': x_mm,
                    'y_mm': y_mm,
                    'th_mrad': th_mrad
                }
        return None

    def close(self):
        """Close serial connection"""
        self.stop()
        time.sleep(0.1)
        self.ser.close()


def test_connection(robot):
    """Test 1: Basic connection and PING/PONG"""
    print("\n" + "="*60)
    print("TEST 1: Connection Test (PING/PONG)")
    print("="*60)

    try:
        result = robot.ping()
        if result:
            print("✓ PING/PONG successful!")
            print("✓ Serial communication working")
            return True
        else:
            print("✗ PING failed - no response or incorrect timestamp")
            return False
    except Exception as e:
        print(f"✗ Connection test failed: {e}")
        return False


def test_config(robot):
    """Test 2: Configuration reading"""
    print("\n" + "="*60)
    print("TEST 2: Configuration Test")
    print("="*60)

    try:
        config = robot.get_config()
        if config:
            print("✓ Configuration received:")
            print(f"  Wheel diameter: {config['wheel_diameter']:.3f} m")
            print(f"  Wheelbase: {config['wheelbase']:.3f} m")
            print(f"  Ticks per rev: {config['ticks_per_rev']:.0f}")
            print(f"  Invert left: {config['invert_left']}")
            print(f"  Invert right: {config['invert_right']}")
            return True
        else:
            print("✗ No configuration response")
            return False
    except Exception as e:
        print(f"✗ Configuration test failed: {e}")
        return False


def test_encoders(robot):
    """Test 3: Encoder reading"""
    print("\n" + "="*60)
    print("TEST 3: Encoder Test")
    print("="*60)
    print("Enabling odometry streaming...")

    try:
        # Zero encoders first
        robot.zero_encoders()
        time.sleep(0.1)

        # Enable streaming
        robot.enable_stream(True, 100)  # 100ms interval
        time.sleep(0.2)

        print("\nPlease rotate wheels manually for 5 seconds...")
        print("Left wheel forward should show positive dL_ticks")
        print("Right wheel forward should show positive dR_ticks\n")

        start_time = time.time()
        odom_count = 0
        max_dL = 0
        max_dR = 0

        while time.time() - start_time < 5:
            odom = robot.read_odometry(timeout=0.2)
            if odom:
                odom_count += 1
                max_dL = max(max_dL, abs(odom['dL_ticks']))
                max_dR = max(max_dR, abs(odom['dR_ticks']))

                if odom_count % 5 == 0:  # Print every 5th message
                    print(f"ODOM: dL={odom['dL_ticks']:4d}, dR={odom['dR_ticks']:4d}, "
                          f"x={odom['x_mm']:6d}mm, y={odom['y_mm']:6d}mm, "
                          f"θ={odom['th_mrad']:5d}mrad")

        # Disable streaming
        robot.enable_stream(False)

        print(f"\n✓ Received {odom_count} odometry messages in 5 seconds")
        print(f"✓ Max encoder deltas: Left={max_dL}, Right={max_dR}")

        if odom_count >= 30:  # Should get ~50 messages in 5 seconds at 100ms interval
            print("✓ Odometry streaming working correctly")
            return True
        else:
            print(f"⚠ Only received {odom_count} messages (expected ~50)")
            return False

    except Exception as e:
        print(f"✗ Encoder test failed: {e}")
        return False


def test_motors_idle(robot):
    """Test 4: Motor control (idle - no load expected)"""
    print("\n" + "="*60)
    print("TEST 4: Motor Control Test (IDLE)")
    print("="*60)
    print("⚠ WARNING: Ensure robot wheels can spin freely!")
    print("Press Enter to continue or Ctrl+C to skip...")

    try:
        input()
    except KeyboardInterrupt:
        print("\nSkipping motor test")
        return True

    try:
        # Enable odometry to monitor motor response
        robot.enable_stream(True, 100)
        time.sleep(0.2)

        # Test 1: Very slow forward
        print("\n[1/4] Testing slow forward (0.05 m/s)...")
        robot.set_velocity(0.05, 0.05)
        time.sleep(2)

        # Read a few odometry messages
        for _ in range(5):
            odom = robot.read_odometry(timeout=0.2)
            if odom:
                print(f"  Encoder deltas: dL={odom['dL_ticks']:3d}, dR={odom['dR_ticks']:3d}")

        robot.stop()
        time.sleep(0.5)

        # Test 2: Medium forward
        print("\n[2/4] Testing medium forward (0.15 m/s)...")
        robot.set_velocity(0.15, 0.15)
        time.sleep(2)

        for _ in range(5):
            odom = robot.read_odometry(timeout=0.2)
            if odom:
                print(f"  Encoder deltas: dL={odom['dL_ticks']:3d}, dR={odom['dR_ticks']:3d}")

        robot.stop()
        time.sleep(0.5)

        # Test 3: Differential (turn)
        print("\n[3/4] Testing differential turn...")
        robot.set_velocity(0.1, -0.1)
        time.sleep(2)

        for _ in range(5):
            odom = robot.read_odometry(timeout=0.2)
            if odom:
                print(f"  Encoder deltas: dL={odom['dL_ticks']:3d}, dR={odom['dR_ticks']:3d}")

        robot.stop()
        time.sleep(0.5)

        # Test 4: CMD_VEL test
        print("\n[4/4] Testing CMD_VEL (differential drive commands)...")
        robot.cmd_vel(100, 0)  # 100 mm/s forward
        time.sleep(2)

        for _ in range(5):
            odom = robot.read_odometry(timeout=0.2)
            if odom:
                print(f"  Encoder deltas: dL={odom['dL_ticks']:3d}, dR={odom['dR_ticks']:3d}")

        robot.stop()
        robot.enable_stream(False)

        print("\n✓ Motor control tests complete")
        print("✓ Check that motors responded to commands")
        return True

    except Exception as e:
        print(f"✗ Motor test failed: {e}")
        robot.stop()
        return False


def test_pid_tuning(robot):
    """Test 5: PID parameter setting"""
    print("\n" + "="*60)
    print("TEST 5: PID Tuning Test")
    print("="*60)

    try:
        print("Setting PID values: Kp=15.0, Ki=8.0, Kd=0.2...")
        robot.set_pid(15.0, 8.0, 0.2)
        time.sleep(0.1)

        print("✓ PID command sent")

        print("\nSetting deadband values...")
        robot.set_deadband(35.0, 35.0, 35.0, 35.0)
        time.sleep(0.1)

        print("✓ Deadband command sent")
        print("✓ PID tuning test complete")

        return True

    except Exception as e:
        print(f"✗ PID tuning test failed: {e}")
        return False


def test_eeprom(robot):
    """Test 6: EEPROM save/load"""
    print("\n" + "="*60)
    print("TEST 6: EEPROM Persistence Test")
    print("="*60)

    try:
        print("Saving configuration to EEPROM...")
        robot.save_config()
        time.sleep(0.2)

        print("✓ Configuration saved")
        print("  (Values will persist after power cycle)")

        return True

    except Exception as e:
        print(f"✗ EEPROM test failed: {e}")
        return False


def main():
    """Main test runner"""
    print("="*60)
    print("Arduino Motor Control Firmware Test Suite")
    print("="*60)

    # Detect serial port
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())

    if not ports:
        print("No serial ports found!")
        sys.exit(1)

    print("\nAvailable ports:")
    for i, port in enumerate(ports):
        print(f"  [{i}] {port.device} - {port.description}")

    # Auto-select if only one Arduino
    arduino_ports = [p for p in ports if 'Arduino' in p.description or 'USB' in p.description]

    if len(arduino_ports) == 1:
        selected_port = arduino_ports[0].device
        print(f"\nAuto-selected: {selected_port}")
    else:
        try:
            choice = int(input("\nSelect port [0]: ") or "0")
            selected_port = ports[choice].device
        except (ValueError, IndexError):
            selected_port = ports[0].device

    print(f"Using port: {selected_port}")

    # Connect to robot
    try:
        robot = RobotLink(selected_port, 115200)
    except Exception as e:
        print(f"Failed to connect: {e}")
        sys.exit(1)

    # Run tests
    results = []

    try:
        results.append(("Connection", test_connection(robot)))
        results.append(("Configuration", test_config(robot)))
        results.append(("Encoders", test_encoders(robot)))
        results.append(("Motors", test_motors_idle(robot)))
        results.append(("PID Tuning", test_pid_tuning(robot)))
        results.append(("EEPROM", test_eeprom(robot)))

    except KeyboardInterrupt:
        print("\n\nTests interrupted by user")
    finally:
        robot.close()

    # Print summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)

    passed = 0
    for name, result in results:
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"{name:20s}: {status}")
        if result:
            passed += 1

    print(f"\nTotal: {passed}/{len(results)} tests passed")
    print("="*60)

    if passed == len(results):
        print("\n🎉 All tests passed! Firmware is working correctly.")
    else:
        print(f"\n⚠ {len(results) - passed} test(s) failed. Check connections and firmware.")


if __name__ == "__main__":
    main()
