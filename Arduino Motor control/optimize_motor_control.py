#!/usr/bin/env python3
"""
Motor Control Optimizer - Direct ESP32 + Arduino Debug
Tests different velocities and tunes PID/deadband for optimal performance
"""

import socket
import struct
import time
import serial
import threading
from dataclasses import dataclass
from typing import Optional

# Configuration
ESP32_HOST = '192.168.68.52'
ESP32_PORT = 5000
ARDUINO_DEBUG_PORT = '/dev/tty.usbmodem212201'
ARDUINO_DEBUG_BAUD = 115200

# Protocol Constants
SOF0 = 0xAA
SOF1 = 0x55

# Message Types
MSG_ODOM = 0x01
MSG_SET_VEL = 0x10
MSG_SET_PID = 0x11
MSG_SET_DEADBAND = 0x12
MSG_ENABLE_STREAM = 0x14
MSG_STOP = 0x1A

# Test velocities (m/s)
TEST_VELOCITIES = [0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50]

# Robot configuration
WHEEL_DIAMETER = 0.082  # meters
TICKS_PER_REV = 360


@dataclass
class OdomData:
    """Odometry data from robot"""
    timestamp: int
    delta_left: int
    delta_right: int
    x_mm: int
    y_mm: int
    theta_mrad: int
    pwm_left: int
    pwm_right: int


class RobotTester:
    def __init__(self):
        self.udp_sock = None
        self.arduino_serial = None
        self.running = False
        self.last_odom = None
        self.odom_count = 0

    def connect(self):
        """Connect to ESP32 and Arduino"""
        print("\n" + "="*70)
        print("MOTOR CONTROL OPTIMIZER")
        print("="*70)

        # Connect to ESP32 via UDP
        print(f"\n📡 Connecting to ESP32 at {ESP32_HOST}:{ESP32_PORT}...")
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.settimeout(0.1)  # Non-blocking with timeout
        print("✓ UDP socket created")

        # Connect to Arduino debug serial
        print(f"\n🔌 Connecting to Arduino debug at {ARDUINO_DEBUG_PORT}...")
        try:
            self.arduino_serial = serial.Serial(ARDUINO_DEBUG_PORT, ARDUINO_DEBUG_BAUD, timeout=0.1)
            time.sleep(2)  # Wait for Arduino reset
            print("✓ Arduino debug connected")

            # Clear any startup messages
            while self.arduino_serial.in_waiting:
                line = self.arduino_serial.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"  Arduino: {line}")
        except Exception as e:
            print(f"⚠️  Could not connect to Arduino debug: {e}")
            self.arduino_serial = None

        # Start Arduino debug monitor thread
        if self.arduino_serial:
            self.running = True
            debug_thread = threading.Thread(target=self._arduino_debug_monitor, daemon=True)
            debug_thread.start()

        print("\n✓ All connections established")

    def _arduino_debug_monitor(self):
        """Background thread to monitor Arduino debug output"""
        print("🔍 Arduino debug monitor started")
        while self.running:
            try:
                if self.arduino_serial and self.arduino_serial.in_waiting:
                    line = self.arduino_serial.readline().decode('utf-8', errors='ignore').strip()
                    if line and not line.startswith("TX ODOM"):  # Filter out TX ODOM spam
                        print(f"  [Arduino] {line}")
            except Exception as e:
                pass
            time.sleep(0.01)

    def crc16_ccitt_false(self, data):
        """Calculate CRC16-CCITT-FALSE"""
        crc = 0xFFFF
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
                crc &= 0xFFFF
        return crc

    def send_frame(self, msg_type, payload=b''):
        """Send RobotLink frame to ESP32"""
        frame = bytes([SOF0, SOF1, msg_type, len(payload)]) + payload
        crc = self.crc16_ccitt_false(frame[2:])
        frame += struct.pack('<H', crc)

        self.udp_sock.sendto(frame, (ESP32_HOST, ESP32_PORT))

    def enable_streaming(self, enable=True, interval_ms=50):
        """Enable/disable odometry streaming"""
        payload = struct.pack('<BH', 1 if enable else 0, interval_ms)
        self.send_frame(MSG_ENABLE_STREAM, payload)
        print(f"📊 Streaming: {'ENABLED' if enable else 'DISABLED'} @ {interval_ms}ms")
        time.sleep(0.5)

    def stop_motors(self):
        """Emergency stop"""
        self.send_frame(MSG_STOP)
        print("🛑 STOP command sent")
        time.sleep(0.5)

    def set_velocity(self, left_ms, right_ms):
        """Set wheel velocities (m/s)"""
        payload = struct.pack('<ff', left_ms, right_ms)
        self.send_frame(MSG_SET_VEL, payload)

    def set_pid(self, kp, ki, kd):
        """Set PID parameters"""
        payload = struct.pack('<fff', kp, ki, kd)
        self.send_frame(MSG_SET_PID, payload)
        print(f"⚙️  PID set: Kp={kp} Ki={ki} Kd={kd}")
        time.sleep(0.5)

    def set_deadband(self, forward, reverse):
        """Set deadband compensation"""
        payload = struct.pack('<ff', forward, reverse)
        self.send_frame(MSG_SET_DEADBAND, payload)
        print(f"⚙️  Deadband set: Fwd={forward} Rev={reverse}")
        time.sleep(0.5)

    def receive_odom(self):
        """Receive odometry data from ESP32"""
        try:
            data, addr = self.udp_sock.recvfrom(1024)

            # Parse RobotLink frame
            if len(data) < 6:
                return None

            if data[0] != SOF0 or data[1] != SOF1:
                return None

            msg_type = data[2]
            payload_len = data[3]

            if msg_type == MSG_ODOM and payload_len >= 20:
                payload = data[4:4+payload_len]

                # Parse odometry payload
                odom = OdomData(*struct.unpack('<IhhhhhhBB', payload[:20]))
                self.last_odom = odom
                self.odom_count += 1
                return odom

        except socket.timeout:
            pass
        except Exception as e:
            pass

        return None

    def test_velocity(self, target_vel, duration=3.0):
        """Test a specific velocity and collect data"""
        print(f"\n{'='*70}")
        print(f"🎯 Testing velocity: {target_vel:.2f} m/s")
        print(f"{'='*70}")

        # Reset odometry counter
        self.odom_count = 0
        odom_samples = []

        # Send velocity command
        self.set_velocity(target_vel, target_vel)
        print(f"▶️  Motors commanded to {target_vel:.2f} m/s")

        start_time = time.time()
        last_report = start_time

        while time.time() - start_time < duration:
            odom = self.receive_odom()

            if odom:
                odom_samples.append(odom)

                # Report every 0.5 seconds
                if time.time() - last_report >= 0.5:
                    # Calculate actual velocity from encoder deltas
                    # vel = (delta_ticks / ticks_per_rev) * wheel_circumference * sample_rate
                    dt = 0.05  # 50ms = 20 Hz
                    circumference = WHEEL_DIAMETER * 3.14159
                    vel_left = (odom.delta_left / TICKS_PER_REV) * circumference / dt
                    vel_right = (odom.delta_right / TICKS_PER_REV) * circumference / dt

                    print(f"  📈 L: {vel_left:+.3f} m/s ({odom.delta_left:+4d} ticks)  "
                          f"R: {vel_right:+.3f} m/s ({odom.delta_right:+4d} ticks)  "
                          f"PWM: L={odom.pwm_left:3d} R={odom.pwm_right:3d}")

                    last_report = time.time()

            time.sleep(0.01)

        # Stop motors
        self.stop_motors()
        time.sleep(0.5)

        # Analyze results
        if odom_samples:
            print(f"\n📊 Test Results for {target_vel:.2f} m/s:")
            print(f"  Samples collected: {len(odom_samples)}")

            # Calculate average velocity
            dt = 0.05
            circumference = WHEEL_DIAMETER * 3.14159
            avg_vel_left = sum((s.delta_left / TICKS_PER_REV) * circumference / dt for s in odom_samples) / len(odom_samples)
            avg_vel_right = sum((s.delta_right / TICKS_PER_REV) * circumference / dt for s in odom_samples) / len(odom_samples)
            avg_pwm_left = sum(s.pwm_left for s in odom_samples) / len(odom_samples)
            avg_pwm_right = sum(s.pwm_right for s in odom_samples) / len(odom_samples)

            print(f"  Average Left:  {avg_vel_left:.3f} m/s (PWM: {avg_pwm_left:.1f})")
            print(f"  Average Right: {avg_vel_right:.3f} m/s (PWM: {avg_pwm_right:.1f})")
            print(f"  Error: {abs(target_vel - avg_vel_left):.3f} m/s ({abs(target_vel - avg_vel_left)/target_vel*100:.1f}%)")

            # Check if motors are moving
            if abs(avg_vel_left) < 0.01 and abs(avg_vel_right) < 0.01:
                print("  ❌ MOTORS NOT MOVING - Deadband too high or PWM too low!")
                return False
            elif abs(avg_vel_left - target_vel) > target_vel * 0.3:
                print("  ⚠️  Large velocity error - PID tuning needed")
                return False
            else:
                print("  ✅ Motors responding")
                return True
        else:
            print("  ❌ No odometry data received!")
            return False

    def optimize_deadband(self):
        """Find minimum deadband that allows motor movement"""
        print("\n" + "="*70)
        print("🔧 PHASE 1: DEADBAND OPTIMIZATION")
        print("="*70)
        print("Finding minimum deadband for reliable motor start...")

        # Test range of deadband values
        deadband_values = [30, 35, 40, 45, 50, 55, 60, 65, 70]
        test_vel = 0.20  # Test at moderate speed

        for db in deadband_values:
            print(f"\n🔍 Testing deadband: {db}")
            self.set_deadband(db, db)
            time.sleep(0.5)

            success = self.test_velocity(test_vel, duration=2.0)

            if success:
                print(f"✅ Deadband {db} works!")
                return db

        print("❌ No suitable deadband found!")
        return 50  # Default fallback

    def optimize_pid(self, deadband):
        """Tune PID for best performance"""
        print("\n" + "="*70)
        print("🔧 PHASE 2: PID OPTIMIZATION")
        print("="*70)

        # Set the optimized deadband
        self.set_deadband(deadband, deadband)

        # Test different PID configurations
        # Start with more aggressive settings for low-speed response
        pid_configs = [
            (15.0, 8.0, 0.2),   # Current
            (20.0, 10.0, 0.3),  # More aggressive
            (25.0, 12.0, 0.4),  # Even more aggressive
            (30.0, 15.0, 0.5),  # Very aggressive
        ]

        best_pid = pid_configs[0]
        best_error = float('inf')

        for kp, ki, kd in pid_configs:
            print(f"\n🔍 Testing PID: Kp={kp} Ki={ki} Kd={kd}")
            self.set_pid(kp, ki, kd)

            # Test at low speed (where problems occur)
            success = self.test_velocity(0.20, duration=2.5)

            if success and self.last_odom:
                # Calculate error
                # This is simplified - in reality we'd calculate from all samples
                target = 0.20
                dt = 0.05
                circumference = WHEEL_DIAMETER * 3.14159
                actual = (self.last_odom.delta_left / TICKS_PER_REV) * circumference / dt
                error = abs(target - actual)

                print(f"  Error: {error:.3f} m/s")

                if error < best_error:
                    best_error = error
                    best_pid = (kp, ki, kd)
                    print(f"  🌟 New best PID!")

        print(f"\n✅ Best PID: Kp={best_pid[0]} Ki={best_pid[1]} Kd={best_pid[2]}")
        return best_pid

    def run_full_test_suite(self):
        """Run complete optimization and testing"""
        print("\n🚀 Starting full optimization suite...")

        # Enable streaming
        self.enable_streaming(True, interval_ms=50)
        time.sleep(1)

        # Phase 1: Optimize deadband
        optimal_deadband = self.optimize_deadband()

        # Phase 2: Optimize PID with new deadband
        optimal_pid = self.optimize_pid(optimal_deadband)

        # Phase 3: Test full velocity range with optimized settings
        print("\n" + "="*70)
        print("🔧 PHASE 3: FULL VELOCITY RANGE TEST")
        print("="*70)

        kp, ki, kd = optimal_pid
        self.set_pid(kp, ki, kd)
        self.set_deadband(optimal_deadband, optimal_deadband)

        print(f"\nTesting with optimized settings:")
        print(f"  PID: Kp={kp} Ki={ki} Kd={kd}")
        print(f"  Deadband: {optimal_deadband}")

        for vel in TEST_VELOCITIES:
            self.test_velocity(vel, duration=2.0)
            time.sleep(1)

        print("\n" + "="*70)
        print("🎉 OPTIMIZATION COMPLETE!")
        print("="*70)
        print(f"\n📋 RECOMMENDED SETTINGS:")
        print(f"  PID: Kp={kp:.2f} Ki={ki:.2f} Kd={kd:.2f}")
        print(f"  Deadband Forward: {optimal_deadband:.2f}")
        print(f"  Deadband Reverse: {optimal_deadband:.2f}")
        print(f"\nUse MSG_SET_PID and MSG_SET_DEADBAND to apply these values,")
        print(f"then MSG_SAVE_CONFIG to persist to EEPROM.")

    def close(self):
        """Clean shutdown"""
        self.running = False
        print("\n🛑 Stopping motors...")
        self.stop_motors()

        if self.udp_sock:
            self.udp_sock.close()
        if self.arduino_serial:
            self.arduino_serial.close()

        print("✓ Shutdown complete")


def main():
    tester = RobotTester()

    try:
        tester.connect()
        tester.run_full_test_suite()

    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        tester.close()


if __name__ == '__main__':
    main()
