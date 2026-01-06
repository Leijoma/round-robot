#!/usr/bin/env python3
"""
RobotLink Integration Test
Tests motor control, encoder streaming, and lidar data reception
"""

import time
import sys
from robotlink import (
    RobotLink, MessageType,
    OdomPayload, LidarStatusPayload, LidarScanPayload
)

class RobotLinkTest:
    def __init__(self, host='192.168.4.1', port=5000):
        self.robot = RobotLink(host=host, port=port)
        self.test_results = {}

        # Track received data
        self.odom_received = []
        self.lidar_status_received = []
        self.lidar_scan_received = []

        # Register callbacks
        self.robot.register_callback(MessageType.MSG_ODOM, self.on_odom)
        self.robot.register_callback(MessageType.MSG_LIDAR_STATUS, self.on_lidar_status)
        self.robot.register_callback(MessageType.MSG_LIDAR_SCAN, self.on_lidar_scan)

    def on_odom(self, payload):
        """Callback for odometry messages"""
        odom = OdomPayload.unpack(payload)
        self.odom_received.append(odom)

    def on_lidar_status(self, payload):
        """Callback for lidar status messages"""
        status = LidarStatusPayload.unpack(payload)
        self.lidar_status_received.append(status)

    def on_lidar_scan(self, payload):
        """Callback for lidar scan messages"""
        scan = LidarScanPayload.unpack(payload)
        self.lidar_scan_received.append(scan)

    def wait_for_messages(self, duration=2.0):
        """Process messages for specified duration"""
        start = time.time()
        while time.time() - start < duration:
            self.robot.process_messages(timeout=0.1)
            time.sleep(0.05)

    def test_connection(self):
        """Test 1: Verify UDP connection to ESP32"""
        print("\n" + "="*60)
        print("TEST 1: Connection Test")
        print("="*60)

        # Send stop command to test connection
        print("Sending STOP command to test connection...")
        result = self.robot.stop()

        if result:
            print("✓ Command sent successfully")
            self.test_results['connection'] = True
        else:
            print("✗ Failed to send command")
            self.test_results['connection'] = False
            return False

        # Wait for any response
        self.wait_for_messages(1.0)

        stats = self.robot.get_stats()
        print(f"Stats: Sent={stats['frames_sent']}, Received={stats['frames_received']}, CRC Errors={stats['crc_errors']}")

        return True

    def test_encoder_stream(self):
        """Test 2: Enable odometry stream and verify encoder data"""
        print("\n" + "="*60)
        print("TEST 2: Encoder Stream Test")
        print("="*60)

        # Clear previous data
        self.odom_received.clear()

        # Enable odometry streaming
        print("Enabling odometry stream (100ms interval)...")
        self.robot.enable_stream(True, 100)

        # Wait and collect data
        print("Collecting odometry data for 3 seconds...")
        self.wait_for_messages(3.0)

        # Analyze results
        odom_count = len(self.odom_received)
        print(f"\nReceived {odom_count} odometry messages")

        if odom_count == 0:
            print("✗ No odometry data received!")
            self.test_results['encoder_stream'] = False
            return False

        # Check data rate (should be ~10 Hz for 100ms interval)
        expected_rate = 10  # Hz
        actual_rate = odom_count / 3.0
        print(f"Data rate: {actual_rate:.1f} Hz (expected ~{expected_rate} Hz)")

        # Display sample data
        if odom_count > 0:
            latest = self.odom_received[-1]
            print(f"\nLatest odometry:")
            print(f"  Encoder Left:  {latest.encoder_left} ticks")
            print(f"  Encoder Right: {latest.encoder_right} ticks")
            print(f"  Velocity Left:  {latest.vel_left:.3f} m/s")
            print(f"  Velocity Right: {latest.vel_right:.3f} m/s")
            print(f"  PWM Left:  {latest.pwm_left}")
            print(f"  PWM Right: {latest.pwm_right}")
            print(f"  Timestamp: {latest.timestamp} ms")

        # Check if data rate is reasonable (within 50% of expected)
        if actual_rate > expected_rate * 0.5:
            print("\n✓ Encoder stream working correctly")
            self.test_results['encoder_stream'] = True
            return True
        else:
            print(f"\n✗ Data rate too low ({actual_rate:.1f} Hz)")
            self.test_results['encoder_stream'] = False
            return False

    def test_motor_control(self):
        """Test 3: Send motor commands and verify encoder changes"""
        print("\n" + "="*60)
        print("TEST 3: Motor Control Test")
        print("="*60)

        # Clear previous data and ensure stopped
        self.odom_received.clear()
        print("Ensuring motors are stopped...")
        self.robot.stop()
        time.sleep(0.5)

        # Get baseline encoder values
        self.wait_for_messages(0.5)
        if len(self.odom_received) == 0:
            print("✗ No odometry data - cannot test motor control")
            self.test_results['motor_control'] = False
            return False

        baseline = self.odom_received[-1]
        baseline_left = baseline.encoder_left
        baseline_right = baseline.encoder_right
        print(f"Baseline encoders: L={baseline_left}, R={baseline_right}")

        # Send forward velocity command
        print("\nCommanding motors: 0.1 m/s forward for 2 seconds...")
        self.robot.set_velocity(0.1, 0.1)

        # Wait and monitor
        self.wait_for_messages(2.0)

        # Stop motors
        print("Stopping motors...")
        self.robot.stop()
        time.sleep(0.3)

        # Get final encoder values
        self.wait_for_messages(0.5)
        if len(self.odom_received) == 0:
            print("✗ No odometry data after motor command")
            self.test_results['motor_control'] = False
            return False

        final = self.odom_received[-1]
        final_left = final.encoder_left
        final_right = final.encoder_right

        # Calculate encoder change
        delta_left = final_left - baseline_left
        delta_right = final_right - baseline_right

        print(f"\nFinal encoders: L={final_left}, R={final_right}")
        print(f"Encoder change: L={delta_left}, R={delta_right}")

        # Check if motors moved
        if abs(delta_left) > 10 or abs(delta_right) > 10:
            print("✓ Motors responded to velocity commands")
            self.test_results['motor_control'] = True

            # Check velocity during motion
            if len(self.odom_received) > 5:
                mid_point = self.odom_received[len(self.odom_received)//2]
                print(f"\nMid-motion velocity: L={mid_point.vel_left:.3f}, R={mid_point.vel_right:.3f} m/s")
                print(f"Mid-motion PWM: L={mid_point.pwm_left}, R={mid_point.pwm_right}")

            return True
        else:
            print("✗ Motors did not move (encoder change too small)")
            print("   This could be normal if motors are not connected or disabled")
            self.test_results['motor_control'] = False
            return False

    def test_lidar_status(self):
        """Test 4: Enable lidar and verify status messages"""
        print("\n" + "="*60)
        print("TEST 4: Lidar Status Test")
        print("="*60)

        # Clear previous data
        self.lidar_status_received.clear()

        # Enable lidar
        print("Enabling lidar motor...")
        self.robot.lidar_enable(True)
        self.robot.lidar_set_rpm(220)

        # Wait for status messages
        print("Waiting for lidar status messages (3 seconds)...")
        self.wait_for_messages(3.0)

        status_count = len(self.lidar_status_received)
        print(f"\nReceived {status_count} lidar status messages")

        if status_count == 0:
            print("✗ No lidar status data received!")
            self.test_results['lidar_status'] = False
            return False

        # Display latest status
        latest = self.lidar_status_received[-1]
        print(f"\nLidar Status:")
        print(f"  Motor Running: {'YES' if latest.motor_running else 'NO'}")
        print(f"  Current RPM: {latest.current_rpm}")
        print(f"  Target RPM: {latest.target_rpm}")
        print(f"  RPM Control: {'ENABLED' if latest.rpm_control_enabled else 'DISABLED'}")
        print(f"  Motor Speed (PWM): {latest.motor_speed}")
        print(f"  Packets Received: {latest.packets_received}")
        print(f"  Packets Invalid: {latest.packets_invalid}")
        print(f"  Scans Complete: {latest.scans_complete}")

        # Check if lidar is actually running
        if latest.motor_running and latest.current_rpm > 0:
            print("\n✓ Lidar motor is running")
            self.test_results['lidar_status'] = True
            return True
        else:
            print("\n✗ Lidar motor not running or RPM is 0")
            print("   Check lidar power and serial connections")
            self.test_results['lidar_status'] = False
            return False

    def test_lidar_scan_data(self):
        """Test 5: Verify lidar scan data reception"""
        print("\n" + "="*60)
        print("TEST 5: Lidar Scan Data Test")
        print("="*60)

        # Clear previous data
        self.lidar_scan_received.clear()

        # Lidar should already be running from previous test
        print("Collecting lidar scan data for 5 seconds...")
        self.wait_for_messages(5.0)

        scan_count = len(self.lidar_scan_received)
        print(f"\nReceived {scan_count} lidar scans")

        if scan_count == 0:
            print("✗ No lidar scan data received!")
            print("   Scan messages may not be enabled in firmware")
            self.test_results['lidar_scan'] = False
            return False

        # Calculate scan rate
        scan_rate = scan_count / 5.0
        print(f"Scan rate: {scan_rate:.1f} Hz")

        # Analyze latest scan
        if scan_count > 0:
            latest = self.lidar_scan_received[-1]
            valid_readings = [r for r in latest.readings if not r.invalid]

            print(f"\nLatest Scan:")
            print(f"  Timestamp: {latest.timestamp} ms")
            print(f"  RPM: {latest.rpm}")
            print(f"  Start Angle: {latest.start_angle}°")
            print(f"  Total Readings: {len(latest.readings)}")
            print(f"  Valid Readings: {len(valid_readings)}")

            if len(valid_readings) > 0:
                distances = [r.distance_mm for r in valid_readings]
                avg_dist = sum(distances) / len(distances)
                min_dist = min(distances)
                max_dist = max(distances)

                print(f"  Distance Range: {min_dist}-{max_dist} mm")
                print(f"  Average Distance: {avg_dist:.0f} mm")

                # Show some sample readings
                print(f"\n  Sample readings (first 5 valid):")
                for i, reading in enumerate(valid_readings[:5]):
                    print(f"    [{i}] {reading.distance_mm}mm, strength={reading.signal_strength}")

        # Check if we got reasonable scan data
        if scan_count > 0 and len(valid_readings) > 0:
            print("\n✓ Lidar scan data received successfully")
            self.test_results['lidar_scan'] = True
            return True
        else:
            print("\n✗ No valid lidar readings")
            self.test_results['lidar_scan'] = False
            return False

    def cleanup(self):
        """Stop motors and lidar, print summary"""
        print("\n" + "="*60)
        print("Cleaning up...")
        print("="*60)

        self.robot.stop()
        self.robot.lidar_enable(False)
        time.sleep(0.5)

        # Final statistics
        stats = self.robot.get_stats()
        print(f"\nFinal Statistics:")
        print(f"  Frames Sent: {stats['frames_sent']}")
        print(f"  Frames Received: {stats['frames_received']}")
        print(f"  CRC Errors: {stats['crc_errors']}")

        if stats['frames_received'] > 0:
            error_rate = stats['crc_errors'] / (stats['frames_received'] + stats['crc_errors'])
            print(f"  Error Rate: {error_rate*100:.2f}%")

        print(f"\nData Collected:")
        print(f"  Odometry Messages: {len(self.odom_received)}")
        print(f"  Lidar Status Messages: {len(self.lidar_status_received)}")
        print(f"  Lidar Scans: {len(self.lidar_scan_received)}")

        self.robot.close()

    def print_summary(self):
        """Print test results summary"""
        print("\n" + "="*60)
        print("TEST SUMMARY")
        print("="*60)

        tests = [
            ('Connection', 'connection'),
            ('Encoder Stream', 'encoder_stream'),
            ('Motor Control', 'motor_control'),
            ('Lidar Status', 'lidar_status'),
            ('Lidar Scan Data', 'lidar_scan')
        ]

        passed = 0
        total = len(tests)

        for name, key in tests:
            if key in self.test_results:
                status = "✓ PASS" if self.test_results[key] else "✗ FAIL"
                if self.test_results[key]:
                    passed += 1
            else:
                status = "- SKIP"
            print(f"  {name:20s} {status}")

        print(f"\nResult: {passed}/{total} tests passed")
        print("="*60 + "\n")

        return passed == total


def main():
    # Parse command line arguments
    host = '192.168.4.1'
    port = 5000

    if len(sys.argv) > 1:
        host = sys.argv[1]
    if len(sys.argv) > 2:
        port = int(sys.argv[2])

    print("="*60)
    print("RobotLink Integration Test Suite")
    print("="*60)
    print(f"Target: {host}:{port}")
    print("="*60)

    # Create test instance
    test = RobotLinkTest(host=host, port=port)

    try:
        # Run all tests
        test.test_connection()
        test.test_encoder_stream()
        test.test_motor_control()
        test.test_lidar_status()
        test.test_lidar_scan_data()

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user!")

    except Exception as e:
        print(f"\n\nTest error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        test.cleanup()
        all_passed = test.print_summary()

        sys.exit(0 if all_passed else 1)


if __name__ == '__main__':
    main()
