#!/usr/bin/env python3
"""
Complete System Verification
Verifies:
1. Motor control works
2. Odometry data streams to host
3. Lidar scans stream to host
"""

import sys
from robotlink import RobotLink, MessageType, OdomPayload, LidarStatusPayload, LidarScanPayload
import time

def main():
    print("="*70)
    print("COMPLETE SYSTEM VERIFICATION")
    print("="*70)

    robot = RobotLink(host='192.168.68.52', port=5000)

    # Track received data
    odom_count = 0
    lidar_status_count = 0
    lidar_scan_count = 0
    latest_odom = None
    latest_lidar_status = None

    def on_odom(payload):
        nonlocal odom_count, latest_odom
        odom_count += 1
        latest_odom = OdomPayload.unpack(payload)
        if odom_count <= 3:  # Print first few
            print(f"  [ODOM #{odom_count}] Encoders: L={latest_odom.encoder_left}, R={latest_odom.encoder_right}, "
                  f"Vel: L={latest_odom.vel_left:.3f}, R={latest_odom.vel_right:.3f} m/s, "
                  f"PWM: L={latest_odom.pwm_left}, R={latest_odom.pwm_right}")

    def on_lidar_status(payload):
        nonlocal lidar_status_count, latest_lidar_status
        lidar_status_count += 1
        latest_lidar_status = LidarStatusPayload.unpack(payload)
        if lidar_status_count <= 3:  # Print first few
            print(f"  [LIDAR STATUS #{lidar_status_count}] RPM: {latest_lidar_status.current_rpm}/{latest_lidar_status.target_rpm}, "
                  f"Motor: {'ON' if latest_lidar_status.motor_running else 'OFF'}, "
                  f"Packets: {latest_lidar_status.packets_received}")

    def on_lidar_scan(payload):
        nonlocal lidar_scan_count
        lidar_scan_count += 1
        scan = LidarScanPayload.unpack(payload)
        valid = sum(1 for r in scan.readings if not r.invalid)
        if lidar_scan_count <= 3:  # Print first few
            print(f"  [LIDAR SCAN #{lidar_scan_count}] {valid}/{len(scan.readings)} valid readings, "
                  f"Start angle: {scan.start_angle}°, RPM: {scan.rpm}")

    robot.register_callback(MessageType.MSG_ODOM, on_odom)
    robot.register_callback(MessageType.MSG_LIDAR_STATUS, on_lidar_status)
    robot.register_callback(MessageType.MSG_LIDAR_SCAN, on_lidar_scan)

    # TEST 1: Check if Arduino is connected (odometry available)
    print("\n" + "="*70)
    print("TEST 1: Check Arduino Connection & Enable Odometry Stream")
    print("="*70)

    robot.enable_stream(True, 100)  # Enable 100ms odometry
    print("Enabled odometry streaming (100ms interval)")
    print("Waiting 3 seconds for odometry data...")

    start = time.time()
    while time.time() - start < 3:
        robot.process_messages(timeout=0.1)
        time.sleep(0.05)

    if odom_count > 0:
        print(f"\n✓ Arduino CONNECTED - Received {odom_count} odometry messages")
        print(f"  Data rate: {odom_count/3:.1f} Hz (expected ~10 Hz)")
    else:
        print("\n✗ Arduino NOT CONNECTED - No odometry data received")
        print("  (This is expected if Arduino is not physically connected)")

    # TEST 2: Motor Control
    print("\n" + "="*70)
    print("TEST 2: Motor Control")
    print("="*70)

    if odom_count > 0 and latest_odom:
        baseline_left = latest_odom.encoder_left
        baseline_right = latest_odom.encoder_right

        print(f"Baseline encoders: L={baseline_left}, R={baseline_right}")
        print("Sending velocity command: 0.1 m/s forward for 2 seconds...")

        robot.set_velocity(0.1, 0.1)

        start = time.time()
        while time.time() - start < 2:
            robot.process_messages(timeout=0.1)
            time.sleep(0.05)

        robot.stop()
        print("Motors stopped")

        time.sleep(0.5)
        robot.process_messages(timeout=0.5)

        if latest_odom:
            delta_left = latest_odom.encoder_left - baseline_left
            delta_right = latest_odom.encoder_right - baseline_right

            print(f"Final encoders: L={latest_odom.encoder_left}, R={latest_odom.encoder_right}")
            print(f"Encoder change: L={delta_left}, R={delta_right}")

            if abs(delta_left) > 10 or abs(delta_right) > 10:
                print("\n✓ MOTOR CONTROL WORKING - Encoders changed significantly")
            else:
                print("\n⚠ Motors may not be working - Small encoder change")
                print("  (Could be normal if robot is blocked or wheels are lifted)")
    else:
        print("✗ Cannot test motor control - No Arduino connection")

    # TEST 3: Lidar Scan Streaming
    print("\n" + "="*70)
    print("TEST 3: Lidar Scan Streaming")
    print("="*70)

    print("Enabling lidar motor...")
    robot.lidar_enable(True)
    robot.lidar_set_rpm(220)

    print("Waiting 8 seconds for lidar to spin up and send scans...")
    print("(Lidar needs time to reach target RPM and generate scans)\n")

    start = time.time()
    while time.time() - start < 8:
        robot.process_messages(timeout=0.1)
        time.sleep(0.05)

    if lidar_scan_count > 0:
        print(f"\n✓ LIDAR SCANS WORKING - Received {lidar_scan_count} scans")
        print(f"  Scan rate: {lidar_scan_count/8:.1f} Hz")
    else:
        print(f"\n✗ No lidar scans received")
        print(f"  But received {lidar_status_count} status messages")
        if latest_lidar_status:
            print(f"  Lidar RPM: {latest_lidar_status.current_rpm}/{latest_lidar_status.target_rpm}")
            print(f"  Packets: {latest_lidar_status.packets_received}")
            print(f"  Scans: {latest_lidar_status.scans_complete}")
        print("  Note: Scan streaming may not be enabled in firmware")

    # Cleanup
    print("\n" + "="*70)
    print("Cleanup...")
    print("="*70)
    robot.stop()
    robot.lidar_enable(False)
    time.sleep(0.5)

    # Final Summary
    stats = robot.get_stats()
    print("\n" + "="*70)
    print("FINAL RESULTS")
    print("="*70)
    print(f"Commands sent:        {stats['frames_sent']}")
    print(f"Responses received:   {stats['frames_received']}")
    print(f"CRC errors:           {stats['crc_errors']}")
    print(f"\nOdometry messages:    {odom_count}")
    print(f"Lidar status msgs:    {lidar_status_count}")
    print(f"Lidar scans:          {lidar_scan_count}")

    print("\n" + "="*70)
    print("VERIFICATION SUMMARY")
    print("="*70)

    tests_passed = 0
    tests_total = 3

    # Test 1: Odometry
    if odom_count > 0:
        print("✓ Odometry streaming: WORKING")
        tests_passed += 1
    else:
        print("✗ Odometry streaming: NOT WORKING (Arduino may not be connected)")

    # Test 2: Motor Control
    if odom_count > 0 and latest_odom and (abs(latest_odom.encoder_left - baseline_left) > 10 or
                                            abs(latest_odom.encoder_right - baseline_right) > 10):
        print("✓ Motor control: WORKING")
        tests_passed += 1
    elif odom_count == 0:
        print("- Motor control: CANNOT TEST (No Arduino)")
        tests_total -= 1
    else:
        print("⚠ Motor control: UNCLEAR (Encoders didn't change much)")

    # Test 3: Lidar Scans
    if lidar_scan_count > 0:
        print("✓ Lidar scan streaming: WORKING")
        tests_passed += 1
    elif lidar_status_count > 0:
        print("⚠ Lidar scan streaming: NOT ENABLED (but lidar is working)")
    else:
        print("✗ Lidar scan streaming: NOT WORKING")

    print(f"\nResult: {tests_passed}/{tests_total} tests passed")
    print("="*70)

    robot.close()
    return tests_passed == tests_total

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
