#!/usr/bin/env python3
"""
Robot Control Example using RobotLink Protocol
Demonstrates motor control and lidar monitoring via UDP
"""

import time
import sys
from robotlink import RobotLink, MessageType, OdomPayload, LidarStatusPayload, LidarScanPayload

def main():
    # Create RobotLink instance
    # Default: 192.168.4.1:5000 (ESP32 AP mode)
    robot = RobotLink(host='192.168.4.1', port=5000)

    print("=" * 60)
    print("RobotLink Control Example")
    print("=" * 60)
    print(f"Connecting to: {robot.host}:{robot.port}\n")

    # Track latest data
    latest_odom = None
    latest_lidar_status = None
    scan_count = 0

    # Register callbacks for incoming messages
    def on_odom(payload):
        nonlocal latest_odom
        latest_odom = OdomPayload.unpack(payload)

    def on_lidar_status(payload):
        nonlocal latest_lidar_status
        latest_lidar_status = LidarStatusPayload.unpack(payload)

    def on_lidar_scan(payload):
        nonlocal scan_count
        scan = LidarScanPayload.unpack(payload)
        scan_count += 1

        # Print occasional scan summary
        if scan_count % 10 == 0:
            valid = sum(1 for r in scan.readings if not r.invalid)
            avg_dist = sum(r.distance_mm for r in scan.readings if not r.invalid) / max(valid, 1)
            print(f"  Scan #{scan_count}: {valid}/{len(scan.readings)} valid, "
                  f"avg={avg_dist:.0f}mm, angle={scan.start_angle}°")

    robot.register_callback(MessageType.MSG_ODOM, on_odom)
    robot.register_callback(MessageType.MSG_LIDAR_STATUS, on_lidar_status)
    robot.register_callback(MessageType.MSG_LIDAR_SCAN, on_lidar_scan)

    try:
        # 1. Enable lidar
        print("1. Starting lidar motor...")
        robot.lidar_enable(True)
        robot.lidar_set_rpm(220)
        time.sleep(1)

        # 2. Enable odometry streaming
        print("2. Enabling odometry stream (100ms interval)...")
        robot.enable_stream(True, 100)
        time.sleep(0.5)

        # 3. Drive forward slowly
        print("3. Driving forward at 0.1 m/s for 3 seconds...")
        robot.set_velocity(0.1, 0.1)

        # Monitor for 3 seconds
        for i in range(30):
            robot.process_messages(timeout=0.1)

            # Print status every second
            if i % 10 == 0 and latest_odom and latest_lidar_status:
                print(f"\n  Motors: L={latest_odom.vel_left:.3f} m/s, R={latest_odom.vel_right:.3f} m/s, "
                      f"PWM=[{latest_odom.pwm_left}, {latest_odom.pwm_right}]")
                print(f"  Lidar: {latest_lidar_status.current_rpm} RPM "
                      f"({'ON' if latest_lidar_status.motor_running else 'OFF'}), "
                      f"Packets={latest_lidar_status.packets_received}")

        # 4. Rotate in place
        print("\n4. Rotating in place (0.05 m/s) for 3 seconds...")
        robot.set_velocity(-0.05, 0.05)

        for i in range(30):
            robot.process_messages(timeout=0.1)

        # 5. Stop
        print("\n5. Stopping motors...")
        robot.stop()
        time.sleep(0.5)

        # 6. Continue monitoring lidar
        print("\n6. Monitoring lidar for 5 seconds...")
        for i in range(50):
            robot.process_messages(timeout=0.1)

        # 7. Stop lidar
        print("\n7. Stopping lidar motor...")
        robot.lidar_enable(False)
        time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user!")
        robot.stop()
        robot.lidar_enable(False)

    except Exception as e:
        print(f"\n\nError: {e}")
        robot.stop()

    finally:
        # Print final statistics
        stats = robot.get_stats()
        print("\n" + "=" * 60)
        print("Session Statistics")
        print("=" * 60)
        print(f"Frames sent:     {stats['frames_sent']}")
        print(f"Frames received: {stats['frames_received']}")
        print(f"CRC errors:      {stats['crc_errors']}")
        print(f"Lidar scans:     {scan_count}")

        if latest_odom:
            print(f"\nFinal odometry:")
            print(f"  Encoder L: {latest_odom.encoder_left} ticks")
            print(f"  Encoder R: {latest_odom.encoder_right} ticks")
            print(f"  Uptime: {latest_odom.timestamp/1000:.1f}s")

        robot.close()
        print("\nConnection closed.")

if __name__ == '__main__':
    main()
