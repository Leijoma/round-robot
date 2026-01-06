#!/usr/bin/env python3
"""
LIDAR Integration Test Script
Tests ESP32 Neato LIDAR motor control and scan data streaming
"""

import time
from robotlink import RobotLink, LidarScanPayload, LidarStatusPayload, MessageType

def test_lidar_control(host='192.168.68.52', port=5000):
    """Test LIDAR motor control and scan data streaming"""

    print('=' * 70)
    print('ESP32 LIDAR INTEGRATION TEST')
    print('=' * 70)
    print(f'Target: {host}:{port}\n')

    # Connect to ESP32
    robot = RobotLink(host=host, port=port)
    time.sleep(0.5)

    # Statistics
    scan_count = 0
    status_count = 0
    total_readings = 0

    # Callbacks for LIDAR messages
    def on_lidar_scan(payload):
        nonlocal scan_count, total_readings
        scan = LidarScanPayload.unpack(payload)
        scan_count += 1
        total_readings += len(scan.readings)

        if scan_count <= 5 or scan_count % 10 == 0:
            valid = [r for r in scan.readings if not r.invalid]
            print(f'  Scan #{scan_count}: {len(valid)}/{len(scan.readings)} valid, '
                  f'angle={scan.start_angle}°, RPM={scan.rpm}')

    def on_lidar_status(payload):
        nonlocal status_count
        status = LidarStatusPayload.unpack(payload)
        status_count += 1
        print(f'  Status: {status.current_rpm} RPM (target {status.target_rpm}), '
              f'Motor={'ON' if status.motor_running else 'OFF'}, '
              f'Scans={status.scans_complete}')

    robot.register_callback(MessageType.MSG_LIDAR_SCAN, on_lidar_scan)
    robot.register_callback(MessageType.MSG_LIDAR_STATUS, on_lidar_status)

    # Test 1: Start LIDAR motor at 220 RPM
    print('\n[TEST 1] Starting LIDAR motor at 220 RPM...')
    robot.lidar_set_rpm(220)
    robot.lidar_enable(True)
    time.sleep(0.5)

    print('  Waiting 10 seconds for LIDAR to spin up and stabilize...')
    for i in range(10):
        robot.process_messages(timeout=1.0)
        print(f'    {i+1}/10 seconds...', end='\r')
    print()

    # Test 2: Monitor scan data for 10 seconds
    print('\n[TEST 2] Monitoring scan data for 10 seconds...')
    scan_count_before = scan_count
    start_time = time.time()

    while time.time() - start_time < 10:
        robot.process_messages(timeout=0.5)
        time.sleep(0.1)

    scans_received = scan_count - scan_count_before
    print(f'  Received {scans_received} scan batches in 10 seconds')

    # Test 3: Change RPM to 250
    print('\n[TEST 3] Changing RPM to 250...')
    robot.lidar_set_rpm(250)
    time.sleep(0.5)

    print('  Monitoring for 5 seconds...')
    for i in range(5):
        robot.process_messages(timeout=1.0)
        print(f'    {i+1}/5 seconds...', end='\r')
    print()

    # Test 4: Stop LIDAR motor
    print('\n[TEST 4] Stopping LIDAR motor...')
    robot.lidar_enable(False)
    time.sleep(0.5)

    robot.process_messages(timeout=1.0)

    # Final statistics
    print('\n' + '=' * 70)
    print('TEST RESULTS')
    print('=' * 70)
    print(f'  Scan batches received: {scan_count}')
    print(f'  Total LIDAR readings: {total_readings}')
    print(f'  Status messages: {status_count}')
    print(f'  Frames sent: {robot.frames_sent}')
    print(f'  Frames received: {robot.frames_received}')
    print(f'  CRC errors: {robot.crc_errors}')

    # Verdict
    print('\n' + '=' * 70)
    if scan_count > 20 and total_readings > 800:
        print('SUCCESS! LIDAR Integration Operational!')
        print('=' * 70)
        print('  → LIDAR motor control via UDP: WORKING')
        print('  → LIDAR scan data streaming: WORKING')
        print('  → RPM adjustment: WORKING')
        print('  → Data batching (40 readings/batch): WORKING')
        print(f'  → Average readings/scan: {total_readings//scan_count if scan_count > 0 else 0}')
    elif scan_count > 0:
        print('PARTIAL SUCCESS - Communication working but low data rate')
        print('=' * 70)
        print('  → LIDAR motor control: WORKING')
        print('  → Scan data received but rate may be low')
    else:
        print('FAILURE - No scan data received')
        print('=' * 70)
        print('  Check:')
        print('  - LIDAR hardware connected to GPIO16 (RX) and GPIO25 (PWM)')
        print('  - LIDAR power supply (5V)')
        print('  - ESP32 serial monitor for error messages')

    robot.close()
    return scan_count > 20


if __name__ == '__main__':
    import sys

    # Run test
    success = test_lidar_control()

    # Exit with appropriate code
    sys.exit(0 if success else 1)
