#!/usr/bin/env python3
"""
Test script to verify motor commands reach the Arduino
Sends direct velocity commands and monitors for Arduino debug output
"""

import sys
import time
sys.path.append('robot-ui/server')

from robotlink import RobotLink

ESP32_IP = "192.168.68.74"
ESP32_PORT = 5000

def test_motor_commands():
    print("=" * 70)
    print("Motor Command Test")
    print("=" * 70)
    print(f"\nConnecting to ESP32 at {ESP32_IP}:{ESP32_PORT}...")

    robot = RobotLink(ESP32_IP, ESP32_PORT)

    if not robot.connect(timeout=3.0):
        print("ERROR: Could not connect to ESP32")
        print("Check:")
        print("  1. ESP32 is powered and connected to WiFi")
        print("  2. IP address is correct")
        print("  3. No firewall blocking UDP port 5000")
        return False

    print("✓ Connected to ESP32")
    print("\nNOTE: Watch the Arduino Serial Monitor (115200 baud) for debug output")
    print("      You should see 'SET_VEL: L=0.10 R=0.10' messages\n")

    tests = [
        ("Forward 0.1 m/s", 0.1, 0.1),
        ("Stop", 0.0, 0.0),
        ("Forward 0.15 m/s", 0.15, 0.15),
        ("Stop", 0.0, 0.0),
        ("Rotate (left 0.1, right -0.1)", 0.1, -0.1),
        ("Stop", 0.0, 0.0),
    ]

    for i, (description, left, right) in enumerate(tests, 1):
        print(f"\nTest {i}/{len(tests)}: {description}")
        print(f"  Sending: MSG_SET_VEL left={left} m/s, right={right} m/s")

        success = robot.set_velocity(left, right)
        if success:
            print(f"  ✓ Command sent to ESP32")
        else:
            print(f"  ✗ Failed to send command")

        time.sleep(2.0)

    print("\n" + "=" * 70)
    print("Test Complete")
    print("=" * 70)
    print("\nDid you see debug output on Arduino Serial Monitor?")
    print("  YES: Commands are reaching Arduino - check PID/motor wiring")
    print("  NO:  Commands not reaching Arduino - check ESP32 Serial1 connection")

    robot.disconnect()
    return True

if __name__ == "__main__":
    try:
        test_motor_commands()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
