#!/usr/bin/env python3
"""
Simple motor control test for ESP32 Robot Controller
Tests if motors respond to velocity commands
"""

import time
from robotlink import RobotLink, MessageType, OdomPayload

def main():
    robot = RobotLink(host='192.168.68.52', port=5000)

    odom_data = []

    def on_odom(payload):
        odom = OdomPayload.unpack(payload)
        odom_data.append({
            'time': time.time(),
            'left_enc': odom.encoder_left,
            'right_enc': odom.encoder_right,
            'left_vel': odom.vel_left,
            'right_vel': odom.vel_right,
            'left_pwm': odom.pwm_left,
            'right_pwm': odom.pwm_right
        })

    robot.register_callback(MessageType.MSG_ODOM, on_odom)

    print("=" * 60)
    print("MOTOR CONTROL TEST")
    print("=" * 60)

    # Enable odometry
    print("\n1. Enabling odometry stream (50ms)...")
    robot.enable_stream(True, 50)
    time.sleep(1)

    # Collect baseline
    print("2. Collecting baseline data for 1 second...")
    odom_data.clear()
    start = time.time()
    while time.time() - start < 1:
        robot.process_messages(timeout=0.05)
        time.sleep(0.01)

    if len(odom_data) == 0:
        print("\n✗ ERROR: No odometry data received!")
        print("  Check if Arduino is connected and powered.")
        robot.close()
        return

    baseline = odom_data[-1]
    print(f"   Baseline: L={baseline['left_enc']}, R={baseline['right_enc']}")
    print(f"   Data rate: {len(odom_data)} Hz")

    # Send velocity command
    print("\n3. Sending velocity command: 0.15 m/s forward...")
    print("   Testing for 4 seconds...")
    robot.set_velocity(0.15, 0.15)
    time.sleep(0.5)

    # Monitor while moving
    odom_data.clear()
    start = time.time()
    print("\n   Time | Left Enc | Right Enc | Left Vel | Right Vel | Left PWM | Right PWM")
    print("   " + "-" * 75)

    while time.time() - start < 4:
        robot.process_messages(timeout=0.05)
        if len(odom_data) > 0 and (len(odom_data) % 20 == 0):  # Print every second
            latest = odom_data[-1]
            elapsed = latest['time'] - start
            print(f"   {elapsed:4.1f}s | {latest['left_enc']:8d} | {latest['right_enc']:9d} | "
                  f"{latest['left_vel']:+8.3f} | {latest['right_vel']:+9.3f} | "
                  f"{latest['left_pwm']:8d} | {latest['right_pwm']:9d}")
        time.sleep(0.01)

    # Stop motors
    print("\n4. Stopping motors...")
    robot.stop()
    time.sleep(0.5)

    # Final reading
    robot.process_messages(timeout=0.1)
    if len(odom_data) > 0:
        final = odom_data[-1]

        left_change = final['left_enc'] - baseline['left_enc']
        right_change = final['right_enc'] - baseline['right_enc']

        print(f"\n   Final: L={final['left_enc']}, R={final['right_enc']}")
        print(f"   Change: L={left_change:+d}, R={right_change:+d}")

        # Evaluate results
        print("\n" + "=" * 60)
        print("RESULTS")
        print("=" * 60)

        if abs(left_change) > 100 or abs(right_change) > 100:
            print("✓ MOTORS ARE WORKING!")
            print(f"  Significant encoder movement detected:")
            print(f"  - Left wheel: {left_change:+d} counts")
            print(f"  - Right wheel: {right_change:+d} counts")
        elif abs(left_change) > 10 or abs(right_change) > 10:
            print("⚠ MOTORS MIGHT BE WORKING")
            print(f"  Small encoder movement detected:")
            print(f"  - Left wheel: {left_change:+d} counts")
            print(f"  - Right wheel: {right_change:+d} counts")
            print("  Possible issues:")
            print("  - Robot wheels are lifted off ground")
            print("  - Motors are mechanically blocked")
            print("  - Low battery voltage")
        else:
            print("✗ NO MOTOR MOVEMENT DETECTED")
            print(f"  Encoder change: L={left_change:+d}, R={right_change:+d}")
            print("  Possible issues:")
            print("  - Motors not connected to Arduino")
            print("  - Motor driver not powered")
            print("  - Arduino PID controller disabled")
            print("  - Encoder wires disconnected")

    stats = robot.get_stats()
    print(f"\nCommunication: {stats['frames_sent']} sent, {stats['frames_received']} received, "
          f"{stats['crc_errors']} CRC errors")

    robot.close()

if __name__ == '__main__':
    main()
