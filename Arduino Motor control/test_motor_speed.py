#!/usr/bin/env python3
"""
Test motor speed and verify encoder response
Runs motors at 0.25 m/s and monitors encoder deltas
"""

import sys
import time
from test_robot import RobotLink

def test_motor_speed(port='/dev/cu.usbmodem212201'):
    """Run motors at 0.25 m/s and verify encoder response"""

    print("="*60)
    print("Motor Speed Test - 0.25 m/s")
    print("="*60)
    print("\n⚠ WARNING: Ensure robot wheels can spin freely!")
    print("Press Enter to continue or Ctrl+C to abort...")

    try:
        input()
    except KeyboardInterrupt:
        print("\nTest aborted")
        return

    # Connect to robot
    print(f"\nConnecting to {port}...")
    robot = RobotLink(port, 115200)

    # Test connection
    if not robot.ping():
        print("✗ Failed to connect to Arduino!")
        return

    print("✓ Connected successfully\n")

    # Zero encoders
    print("Zeroing encoders...")
    robot.zero_encoders()
    time.sleep(0.2)

    # Enable odometry streaming at 50ms (20 Hz)
    print("Enabling odometry streaming (20 Hz)...")
    robot.enable_stream(True, 50)
    time.sleep(0.3)

    # Clear any buffered messages
    for _ in range(5):
        robot.read_odometry(timeout=0.1)

    print("\n" + "="*60)
    print("Starting motors at 0.25 m/s for 5 seconds...")
    print("="*60)
    print("\nExpected behavior:")
    print("  - Encoders should show positive deltas if motors are running")
    print("  - At 0.25 m/s with 82mm wheels and 360 ticks/rev:")
    print("    ~173 ticks/second per motor")
    print("    ~8-9 ticks per 50ms update")
    print("\nActual output:\n")

    # Start motors
    robot.set_velocity(0.25, 0.25)

    start_time = time.time()
    total_left = 0
    total_right = 0
    message_count = 0
    max_dL = 0
    max_dR = 0

    # Monitor for 5 seconds
    while time.time() - start_time < 5.0:
        odom = robot.read_odometry(timeout=0.2)

        if odom:
            message_count += 1
            dL = odom['dL_ticks']
            dR = odom['dR_ticks']
            total_left += dL
            total_right += dR
            max_dL = max(max_dL, abs(dL))
            max_dR = max(max_dR, abs(dR))

            # Calculate instantaneous velocity estimate
            dt = 0.05  # 50ms
            wheel_diameter = 0.082  # meters
            ticks_per_rev = 360.0
            meters_per_tick = (3.14159 * wheel_diameter) / ticks_per_rev

            vel_left = (dL * meters_per_tick) / dt
            vel_right = (dR * meters_per_tick) / dt

            # Print every message with color coding
            if abs(dL) > 0 or abs(dR) > 0:
                status = "✓ MOVING"
            else:
                status = "⚠ STOPPED"

            print(f"[{time.time() - start_time:5.2f}s] {status} | "
                  f"dL={dL:4d} ticks, dR={dR:4d} ticks | "
                  f"vel_L={vel_left:6.3f} m/s, vel_R={vel_right:6.3f} m/s | "
                  f"pos: x={odom['x_mm']:6d}mm, y={odom['y_mm']:6d}mm")

    # Stop motors
    print("\nStopping motors...")
    robot.stop()
    time.sleep(0.5)

    # Disable streaming
    robot.enable_stream(False)

    # Analysis
    print("\n" + "="*60)
    print("TEST RESULTS")
    print("="*60)

    print(f"\nMessages received: {message_count} (expected ~100 at 20 Hz)")
    print(f"Total encoder ticks over 5 seconds:")
    print(f"  Left motor:  {total_left:6d} ticks")
    print(f"  Right motor: {total_right:6d} ticks")
    print(f"\nMax delta per update:")
    print(f"  Left motor:  {max_dL:3d} ticks/50ms")
    print(f"  Right motor: {max_dR:3d} ticks/50ms")

    # Calculate average velocity
    if message_count > 0:
        avg_left_per_update = total_left / message_count
        avg_right_per_update = total_right / message_count

        wheel_diameter = 0.082  # meters
        ticks_per_rev = 360.0
        meters_per_tick = (3.14159 * wheel_diameter) / ticks_per_rev
        dt = 0.05  # 50ms

        avg_vel_left = (avg_left_per_update * meters_per_tick) / dt
        avg_vel_right = (avg_right_per_update * meters_per_tick) / dt

        print(f"\nAverage velocity over test:")
        print(f"  Left motor:  {avg_vel_left:6.3f} m/s (target: 0.250 m/s)")
        print(f"  Right motor: {avg_vel_right:6.3f} m/s (target: 0.250 m/s)")

        # Error analysis
        error_left = abs(avg_vel_left - 0.25)
        error_right = abs(avg_vel_right - 0.25)

        print(f"\nVelocity error:")
        print(f"  Left motor:  {error_left:6.3f} m/s ({error_left/0.25*100:5.1f}%)")
        print(f"  Right motor: {error_right:6.3f} m/s ({error_right/0.25*100:5.1f}%)")

    # Verdict
    print("\n" + "="*60)
    print("DIAGNOSIS")
    print("="*60)

    if total_left == 0 and total_right == 0:
        print("\n✗ MOTORS NOT RUNNING")
        print("\nPossible causes:")
        print("  1. Motor power supply not connected to Monster Moto Shield")
        print("  2. Motors not connected or faulty wiring")
        print("  3. Deadband too low (motors can't overcome friction)")
        print("  4. H-bridge fault or protection triggered")
        print("\nTroubleshooting steps:")
        print("  - Check power supply voltage (should be 7-30V)")
        print("  - Verify motor wire connections to M1 and M2 terminals")
        print("  - Try increasing deadband: robot.set_deadband(50, 50, 50, 50)")
        print("  - Check Monster Moto Shield LED indicators")

    elif total_left > 0 and total_right > 0:
        print("\n✓ MOTORS RUNNING CORRECTLY")

        if abs(avg_vel_left - 0.25) < 0.05 and abs(avg_vel_right - 0.25) < 0.05:
            print("✓ PID controller tracking target velocity well")
        else:
            print("⚠ PID controller needs tuning for better tracking")
            print(f"  Consider adjusting Kp (current: 10.0)")

        if abs(total_left - total_right) > total_left * 0.2:  # More than 20% difference
            print(f"\n⚠ Motors not balanced:")
            print(f"  Left:  {total_left} ticks")
            print(f"  Right: {total_right} ticks")
            print("  This is normal if motors/wheels have different characteristics")
            print("  The PID controller should compensate over time")
        else:
            print("\n✓ Motors well balanced")

    elif total_left > 0 or total_right > 0:
        print("\n⚠ ONLY ONE MOTOR RUNNING")
        if total_left == 0:
            print("  Left motor (M1) not moving")
        else:
            print("  Right motor (M2) not moving")
        print("\nCheck:")
        print("  - Motor connections")
        print("  - Motor wiring polarity")
        print("  - Individual motor deadband settings")

    else:  # Negative values
        print("\n⚠ MOTORS RUNNING BACKWARD")
        print("  Check motor wiring polarity or encoder connections")

    print("\n" + "="*60)

    robot.close()


if __name__ == "__main__":
    # Auto-detect Arduino port
    import serial.tools.list_ports

    ports = list(serial.tools.list_ports.comports())
    arduino_ports = [p for p in ports if 'usbmodem' in p.device or 'Arduino' in p.description]

    if arduino_ports:
        port = arduino_ports[0].device
        print(f"Using Arduino on: {port}\n")
        test_motor_speed(port)
    else:
        print("No Arduino found! Please specify port manually.")
        if len(sys.argv) > 1:
            test_motor_speed(sys.argv[1])
