#!/usr/bin/env python3
"""
Comprehensive test of RobotLink binary protocol
Tests velocity control, streaming, configuration, and EEPROM
"""

import time
from robotlink import RobotLink

PORT = '/dev/cu.usbmodem212201'
BAUDRATE = 115200


def test_streaming_velocity():
    """Test continuous streaming with velocity control"""
    print("=" * 70)
    print("TEST: Streaming Odometry with Velocity Control")
    print("=" * 70)

    with RobotLink(PORT, baudrate=BAUDRATE) as robot:
        # Zero encoders
        robot.zero_encoders()
        time.sleep(0.1)

        # Enable streaming at 50ms (20 Hz)
        robot.enable_stream(True, 50)
        time.sleep(0.2)

        # Set velocity
        print("\nSetting velocity to 0.3 m/s...")
        robot.set_velocity(0.3, 0.3)

        # Collect 20 odometry messages (1 second at 20 Hz)
        print("Collecting odometry data...")
        odom_data = []
        start_time = time.time()

        for i in range(20):
            odom = robot.read_odom(timeout=0.2)
            if odom:
                odom_data.append(odom)
                if i % 5 == 0:  # Print every 5th message
                    print(f"  {i:2d}: vel_L={odom.vel_left:6.3f} vel_R={odom.vel_right:6.3f} "
                          f"pwm_L={odom.pwm_left:4d} pwm_R={odom.pwm_right:4d} "
                          f"enc_L={odom.encoder_left:5d} enc_R={odom.encoder_right:5d}")
            else:
                print(f"  {i:2d}: No data received")

        elapsed = time.time() - start_time

        # Stop
        robot.stop()
        robot.enable_stream(False, 0)

        # Analysis
        print(f"\nReceived {len(odom_data)}/20 messages in {elapsed:.2f}s")
        print(f"Update rate: {len(odom_data)/elapsed:.1f} Hz (target: 20 Hz)")

        if odom_data:
            final = odom_data[-1]
            avg_vel_left = sum(d.vel_left for d in odom_data) / len(odom_data)
            avg_vel_right = sum(d.vel_right for d in odom_data) / len(odom_data)

            print(f"\nFinal encoders: L={final.encoder_left}, R={final.encoder_right}")
            print(f"Average velocities: L={avg_vel_left:.3f}, R={avg_vel_right:.3f} m/s")
            print(f"Target velocity: 0.300 m/s")
            print(f"Accuracy: L={avg_vel_left/0.3*100:.1f}%, R={avg_vel_right/0.3*100:.1f}%")

            # Check balance
            balance_diff = abs(final.encoder_left - final.encoder_right)
            balance_pct = balance_diff / max(abs(final.encoder_left), abs(final.encoder_right)) * 100
            print(f"Encoder balance: {balance_pct:.1f}% difference")

            if balance_pct < 5:
                print("✓ PASS: Good balance")
            else:
                print("⚠ WARNING: Balance could be improved")

        else:
            print("✗ FAIL: No odometry data received")


def test_pid_tuning():
    """Test PID parameter adjustment"""
    print("\n" + "=" * 70)
    print("TEST: PID Parameter Adjustment")
    print("=" * 70)

    with RobotLink(PORT, baudrate=BAUDRATE) as robot:
        # Get initial config
        config = robot.get_config(timeout=1.0)
        if config:
            print(f"\nInitial PID gains: Kp={config}")  # Config doesn't have PID, need to add

        # Set new PID gains
        print("\nSetting PID: Kp=0.3, Ki=0.06, Kd=0.0")
        robot.set_pid(0.3, 0.06, 0.0)
        time.sleep(0.1)

        # Test with new gains
        robot.zero_encoders()
        robot.enable_stream(True, 100)
        time.sleep(0.2)

        print("Testing with velocity 0.25 m/s for 1.5s...")
        robot.set_velocity(0.25, 0.25)

        odom_data = []
        for _ in range(15):
            odom = robot.read_odom(timeout=0.2)
            if odom:
                odom_data.append(odom)

        robot.stop()
        robot.enable_stream(False, 0)

        if odom_data:
            avg_vel = sum(d.vel_left for d in odom_data) / len(odom_data)
            print(f"Average velocity: {avg_vel:.3f} m/s (target: 0.250 m/s)")
            print(f"✓ PID adjustment successful")
        else:
            print("✗ FAIL: No data received")

        # Restore original gains
        print("\nRestoring original PID: Kp=0.25, Ki=0.05, Kd=0.0")
        robot.set_pid(0.25, 0.05, 0.0)


def test_deadband_adjustment():
    """Test deadband adjustment"""
    print("\n" + "=" * 70)
    print("TEST: Deadband Adjustment")
    print("=" * 70)

    with RobotLink(PORT, baudrate=BAUDRATE) as robot:
        print("\nSetting deadband: L=15/15, R=12/12")
        robot.set_deadband(15.0, 15.0, 12.0, 12.0)
        time.sleep(0.1)

        # Test at low speed
        robot.zero_encoders()
        robot.enable_stream(True, 100)
        time.sleep(0.2)

        print("Testing low speed (0.15 m/s) for 1s...")
        robot.set_velocity(0.15, 0.15)

        odom_data = []
        for _ in range(10):
            odom = robot.read_odom(timeout=0.2)
            if odom:
                odom_data.append(odom)

        robot.stop()
        robot.enable_stream(False, 0)

        if odom_data:
            avg_vel = sum(d.vel_left for d in odom_data) / len(odom_data)
            print(f"Average velocity: {avg_vel:.3f} m/s (target: 0.150 m/s)")
            if avg_vel > 0.05:
                print("✓ Motors running with adjusted deadband")
            else:
                print("⚠ Motors struggling at low speed")
        else:
            print("✗ FAIL: No data received")

        # Restore original deadband
        print("\nRestoring original deadband: L=12/12, R=10/10")
        robot.set_deadband(12.0, 12.0, 10.0, 10.0)


def test_eeprom_save_load():
    """Test EEPROM save/load"""
    print("\n" + "=" * 70)
    print("TEST: EEPROM Save/Load")
    print("=" * 70)

    with RobotLink(PORT, baudrate=BAUDRATE) as robot:
        # Get current config
        config1 = robot.get_config(timeout=1.0)
        if not config1:
            print("✗ FAIL: Could not get config")
            return

        print("\nOriginal config:")
        print(f"  Wheel diameter: {config1.wheel_diameter} m")
        print(f"  Wheelbase: {config1.wheelbase} m")
        print(f"  Ticks/rev: {config1.ticks_per_rev}")

        # Modify config
        print("\nModifying wheelbase to 0.160 m...")
        robot.set_config(
            wheel_diameter=config1.wheel_diameter,
            wheelbase=0.160,  # Changed
            ticks_per_rev=config1.ticks_per_rev,
            invert_left=config1.invert_left,
            invert_right=config1.invert_right,
            balance_enable=config1.balance_enable,
            balance_gain=config1.balance_gain
        )
        time.sleep(0.1)

        # Save to EEPROM
        print("Saving to EEPROM...")
        robot.save_config()
        time.sleep(0.2)

        # Verify
        config2 = robot.get_config(timeout=1.0)
        if config2 and abs(config2.wheelbase - 0.160) < 0.001:
            print(f"✓ Config saved: wheelbase = {config2.wheelbase} m")
        else:
            print(f"✗ FAIL: Config not saved correctly")
            return

        # Restore original
        print("\nRestoring original wheelbase...")
        robot.set_config(
            wheel_diameter=config1.wheel_diameter,
            wheelbase=config1.wheelbase,
            ticks_per_rev=config1.ticks_per_rev,
            invert_left=config1.invert_left,
            invert_right=config1.invert_right,
            balance_enable=config1.balance_enable,
            balance_gain=config1.balance_gain
        )
        time.sleep(0.1)

        robot.save_config()
        time.sleep(0.2)

        config3 = robot.get_config(timeout=1.0)
        if config3:
            print(f"✓ Original config restored: wheelbase = {config3.wheelbase} m")
        else:
            print("✗ FAIL: Could not verify restore")


def test_motor_balance():
    """Test motor balance at different speeds"""
    print("\n" + "=" * 70)
    print("TEST: Motor Balance at Multiple Speeds")
    print("=" * 70)

    speeds = [0.15, 0.2, 0.25, 0.3, 0.35, 0.4]

    with RobotLink(PORT, baudrate=BAUDRATE) as robot:
        robot.enable_stream(True, 50)
        time.sleep(0.2)

        print(f"\n{'Speed':>6}  {'Vel L':>7}  {'Vel R':>7}  {'Enc L':>6}  {'Enc R':>6}  {'Diff':>5}  {'Balance':>7}")
        print("-" * 70)

        for speed in speeds:
            robot.zero_encoders()
            time.sleep(0.1)

            robot.set_velocity(speed, speed)

            # Collect data for 1.5 seconds
            odom_data = []
            for _ in range(30):  # 30 samples at 50ms = 1.5s
                odom = robot.read_odom(timeout=0.15)
                if odom:
                    odom_data.append(odom)

            robot.stop()
            time.sleep(0.2)

            if odom_data:
                final = odom_data[-1]
                avg_vel_l = sum(d.vel_left for d in odom_data) / len(odom_data)
                avg_vel_r = sum(d.vel_right for d in odom_data) / len(odom_data)

                diff = abs(final.encoder_left - final.encoder_right)
                balance_pct = diff / max(abs(final.encoder_left), 1) * 100

                status = "GOOD" if balance_pct < 5 else "FAIR" if balance_pct < 10 else "POOR"

                print(f"{speed:6.2f}  {avg_vel_l:7.3f}  {avg_vel_r:7.3f}  "
                      f"{final.encoder_left:6d}  {final.encoder_right:6d}  "
                      f"{diff:5d}  {status:>7}")

        robot.enable_stream(False, 0)

        print("\n✓ Motor balance test complete")


def main():
    """Run all tests"""
    print("\n" + "=" * 70)
    print("ROBOTLINK PROTOCOL COMPREHENSIVE TEST SUITE")
    print("=" * 70)

    try:
        test_streaming_velocity()
        test_pid_tuning()
        test_deadband_adjustment()
        test_eeprom_save_load()
        test_motor_balance()

        print("\n" + "=" * 70)
        print("ALL TESTS COMPLETE")
        print("=" * 70)

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n\nTest failed with error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
