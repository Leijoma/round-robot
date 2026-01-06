#!/usr/bin/env python3
"""
Test motor balance - check if both motors run at same speed
Critical for straight line driving
"""
import serial
import time
import statistics

def test_velocity(ser, vel_ms, duration=5):
    """Test a specific velocity and return motor balance statistics"""

    # Zero encoders and reset
    ser.write(b'STOP\n')
    time.sleep(0.3)
    ser.write(b'ZERO\n')
    time.sleep(0.3)

    # Clear buffer
    while ser.in_waiting > 0:
        ser.readline()

    # Start velocity
    cmd = f'VG {vel_ms}\n'.encode()
    ser.write(cmd)
    time.sleep(0.3)

    # Collect data
    left_vels = []
    right_vels = []
    left_pwms = []
    right_pwms = []

    start = time.time()
    while time.time() - start < duration:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('V '):
                    # Parse: V 0.2719 0.2719 T 0.300 0.300 PWM 43 44 PID ON
                    parts = line.split()
                    if len(parts) >= 10:
                        vel_left = float(parts[1])
                        vel_right = float(parts[2])
                        pwm_left = int(parts[7])
                        pwm_right = int(parts[8])

                        # Only collect after 1 second (let it stabilize)
                        if time.time() - start > 1.0:
                            left_vels.append(vel_left)
                            right_vels.append(vel_right)
                            left_pwms.append(pwm_left)
                            right_pwms.append(pwm_right)
        except:
            pass
        time.sleep(0.01)

    # Stop
    ser.write(b'STOP\n')
    time.sleep(0.5)

    # Calculate statistics
    if len(left_vels) > 5:
        avg_left = statistics.mean(left_vels)
        avg_right = statistics.mean(right_vels)
        std_left = statistics.stdev(left_vels) if len(left_vels) > 1 else 0
        std_right = statistics.stdev(right_vels) if len(right_vels) > 1 else 0
        avg_pwm_left = statistics.mean(left_pwms)
        avg_pwm_right = statistics.mean(right_pwms)

        diff_vel = avg_left - avg_right
        diff_percent = (diff_vel / vel_ms * 100) if vel_ms > 0 else 0

        return {
            'target': vel_ms,
            'left_vel': avg_left,
            'right_vel': avg_right,
            'left_std': std_left,
            'right_std': std_right,
            'diff_vel': diff_vel,
            'diff_percent': diff_percent,
            'left_pwm': avg_pwm_left,
            'right_pwm': avg_pwm_right,
            'samples': len(left_vels)
        }
    else:
        return None


def main():
    print("="*70)
    print("MOTOR BALANCE TEST")
    print("="*70)
    print("\nTesting if both motors run at same speed for straight driving")
    print("Critical: difference should be < 5% for good tracking\n")

    ser = serial.Serial('/dev/cu.usbmodem212201', 9600, timeout=0.1)
    time.sleep(2.5)

    # Clear startup messages
    while ser.in_waiting > 0:
        ser.readline()

    # Test velocities from 0.05 to 0.5 m/s
    test_velocities = [0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5]

    results = []

    print(f"{'Target':>8} {'Left':>8} {'Right':>8} {'Diff':>8} {'Diff%':>7} {'L_PWM':>7} {'R_PWM':>7} {'Status':>10}")
    print("-"*70)

    for vel in test_velocities:
        print(f"{vel:>8.2f} ", end='', flush=True)

        result = test_velocity(ser, vel, duration=6)

        if result:
            results.append(result)

            # Determine status
            diff_abs = abs(result['diff_percent'])
            if diff_abs < 2:
                status = "EXCELLENT"
            elif diff_abs < 5:
                status = "GOOD"
            elif diff_abs < 10:
                status = "FAIR"
            else:
                status = "POOR"

            print(f"{result['left_vel']:>8.3f} {result['right_vel']:>8.3f} "
                  f"{result['diff_vel']:>+8.3f} {result['diff_percent']:>+6.1f}% "
                  f"{result['left_pwm']:>7.1f} {result['right_pwm']:>7.1f} "
                  f"{status:>10}")
        else:
            print("FAILED - no data")

        time.sleep(0.5)

    # Summary
    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)

    if results:
        avg_diff = statistics.mean([abs(r['diff_percent']) for r in results])
        max_diff = max([abs(r['diff_percent']) for r in results])

        print(f"\nAverage difference: {avg_diff:.2f}%")
        print(f"Maximum difference: {max_diff:.2f}%")

        if avg_diff < 5:
            print("\n✓ PASS: Motors are well balanced for straight driving")
        elif avg_diff < 10:
            print("\n⚠ WARNING: Moderate imbalance - may need calibration")
        else:
            print("\n✗ FAIL: Significant imbalance - calibration required")

        # Check if one motor consistently faster
        left_faster_count = sum(1 for r in results if r['diff_vel'] > 0)
        right_faster_count = sum(1 for r in results if r['diff_vel'] < 0)

        print(f"\nLeft motor faster:  {left_faster_count}/{len(results)} times")
        print(f"Right motor faster: {right_faster_count}/{len(results)} times")

        if left_faster_count > right_faster_count * 2:
            print("\n→ Left motor consistently faster - may need deadband adjustment")
            print(f"   Try: DB R F {results[-1]['right_pwm'] + 5:.0f}")
        elif right_faster_count > left_faster_count * 2:
            print("\n→ Right motor consistently faster - may need deadband adjustment")
            print(f"   Try: DB L F {results[-1]['left_pwm'] + 5:.0f}")
        else:
            print("\n→ Balance is good across speed range")

    ser.close()
    print("\n" + "="*70)


if __name__ == "__main__":
    main()
