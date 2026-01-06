#!/usr/bin/env python3
"""
Test encoder balancing feature
Compare straight-line driving with and without balancing
"""
import serial
import time

def run_test(ser, vel_ms, duration, balance_enabled):
    """Run test with/without balancing and return encoder difference"""

    # Set balance mode
    if balance_enabled:
        ser.write(b'BAL ON\n')
    else:
        ser.write(b'BAL OFF\n')
    time.sleep(0.2)

    # Clear
    while ser.in_waiting > 0:
        ser.readline()

    # Zero and start
    ser.write(b'STOP\n')
    time.sleep(0.3)
    ser.write(b'ZERO\n')
    time.sleep(0.3)

    # Start driving
    cmd = f'VG {vel_ms}\n'.encode()
    ser.write(cmd)
    time.sleep(0.2)

    # Clear initial responses
    while ser.in_waiting > 0:
        ser.readline()

    # Drive for duration
    time.sleep(duration)

    # Stop
    ser.write(b'STOP\n')
    time.sleep(0.5)

    # Get final encoder values
    ser.write(b'ENC\n')
    time.sleep(0.3)

    encL, encR = 0, 0
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line.startswith('ENC L='):
            # Parse: ENC L=1234 R=1230
            parts = line.split()
            encL = int(parts[1].split('=')[1])
            encR = int(parts[2].split('=')[1])

    diff = encL - encR
    avg_enc = (encL + encR) / 2
    diff_percent = (diff / avg_enc * 100) if avg_enc > 0 else 0

    return {
        'left': encL,
        'right': encR,
        'diff': diff,
        'diff_percent': diff_percent
    }

def main():
    print("="*70)
    print("ENCODER BALANCING TEST")
    print("="*70)
    print("\nComparing straight-line driving with and without balancing")
    print("Goal: Keep encoder difference close to zero for straight driving\n")

    ser = serial.Serial('/dev/cu.usbmodem212201', 9600, timeout=0.1)
    time.sleep(2.5)

    # Clear startup
    while ser.in_waiting > 0:
        ser.readline()

    test_speeds = [0.2, 0.3, 0.4]
    duration = 8  # seconds

    print(f"{'Speed':>7} {'Mode':>12} {'Left':>8} {'Right':>8} {'Diff':>7} {'Diff%':>7} {'Result':>10}")
    print("-"*70)

    for vel in test_speeds:
        # Test WITHOUT balancing
        result_off = run_test(ser, vel, duration, False)
        status_off = "GOOD" if abs(result_off['diff_percent']) < 5 else "POOR"

        print(f"{vel:>7.2f} {'NO BALANCE':>12} "
              f"{result_off['left']:>8} {result_off['right']:>8} "
              f"{result_off['diff']:>+7} {result_off['diff_percent']:>+6.1f}% "
              f"{status_off:>10}")

        time.sleep(1)

        # Test WITH balancing
        result_on = run_test(ser, vel, duration, True)
        status_on = "EXCELLENT" if abs(result_on['diff_percent']) < 2 else \
                    "GOOD" if abs(result_on['diff_percent']) < 5 else "POOR"

        print(f"{vel:>7.2f} {'BALANCED':>12} "
              f"{result_on['left']:>8} {result_on['right']:>8} "
              f"{result_on['diff']:>+7} {result_on['diff_percent']:>+6.1f}% "
              f"{status_on:>10}")

        # Show improvement
        improvement = abs(result_off['diff_percent']) - abs(result_on['diff_percent'])
        if improvement > 0:
            print(f"{'':>7} {'→ IMPROVED':>12} by {improvement:+.1f}%\n")
        elif improvement < 0:
            print(f"{'':>7} {'→ WORSE':>12} by {abs(improvement):.1f}%\n")
        else:
            print(f"{'':>7} {'→ NO CHANGE':>12}\n")

        time.sleep(1)

    ser.close()

    print("="*70)
    print("SUMMARY")
    print("="*70)
    print("\nEncoder balancing adjusts motor speeds dynamically to keep")
    print("same total distance on both wheels for perfect straight driving.")
    print("\nCommands:")
    print("  BAL ON   - Enable balancing (default)")
    print("  BAL OFF  - Disable balancing")
    print("  BALG <val> - Adjust balance gain (default 0.02)")
    print("="*70)

if __name__ == "__main__":
    main()
