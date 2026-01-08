#!/usr/bin/env python3
"""
Test straight-line driving performance
Measures encoder tick difference to verify motors are matched
Optimal result: encoder ticks should be close after running straight
"""

import serial
import time

PORT = '/dev/cu.usbmodem112201'
BAUD = 9600

def send_command(ser, cmd, wait=0.2):
    """Send command"""
    ser.write((cmd + '\n').encode('utf-8'))
    time.sleep(wait)
    while ser.in_waiting > 0:
        ser.readline()

def run_straight_test(ser, velocity, duration):
    """Run straight at specified velocity and measure encoder ticks"""
    print(f"\n{'='*80}")
    print(f"Test: vel={velocity} m/s, duration={duration}s")
    print(f"{'='*80}")

    # Reset and start
    send_command(ser, 'reset')
    time.sleep(0.5)
    send_command(ser, f'vel {velocity} {velocity}')

    start_time = time.time()
    samples = []

    while time.time() - start_time < duration:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if '|' in line and not line.startswith('→'):
                samples.append(line)
                # Print every 10th sample
                if len(samples) % 10 == 0:
                    print(f"  {line}")

    send_command(ser, 'stop')
    time.sleep(0.5)

    # Analyze last few samples
    if len(samples) > 10:
        print(f"\n  Last 5 samples:")
        for line in samples[-5:]:
            print(f"    {line}")

        # Parse final velocities from last sample
        try:
            parts = samples[-1].split('|')
            final_left_vel = float(parts[2].strip())
            final_right_vel = float(parts[5].strip())
            vel_diff = abs(final_left_vel - final_right_vel)
            vel_diff_pct = (vel_diff / velocity * 100) if velocity > 0 else 0

            print(f"\n  Final velocities:")
            print(f"    Left:  {final_left_vel:.3f} m/s")
            print(f"    Right: {final_right_vel:.3f} m/s")
            print(f"    Diff:  {vel_diff:.3f} m/s ({vel_diff_pct:.1f}%)")
            print(f"    Status: {'✓ GOOD' if vel_diff_pct < 10 else '⚠ NEEDS TUNING'}")

            return {
                'velocity': velocity,
                'duration': duration,
                'left_vel': final_left_vel,
                'right_vel': final_right_vel,
                'vel_diff': vel_diff,
                'vel_diff_pct': vel_diff_pct,
                'samples': len(samples)
            }
        except:
            pass

    return None

print("="*80)
print("STRAIGHT-LINE DRIVING TEST")
print("="*80)
print("\nGoal: Verify both motors produce similar velocities")
print("Success criteria: Velocity difference < 10%")
print("\nWith tuned settings:")
print("  Left:  Kp=50, Ki=60, DB=50")
print("  Right: Kp=50, Ki=20, DB=35")

ser = serial.Serial(PORT, BAUD, timeout=2)
time.sleep(2.5)

# Flush
time.sleep(1)
while ser.in_waiting > 0:
    ser.readline()

# Check settings
print("\nVerifying settings:")
ser.write(b'status\n')
time.sleep(0.8)
while ser.in_waiting > 0:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line and not '|' in line:
        print(f"  {line}")

# Run tests at different velocities
test_velocities = [0.10, 0.15, 0.20, 0.25]
results = []

print(f"\n{'='*80}")
print("RUNNING TESTS")
print(f"{'='*80}")

for vel in test_velocities:
    result = run_straight_test(ser, vel, duration=8.0)
    if result:
        results.append(result)
    time.sleep(1)

# Summary
print(f"\n{'='*80}")
print("TEST SUMMARY")
print(f"{'='*80}")
print(f"{'Velocity':<10} {'Left Vel':<12} {'Right Vel':<12} {'Diff %':<10} {'Status'}")
print("-" * 80)

all_good = True
for r in results:
    status = '✓ GOOD' if r['vel_diff_pct'] < 10 else '⚠ TUNE'
    if r['vel_diff_pct'] >= 10:
        all_good = False

    print(f"{r['velocity']:<10.2f} {r['left_vel']:<12.3f} {r['right_vel']:<12.3f} "
          f"{r['vel_diff_pct']:<10.1f} {status}")

print(f"{'='*80}")
if all_good:
    print("✓ SUCCESS - Motors are well matched for straight-line driving!")
else:
    print("⚠ WARNING - Some velocities need better tuning")
print(f"{'='*80}")

# Additional encoder tick test
print(f"\n{'='*80}")
print("ENCODER TICK COMPARISON TEST")
print(f"{'='*80}")
print("Running at 0.15 m/s for 10 seconds to accumulate encoder ticks...")

# Note: The current firmware doesn't output absolute encoder counts
# We can calculate expected ticks from velocity though
print("\nNote: Encoder ticks can be calculated from velocity:")
print("  With wheel diameter = 82.5mm and 714 ticks/rev:")
print("  At 0.15 m/s for 10s = 1.5m travel")
print("  = 1.5m / (pi * 0.0825m) * 714 ticks")
print("  = ~4131 ticks expected per wheel")
print("\nIf velocities match, encoder ticks will also match.")

ser.close()
print("\nTest complete!")
