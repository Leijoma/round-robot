#!/usr/bin/env python3
"""
Tune Ki for left motor with DB=50 to reduce steady-state error
"""

import serial
import time
import numpy as np

PORT = '/dev/cu.usbmodem112201'
BAUD = 9600

def send_command(ser, cmd, wait=0.2):
    """Send command"""
    ser.write((cmd + '\n').encode('utf-8'))
    time.sleep(wait)
    while ser.in_waiting > 0:
        ser.readline()

def test_ki(ser, ki_value, target_vel=0.15, duration=6.0):
    """Test specific Ki value"""
    print(f"\n--- Testing Left Ki = {ki_value} ---")

    send_command(ser, f'ki_l {ki_value}')
    send_command(ser, 'reset')
    time.sleep(0.5)
    send_command(ser, f'vel {target_vel} {target_vel}')

    # Collect data
    left_vels = []
    right_vels = []
    pwms_left = []
    start = time.time()

    while time.time() - start < duration:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if '|' in line and not line.startswith('→'):
                try:
                    parts = line.split('|')
                    if len(parts) >= 7:
                        left_vel = float(parts[2].strip())
                        right_vel = float(parts[5].strip())
                        pwm_left = int(parts[3].strip())
                        left_vels.append(left_vel)
                        right_vels.append(right_vel)
                        pwms_left.append(pwm_left)
                except:
                    pass

    send_command(ser, 'stop')
    time.sleep(0.5)

    if len(left_vels) < 20:
        return None

    # Analyze steady-state (last 50%)
    mid = len(left_vels) // 2
    left_ss = left_vels[mid:]
    right_ss = right_vels[mid:]

    left_mean = np.mean(left_ss)
    right_mean = np.mean(right_ss)
    left_std = np.std(left_ss)
    match_error = abs(left_mean - right_mean)
    left_error = abs(left_mean - target_vel)
    left_error_pct = (left_error / target_vel * 100)

    print(f"  Left:  Mean={left_mean:.3f} Error={left_error_pct:.1f}% Std={left_std:.3f}")
    print(f"  Right: Mean={right_mean:.3f}")
    print(f"  Match: {match_error:.3f} m/s ({match_error/target_vel*100:.1f}%)")

    return {
        'ki': ki_value,
        'left_mean': left_mean,
        'right_mean': right_mean,
        'left_error_pct': left_error_pct,
        'match_error': match_error,
        'left_std': left_std
    }

print("="*80)
print("Ki TUNING - Reducing steady-state error with DB=50")
print("="*80)

ser = serial.Serial(PORT, BAUD, timeout=2)
time.sleep(2.5)

# Flush
time.sleep(1)
while ser.in_waiting > 0:
    ser.readline()

print("\nSetting left motor DB=50, Kp=50")
send_command(ser, 'db_l 50 50')
send_command(ser, 'kp_l 50')
time.sleep(0.5)

# Test Ki values
ki_values = [20, 30, 40, 50, 60]
results = []

for ki in ki_values:
    result = test_ki(ser, ki, target_vel=0.15, duration=6.0)
    if result:
        results.append(result)
    time.sleep(0.5)

print(f"\n{'='*80}")
print("RESULTS SUMMARY")
print(f"{'='*80}")
print(f"{'Ki':<6} {'Left Mean':<12} {'Right Mean':<12} {'Match Err':<12} {'Left Err%':<10} {'Std':<8}")
print("-" * 80)

best_ki = None
best_score = float('inf')

for r in results:
    # Score based on match error and tracking error
    score = r['match_error'] + r['left_error_pct']/100.0

    print(f"{r['ki']:<6.0f} {r['left_mean']:<12.3f} {r['right_mean']:<12.3f} "
          f"{r['match_error']:<12.3f} {r['left_error_pct']:<10.1f} {r['left_std']:<8.3f}")

    if score < best_score and r['left_std'] < 0.015:  # Not too oscillatory
        best_score = score
        best_ki = r['ki']

print(f"\n{'='*80}")
print(f"RECOMMENDATION: Set Left Ki = {best_ki}")
print(f"{'='*80}")

# Apply and verify
if best_ki:
    print(f"\nApplying Ki={best_ki} and running final verification...")
    send_command(ser, f'ki_l {best_ki}')
    send_command(ser, 'reset')
    time.sleep(0.5)
    send_command(ser, 'vel 0.15 0.15')

    print("\nVerification (8 seconds):")
    start = time.time()
    count = 0
    while time.time() - start < 8.0:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if '|' in line and not line.startswith('→'):
                count += 1
                if count % 10 == 0:
                    print(f"  {line}")

    send_command(ser, 'stop')

print("\n" + "="*80)
print("TUNING COMPLETE")
print("="*80)
print(f"\nOptimal settings:")
print(f"  Left:  Kp=50, Ki={best_ki}, DB=50")
print(f"  Right: Kp=50, Ki=20, DB=35")

ser.close()
