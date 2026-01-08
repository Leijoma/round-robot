#!/usr/bin/env python3
"""
Iterative PID tuning to match motor characteristics
Tests different Kp values for left motor to match right motor performance
"""

import serial
import time
import numpy as np

PORT = '/dev/cu.usbmodem112201'
BAUD = 9600

def send_command(ser, cmd, wait=0.2):
    """Send command and read responses"""
    ser.write((cmd + '\n').encode('utf-8'))
    time.sleep(wait)
    while ser.in_waiting > 0:
        ser.readline()

def collect_velocity_data(ser, duration):
    """Collect velocity data and return left/right velocities"""
    start = time.time()
    left_vels = []
    right_vels = []

    while time.time() - start < duration:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if '|' in line and not line.startswith('→'):
                try:
                    parts = line.split('|')
                    if len(parts) >= 7:
                        left_vel = float(parts[2].strip())
                        right_vel = float(parts[5].strip())
                        left_vels.append(left_vel)
                        right_vels.append(right_vel)
                except:
                    pass

    return left_vels, right_vels

def analyze_performance(velocities, target):
    """Calculate performance metrics"""
    if len(velocities) < 10:
        return None

    # Use last 50% of data for steady-state analysis
    steady_state = velocities[len(velocities)//2:]

    mean_vel = np.mean(steady_state)
    std_vel = np.std(steady_state)
    error = abs(mean_vel - target)
    error_pct = (error / target * 100) if target > 0 else 0

    return {
        'mean': mean_vel,
        'std': std_vel,
        'error': error,
        'error_pct': error_pct,
        'overshoot': max(0, mean_vel - target)
    }

def test_kp_value(ser, kp_left, target_vel=0.15, duration=5.0):
    """Test a specific Kp value for left motor"""
    print(f"\n--- Testing Left Kp = {kp_left} ---")

    send_command(ser, f'kp_l {kp_left}')
    send_command(ser, 'reset')
    time.sleep(0.5)
    send_command(ser, f'vel {target_vel} {target_vel}')

    left_vels, right_vels = collect_velocity_data(ser, duration)

    send_command(ser, 'stop')
    time.sleep(1)

    left_perf = analyze_performance(left_vels, target_vel)
    right_perf = analyze_performance(right_vels, target_vel)

    if left_perf and right_perf:
        print(f"Left:  Mean={left_perf['mean']:.3f} Error={left_perf['error_pct']:.1f}% Std={left_perf['std']:.3f}")
        print(f"Right: Mean={right_perf['mean']:.3f} Error={right_perf['error_pct']:.1f}% Std={right_perf['std']:.3f}")

        # Calculate how well motors match
        match_error = abs(left_perf['mean'] - right_perf['mean'])
        print(f"Match: Difference={match_error:.3f} m/s ({match_error/target_vel*100:.1f}%)")

        return {
            'kp': kp_left,
            'left': left_perf,
            'right': right_perf,
            'match_error': match_error
        }

    return None

print("="*80)
print("ITERATIVE PID TUNING - Finding Optimal Left Motor Kp")
print("="*80)

ser = serial.Serial(PORT, BAUD, timeout=2)
time.sleep(2.5)

# Flush
time.sleep(1)
while ser.in_waiting > 0:
    ser.readline()

print("\nCurrent settings:")
send_command(ser, 'status', wait=0.8)

# Test different Kp values for left motor
# Right motor works well with Kp=50, left overshoots
# Try reducing left Kp to find sweet spot
kp_values = [50, 40, 35, 30, 25]
results = []

print(f"\n{'='*80}")
print("TUNING SEQUENCE - Testing Kp values")
print(f"{'='*80}")

for kp in kp_values:
    result = test_kp_value(ser, kp, target_vel=0.15, duration=5.0)
    if result:
        results.append(result)
    time.sleep(1)

# Find best Kp (minimum match error while maintaining good tracking)
print(f"\n{'='*80}")
print("RESULTS SUMMARY")
print(f"{'='*80}")
print(f"{'Kp':<6} {'Left Mean':<12} {'Right Mean':<12} {'Match Err':<12} {'Left Err%':<10}")
print("-" * 80)

best_kp = None
best_match = float('inf')

for r in results:
    print(f"{r['kp']:<6.0f} {r['left']['mean']:<12.3f} {r['right']['mean']:<12.3f} "
          f"{r['match_error']:<12.3f} {r['left']['error_pct']:<10.1f}")

    # Find best match (low match error and low tracking error)
    if r['match_error'] < best_match and r['left']['error_pct'] < 20:
        best_match = r['match_error']
        best_kp = r['kp']

print(f"\n{'='*80}")
print(f"RECOMMENDATION: Set Left Kp = {best_kp}")
print(f"{'='*80}")

# Apply best settings
if best_kp:
    print(f"\nApplying optimal settings...")
    send_command(ser, f'kp_l {best_kp}')
    time.sleep(0.5)

print("\nFinal verification test (6 seconds)...")
send_command(ser, 'reset')
time.sleep(0.5)
send_command(ser, 'vel 0.15 0.15')

start = time.time()
sample_count = 0
while time.time() - start < 6.0:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if '|' in line and not line.startswith('→'):
            sample_count += 1
            if sample_count % 10 == 0:
                print(f"  {line}")

send_command(ser, 'stop')

print("\n" + "="*80)
print("TUNING COMPLETE")
print("="*80)
print(f"\nFinal settings: Left Kp={best_kp}, Right Kp=50")
print("Left Deadband=55, Right Deadband=35")

ser.close()
