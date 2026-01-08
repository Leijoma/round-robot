#!/usr/bin/env python3
"""
Find optimal deadband for left motor
Test progressively lower deadband values to find minimum that still starts motor
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

def test_deadband(ser, db_value, target_vel=0.15, duration=5.0):
    """Test if motor starts and measure steady-state velocity"""
    print(f"\n--- Testing Left Deadband = {db_value} ---")

    send_command(ser, f'db_l {db_value} {db_value}')
    send_command(ser, 'reset')
    time.sleep(0.5)
    send_command(ser, f'vel {target_vel} {target_vel}')

    # Collect data
    left_vels = []
    right_vels = []
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
                        left_vels.append(left_vel)
                        right_vels.append(right_vel)
                except:
                    pass

    send_command(ser, 'stop')
    time.sleep(1)

    if len(left_vels) < 20:
        return None

    # Check if motor started (velocity > 0.05 m/s within first 2 seconds)
    early_samples = left_vels[:20]  # First ~2 seconds
    motor_started = any(v > 0.05 for v in early_samples)

    # Steady state analysis (last half of data)
    steady_state = left_vels[len(left_vels)//2:]
    mean_vel = np.mean(steady_state)
    error = abs(mean_vel - target_vel)
    error_pct = (error / target_vel * 100)

    print(f"  Motor started: {'YES' if motor_started else 'NO'}")
    print(f"  Steady-state: {mean_vel:.3f} m/s (error: {error_pct:.1f}%)")

    return {
        'db': db_value,
        'started': motor_started,
        'mean_vel': mean_vel,
        'error': error,
        'error_pct': error_pct
    }

print("="*80)
print("DEADBAND TUNING - Finding Optimal Left Motor Deadband")
print("="*80)

ser = serial.Serial(PORT, BAUD, timeout=2)
time.sleep(2.5)

# Flush
time.sleep(1)
while ser.in_waiting > 0:
    ser.readline()

print("\nStrategy: Find minimum deadband that reliably starts motor")
print("Testing deadband values from 55 down to 40...")

# Test deadband values
deadband_values = [55, 50, 48, 46, 44, 42, 40]
results = []

for db in deadband_values:
    result = test_deadband(ser, db, target_vel=0.15, duration=5.0)
    if result:
        results.append(result)
    time.sleep(0.5)

print(f"\n{'='*80}")
print("RESULTS SUMMARY")
print(f"{'='*80}")
print(f"{'Deadband':<10} {'Started':<10} {'Mean Vel':<12} {'Error %':<10} {'Status'}")
print("-" * 80)

best_db = None
for r in results:
    status = "✓ Good" if (r['started'] and r['error_pct'] < 15) else ("✗ Stuck" if not r['started'] else "✓ Started")
    print(f"{r['db']:<10.0f} {'YES' if r['started'] else 'NO':<10} {r['mean_vel']:<12.3f} {r['error_pct']:<10.1f} {status}")

    # Best deadband: lowest value that starts motor and has reasonable error
    if r['started'] and r['error_pct'] < 20:
        best_db = r['db']

print(f"\n{'='*80}")
if best_db:
    print(f"RECOMMENDATION: Set Left Deadband = {best_db}")
else:
    print("WARNING: Could not find suitable deadband!")
print(f"{'='*80}")

# Apply and verify
if best_db:
    print(f"\nApplying deadband = {best_db} and running verification...")
    send_command(ser, f'db_l {best_db} {best_db}')
    send_command(ser, 'kp_l 50')  # Restore default Kp
    send_command(ser, 'reset')
    time.sleep(0.5)
    send_command(ser, 'vel 0.15 0.15')

    print("\nVerification test (6 seconds):")
    start = time.time()
    count = 0
    while time.time() - start < 6.0:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if '|' in line and not line.startswith('→'):
                count += 1
                if count % 10 == 0:
                    print(f"  {line}")

    send_command(ser, 'stop')

print("\nFinal settings check:")
send_command(ser, 'status', wait=0.8)

ser.close()
