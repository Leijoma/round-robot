#!/usr/bin/env python3
"""
Test differential drive kinematics
Verify angular velocity control and encoder tick matching
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

def run_drive_test(ser, linear, angular, duration, label):
    """Run drive command and collect data"""
    print(f"\n{'='*80}")
    print(f"{label}")
    print(f"Linear={linear} m/s, Angular={angular} rad/s, Duration={duration}s")
    print(f"{'='*80}")

    send_command(ser, 'reset')
    time.sleep(0.5)
    send_command(ser, f'drive {linear} {angular}')

    start = time.time()
    samples = []

    while time.time() - start < duration:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if '|' in line and not line.startswith('→'):
                samples.append(line)
                if len(samples) % 10 == 0:
                    print(f"  {line}")

    send_command(ser, 'stop')
    time.sleep(0.5)

    if len(samples) > 5:
        print(f"\n  Last 3 samples:")
        for line in samples[-3:]:
            print(f"    {line}")

    return samples

print("="*80)
print("DIFFERENTIAL DRIVE TEST")
print("="*80)
print("\nDifferential Drive Kinematics:")
print("  v_left = v_linear - (omega * wheelbase / 2)")
print("  v_right = v_linear + (omega * wheelbase / 2)")
print("\nWith wheelbase = 0.244m:")
print("  omega = 1 rad/s → v_left = -0.122 m/s, v_right = +0.122 m/s")

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

print(f"\n{'='*80}")
print("RUNNING TESTS")
print(f"{'='*80}")

# Test 1: Drive straight
test1 = run_drive_test(ser, 0.15, 0.0, 6.0, "TEST 1: Drive Straight")

# Test 2: Rotate in place (counterclockwise)
test2 = run_drive_test(ser, 0.0, 0.5, 6.0, "TEST 2: Rotate in Place (CCW)")

# Test 3: Drive forward while turning left
test3 = run_drive_test(ser, 0.15, 0.3, 6.0, "TEST 3: Forward + Turn Left")

# Test 4: Drive forward while turning right
test4 = run_drive_test(ser, 0.15, -0.3, 6.0, "TEST 4: Forward + Turn Right")

# Test 5: Gentle rotation
test5 = run_drive_test(ser, 0.0, 0.2, 6.0, "TEST 5: Slow Rotation")

print(f"\n{'='*80}")
print("TEST SUMMARY")
print(f"{'='*80}")
print(f"Test 1 (Straight):    {len(test1)} samples")
print(f"Test 2 (Rotate):      {len(test2)} samples")
print(f"Test 3 (Fwd+Left):    {len(test3)} samples")
print(f"Test 4 (Fwd+Right):   {len(test4)} samples")
print(f"Test 5 (Slow Rot):    {len(test5)} samples")

print(f"\n{'='*80}")
print("ANALYSIS NOTES")
print(f"{'='*80}")
print("1. For straight driving (Test 1):")
print("   - Left and right actual velocities should match")
print("   - Angular velocity should be near 0")
print("\n2. For rotation in place (Tests 2, 5):")
print("   - Left and right velocities should be equal and opposite")
print("   - Actual angular velocity should track target")
print("\n3. For turning while driving (Tests 3, 4):")
print("   - One wheel faster, one slower")
print("   - Combined linear velocity should track target")
print("\nKey metric: Encoder tick difference should be minimal for")
print("straight driving, and symmetric for rotation in place.")

ser.close()
print("\nTest complete!")
