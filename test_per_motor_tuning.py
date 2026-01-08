#!/usr/bin/env python3
"""
Test per-motor PID tuning capabilities
Focus on testing the left motor's higher deadband (55) vs right motor (35)
"""

import serial
import time

PORT = '/dev/cu.usbmodem112201'
BAUD = 9600

def send_command(ser, cmd, wait=0.3):
    """Send command and read responses"""
    print(f"\n> {cmd}")
    ser.write((cmd + '\n').encode('utf-8'))
    time.sleep(wait)

    responses = []
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            responses.append(line)
            print(f"  {line}")
    return responses

def collect_data(ser, duration, label):
    """Collect telemetry data for specified duration"""
    print(f"\n{'='*80}")
    print(f"Collecting data: {label}")
    print(f"{'='*80}")

    start = time.time()
    samples = []

    while time.time() - start < duration:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line and not line.startswith('→'):
                samples.append(line)
                # Print every 5th sample
                if len(samples) % 5 == 0:
                    print(f"  {line}")

    print(f"\nCollected {len(samples)} samples in {duration}s")
    print(f"{'='*80}")
    return samples

print("="*80)
print("PER-MOTOR TUNING TEST")
print("="*80)
print("\nConnecting to Arduino...")
ser = serial.Serial(PORT, BAUD, timeout=2)
time.sleep(2.5)  # Wait for Arduino reset

# Flush initial output
print("\nFlushing initial output...")
time.sleep(1)
while ser.in_waiting > 0:
    ser.readline()

print("\n" + "="*80)
print("TEST SEQUENCE START")
print("="*80)

# Check current settings - should show different deadbands
print("\n--- Current Settings (should show Left DB=55, Right DB=35) ---")
send_command(ser, 'status', wait=0.8)

# Test 1: Both motors with new deadband settings
print("\n\n### TEST 1: Both motors at 0.15 m/s (Left DB=55, Right DB=35) ###")
print("Expected: Left motor should start moving now (vs stuck before)")
send_command(ser, 'reset')
time.sleep(0.5)
send_command(ser, 'vel 0.15 0.15')
data1 = collect_data(ser, 6.0, "Both motors with independent deadband")
send_command(ser, 'stop')
time.sleep(1)

# Test 2: Try reducing left deadband to prove it needs the higher value
print("\n\n### TEST 2: Reduce left deadband to 35 (should get stuck again) ###")
send_command(ser, 'db_l 35 35')
send_command(ser, 'reset')
time.sleep(0.5)
send_command(ser, 'vel 0.15 0.15')
data2 = collect_data(ser, 4.0, "Left motor with lower deadband (35)")
send_command(ser, 'stop')
time.sleep(1)

# Test 3: Restore left deadband to 55
print("\n\n### TEST 3: Restore left deadband to 55 ###")
send_command(ser, 'db_l 55 55')
send_command(ser, 'reset')
time.sleep(0.5)
send_command(ser, 'vel 0.15 0.15')
data3 = collect_data(ser, 4.0, "Left motor with restored deadband (55)")
send_command(ser, 'stop')
time.sleep(1)

# Test 4: Try higher velocity
print("\n\n### TEST 4: Higher velocity (0.25 m/s) with optimal deadbands ###")
send_command(ser, 'reset')
time.sleep(0.5)
send_command(ser, 'vel 0.25 0.25')
data4 = collect_data(ser, 4.0, "Higher velocity test")
send_command(ser, 'stop')

print("\n" + "="*80)
print("TEST SEQUENCE COMPLETE")
print("="*80)

# Summary
print("\n\n### SUMMARY ###")
print(f"Test 1 (Optimal deadbands):    {len(data1)} samples")
print(f"Test 2 (Left DB reduced):      {len(data2)} samples")
print(f"Test 3 (Left DB restored):     {len(data3)} samples")
print(f"Test 4 (Higher velocity):      {len(data4)} samples")

print("\n\nAnalyze the output:")
print("1. Test 1: Left motor should start moving (not stuck)")
print("2. Test 2: Left motor likely stuck again with DB=35")
print("3. Test 3: Left motor should work again with DB=55")
print("4. Test 4: Both motors should track well at higher speed")

print("\nFinal Settings Check:")
send_command(ser, 'status', wait=0.8)

ser.close()
print("\nConnection closed.")
