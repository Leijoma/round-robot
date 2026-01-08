#!/usr/bin/env python3
"""
Automated PID tuning test sequence
Collects data for analyzing PID performance
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
                # Print every 10th sample to avoid spam
                if len(samples) % 10 == 0:
                    print(f"  {line}")

    print(f"\nCollected {len(samples)} samples in {duration}s")
    print(f"{'='*80}")
    return samples

print("="*80)
print("AUTOMATED PID TUNING TEST")
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

# Get current status
print("\n--- Current Settings ---")
send_command(ser, 'status', wait=0.5)

# Test 1: Current settings with low velocity
print("\n\n### TEST 1: Baseline with current PID settings ###")
send_command(ser, 'vel 0.15 0.15')
data1 = collect_data(ser, 5.0, "Baseline: Kp=50, Ki=20, vel=0.15")
send_command(ser, 'stop')
time.sleep(1)

# Test 2: Increase Kp to see if response improves
print("\n\n### TEST 2: Higher Kp gain ###")
send_command(ser, 'kp 80')
send_command(ser, 'reset')
time.sleep(0.5)
send_command(ser, 'vel 0.15 0.15')
data2 = collect_data(ser, 5.0, "Higher Kp: Kp=80, Ki=20, vel=0.15")
send_command(ser, 'stop')
time.sleep(1)

# Test 3: Test with original Kp but higher Ki
print("\n\n### TEST 3: Higher Ki gain ###")
send_command(ser, 'kp 50')
send_command(ser, 'ki 40')
send_command(ser, 'reset')
time.sleep(0.5)
send_command(ser, 'vel 0.15 0.15')
data3 = collect_data(ser, 5.0, "Higher Ki: Kp=50, Ki=40, vel=0.15")
send_command(ser, 'stop')
time.sleep(1)

# Test 4: Higher velocity to see performance
print("\n\n### TEST 4: Higher velocity test ###")
send_command(ser, 'kp 50')
send_command(ser, 'ki 20')
send_command(ser, 'reset')
time.sleep(0.5)
send_command(ser, 'vel 0.25 0.25')
data4 = collect_data(ser, 5.0, "Higher velocity: Kp=50, Ki=20, vel=0.25")
send_command(ser, 'stop')
time.sleep(1)

# Test 5: Single motor test
print("\n\n### TEST 5: Left motor only ###")
send_command(ser, 'reset')
time.sleep(0.5)
send_command(ser, 'vel 0.2 0')
data5 = collect_data(ser, 5.0, "Left only: Kp=50, Ki=20, vel=0.2")
send_command(ser, 'stop')

print("\n" + "="*80)
print("TEST SEQUENCE COMPLETE")
print("="*80)

# Summary
print("\n\n### SUMMARY ###")
print(f"Test 1 (Baseline):     {len(data1)} samples")
print(f"Test 2 (Higher Kp):    {len(data2)} samples")
print(f"Test 3 (Higher Ki):    {len(data3)} samples")
print(f"Test 4 (High velocity):{len(data4)} samples")
print(f"Test 5 (Single motor): {len(data5)} samples")

print("\n\nAnalyze the telemetry output above to determine:")
print("1. Does actual velocity track target velocity?")
print("2. Is there oscillation (PWM/velocity bouncing)?")
print("3. Is there steady-state error?")
print("4. Which gain settings performed best?")

print("\nRecommendations:")
print("- If actual << target: Increase Kp or check deadband")
print("- If oscillating: Reduce Kp or Ki")
print("- If steady-state error: Increase Ki")
print("- If one motor is worse: Check mechanical/electrical")

ser.close()
print("\nConnection closed.")
