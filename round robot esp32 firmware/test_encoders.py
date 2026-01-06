#!/usr/bin/env python3
"""Test if encoders are working"""
import serial
import time

ser = serial.Serial('/dev/cu.usbmodem212201', 9600, timeout=0.1)
time.sleep(2.5)

# Clear buffer
while ser.in_waiting > 0:
    ser.readline()

print("Testing encoder response to direct PWM...\n")

# Zero encoders
ser.write(b'ZERO\n')
time.sleep(0.3)

# Send direct PWM (bypasses PID)
print("Sending L 100 (left motor PWM=100)")
ser.write(b'L 100\n')
time.sleep(0.2)

# Wait and watch telemetry
print("Watching telemetry for 3 seconds...\n")
for i in range(15):
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(line)
    except:
        pass
    time.sleep(0.2)

# Stop
print("\nStopping...")
ser.write(b'STOP\n')
time.sleep(0.5)

ser.close()
print("\nIf velocity stayed at 0.0000, encoders are not working!")
