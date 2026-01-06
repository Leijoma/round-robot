#!/usr/bin/env python3
"""Test velocity control"""
import serial
import time

ser = serial.Serial('/dev/cu.usbmodem212201', 9600, timeout=0.1)
time.sleep(2.5)

print("Sending VG 0.3...")
ser.write(b'VG 0.3\n')
ser.flush()

# Read for 10 seconds
start = time.time()
while time.time() - start < 10:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(line)
    time.sleep(0.01)

print("\nSending STOP...")
ser.write(b'STOP\n')
ser.flush()
time.sleep(0.5)

ser.close()
