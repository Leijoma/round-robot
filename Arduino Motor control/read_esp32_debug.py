#!/usr/bin/env python3
import serial
import time

try:
    ser = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=0.1)
    print("Reading ESP32 debug output for 5 seconds...\n")
    time.sleep(0.5)

    start = time.time()
    while time.time() - start < 5:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"[ESP32] {line}")
        time.sleep(0.01)

    ser.close()
except Exception as e:
    print(f"Error: {e}")
