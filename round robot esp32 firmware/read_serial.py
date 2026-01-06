#!/usr/bin/env python3
"""
Simple serial reader for ESP32 Neato Lidar
Reads and displays output from the ESP32
"""

import serial
import time
import sys

PORT = '/dev/cu.usbserial-0001'
BAUD = 115200

def main():
    print(f"Opening serial port {PORT} at {BAUD} baud...")

    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)  # Wait for ESP32 to reset

        print("Connected! Reading ESP32 output...\n")
        print("=" * 60)

        # Send 'h' to request help
        ser.write(b'h')
        time.sleep(0.5)

        # Read initial output
        start_time = time.time()
        while time.time() - start_time < 5:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                print(data.decode('utf-8', errors='ignore'), end='', flush=True)
            time.sleep(0.1)

        print("\n" + "=" * 60)
        print("\nSending 's' command to start motor...")
        ser.write(b's')
        time.sleep(0.5)

        # Read for 10 seconds after starting motor
        print("\nReading Lidar data for 10 seconds...\n")
        print("=" * 60)

        start_time = time.time()
        while time.time() - start_time < 10:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                print(data.decode('utf-8', errors='ignore'), end='', flush=True)
            time.sleep(0.05)

        print("\n" + "=" * 60)
        print("\nSending 'i' command to get info...")
        ser.write(b'i')
        time.sleep(0.5)

        # Read info
        start_time = time.time()
        while time.time() - start_time < 2:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                print(data.decode('utf-8', errors='ignore'), end='', flush=True)
            time.sleep(0.1)

        print("\n" + "=" * 60)
        print("\nTest complete!")

        ser.close()

    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return 1
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        if 'ser' in locals():
            ser.close()
        return 1

    return 0

if __name__ == '__main__':
    sys.exit(main())
