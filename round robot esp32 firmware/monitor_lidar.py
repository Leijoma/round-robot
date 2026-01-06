#!/usr/bin/env python3
"""
Interactive monitor for ESP32 Neato Lidar with P-controller
Monitors RPM control and allows manual commands
"""

import serial
import time
import sys

PORT = '/dev/cu.usbserial-0001'
BAUD = 115200

def main():
    print(f"Opening serial port {PORT} at {BAUD} baud...")

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        time.sleep(2)  # Wait for ESP32 to reset

        print("Connected! Monitoring ESP32 output...")
        print("=" * 70)
        print("Commands: s=start, x=stop, +=faster, -=slower, i=info, r=reset, h=help")
        print("=" * 70)
        print()

        # Send 's' to start motor
        print("Sending 's' to start motor...")
        ser.write(b's')
        time.sleep(0.5)

        print("\nMonitoring Lidar output (Ctrl+C to exit):\n")

        # Monitor for 60 seconds
        start_time = time.time()
        while time.time() - start_time < 60:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                print(data.decode('utf-8', errors='ignore'), end='', flush=True)
            time.sleep(0.05)

        print("\n\n" + "=" * 70)
        print("Monitoring complete. Requesting final status...")
        print("=" * 70)

        # Get final info
        ser.write(b'i')
        time.sleep(1)

        while ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print(data.decode('utf-8', errors='ignore'), end='', flush=True)
            time.sleep(0.1)

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
