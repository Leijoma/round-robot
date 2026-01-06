#!/usr/bin/env python3
"""
USB Serial Debug Monitor
Displays debug messages from Arduino on hardware serial (USB)
"""

import serial
import sys
import time

def monitor_debug(port='/dev/cu.usbmodem212201', baud=115200):
    """Monitor debug output on USB serial"""

    print(f"Connecting to {port} at {baud} baud...")
    print("Press Ctrl+C to exit\n")
    print("="*60)

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(0.5)  # Wait for Arduino reset

        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    print(line)

    except KeyboardInterrupt:
        print("\n\nMonitoring stopped")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/cu.usbmodem212201'
    monitor_debug(port)
