#!/usr/bin/env python3
"""
Simple Loopback Test
Watches USB debug output while Arduino's A0-A1 are connected (loopback)
Any message sent on TX (A1) will be received on RX (A0)
"""

import serial
import time

print("="*60)
print("Simple Loopback Test - A0 and A1 connected")
print("="*60)
print("\nMonitoring USB debug output at 115200 baud")
print("You should see Arduino receiving its own messages!")
print("\nPress Ctrl+C to stop\n")
print("="*60)

try:
    ser = serial.Serial('/dev/cu.usbmodem212201', 115200, timeout=0.1)

    # Wait a bit for any startup messages
    time.sleep(1)

    # Clear any buffered data
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if line:
            print(line)

    print("\n[Waiting for loopback messages...]")
    print("(Arduino sends ODOM messages that loop back to itself)\n")

    msg_count = 0
    start_time = time.time()

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if line:
                print(line)

                # Count received messages
                if "RX: Type=" in line:
                    msg_count += 1
                    elapsed = time.time() - start_time
                    print(f"  --> Loopback message #{msg_count} (at {elapsed:.1f}s)")

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n\nTest stopped")
    print(f"Total messages received via loopback: {msg_count}")
except Exception as e:
    print(f"\nError: {e}")
finally:
    if 'ser' in locals():
        ser.close()
