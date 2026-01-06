#!/usr/bin/env python3
"""
Simple serial monitor for ESP32 Neato Lidar
Press 's' to start motor, 'x' to stop, 'h' for help
Press Ctrl+C to exit
"""

import serial
import sys
import select
import termios
import tty

PORT = '/dev/cu.usbserial-0001'
BAUD = 115200

def main():
    print(f"Opening serial port {PORT} at {BAUD} baud...")

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        print("Connected! Press Ctrl+C to exit\n")
        print("Available commands: s=start motor, x=stop, +=faster, -=slower, h=help, i=info\n")

        # Save original terminal settings
        old_settings = termios.tcgetattr(sys.stdin)

        try:
            # Set terminal to raw mode for single-char input
            tty.setraw(sys.stdin.fileno())

            while True:
                # Check for keyboard input
                if select.select([sys.stdin], [], [], 0)[0]:
                    char = sys.stdin.read(1)
                    if char == '\x03':  # Ctrl+C
                        break
                    # Send to ESP32
                    ser.write(char.encode())

                # Read from ESP32
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    sys.stdout.write(data.decode('utf-8', errors='ignore'))
                    sys.stdout.flush()

        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return 1
    except KeyboardInterrupt:
        print("\n\nExiting...")
    finally:
        if 'ser' in locals():
            ser.close()

    return 0

if __name__ == '__main__':
    sys.exit(main())
