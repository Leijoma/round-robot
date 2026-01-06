#!/usr/bin/env python3
"""
Run auto-tune sequence on Arduino motors
"""
import serial
import time
import sys

def run_autotune(port='/dev/cu.usbmodem212201', motor='L'):
    """Run auto-tune on specified motor (L or R)"""

    print(f"Connecting to {port}...")
    try:
        ser = serial.Serial(port, 9600, timeout=0.1)
    except serial.SerialException as e:
        print(f"Error: {e}")
        print("Make sure no other programs are using the serial port!")
        return False

    time.sleep(2.5)  # Wait for Arduino reset
    print("Connected!\n")

    # Stop motors first
    print("Stopping motors...")
    ser.write(b'STOP\n')
    ser.flush()
    time.sleep(1)

    # Clear any old data
    ser.reset_input_buffer()

    # Zero encoders
    print("Zeroing encoders...")
    ser.write(b'ZERO\n')
    ser.flush()
    time.sleep(1)

    # Clear buffer again
    ser.reset_input_buffer()

    # Start auto-tune
    cmd = f'TUNE{motor}\n'.encode()
    print(f"\nStarting auto-tune for {motor} motor...")
    print("This will take 30-40 seconds\n")
    print("=" * 60)
    ser.write(cmd)
    ser.flush()

    # Read output
    start_time = time.time()
    last_line = ""
    tune_complete = False

    try:
        while time.time() - start_time < 50:  # 50 second timeout
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(line)
                        last_line = line

                        # Check if tune is complete
                        if 'TUNE DONE' in line or 'All values auto-applied' in line:
                            tune_complete = True
                        elif 'TUNE FAILED' in line:
                            print("\nAuto-tune failed!")
                            break

                        # If we've been complete for 2 seconds, we're done
                        if tune_complete and 'auto-applied' in line:
                            time.sleep(2)
                            break

            except serial.SerialException:
                # Port was closed or disconnected
                print("\nSerial port error - connection lost")
                break

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        ser.write(b'STOP\n')
        ser.flush()

    print("=" * 60)

    # Stop motors
    print("\nStopping motors...")
    ser.write(b'STOP\n')
    ser.flush()
    time.sleep(0.5)

    ser.close()
    print("\nAuto-tune complete!")
    return tune_complete


if __name__ == "__main__":
    if len(sys.argv) > 1:
        motor = sys.argv[1].upper()
        if motor not in ['L', 'R', 'LEFT', 'RIGHT']:
            print("Usage: python3 run_autotune.py [L|R]")
            print("  L or LEFT  - tune left motor")
            print("  R or RIGHT - tune right motor")
            sys.exit(1)

        if motor == 'LEFT':
            motor = 'L'
        elif motor == 'RIGHT':
            motor = 'R'
    else:
        motor = 'L'  # Default to left motor

    success = run_autotune(motor=motor)
    sys.exit(0 if success else 1)
