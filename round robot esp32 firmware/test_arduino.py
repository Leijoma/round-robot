#!/usr/bin/env python3
"""
Arduino motor controller test script
Sends commands and reads telemetry output
"""

import serial
import time
import sys
import threading

class ArduinoTester:
    def __init__(self, port='/dev/cu.usbmodem212201', baud=9600):
        """Initialize serial connection to Arduino"""
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2)  # Wait for Arduino to reset
            print(f"Connected to {port} at {baud} baud")

            # Start reading thread
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()

        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            sys.exit(1)

    def _read_loop(self):
        """Continuously read and print data from Arduino"""
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"RX: {line}")
            except Exception as e:
                print(f"Read error: {e}")
                break

    def send(self, cmd):
        """Send a command to Arduino"""
        try:
            self.ser.write(f"{cmd}\n".encode('utf-8'))
            print(f"TX: {cmd}")
            time.sleep(0.05)  # Small delay for command processing
        except Exception as e:
            print(f"Send error: {e}")

    def stop(self):
        """Stop motors"""
        self.send("STOP")

    def zero(self):
        """Zero encoders"""
        self.send("ZERO")

    def velocity(self, left_ms, right_ms=None):
        """Set velocity in m/s"""
        if right_ms is None:
            # Same velocity for both motors
            self.send(f"VG {left_ms}")
        else:
            # Individual velocities
            self.send(f"VL {left_ms}")
            time.sleep(0.1)
            self.send(f"VR {right_ms}")

    def pwm(self, left_pwm, right_pwm=None):
        """Set PWM directly"""
        if right_pwm is None:
            # Same PWM for both motors
            self.send(f"GO {left_pwm}")
        else:
            # Individual PWM
            self.send(f"L {left_pwm}")
            time.sleep(0.1)
            self.send(f"R {right_pwm}")

    def tune_left(self):
        """Start auto-tune for left motor"""
        self.send("TUNEL")

    def tune_right(self):
        """Start auto-tune for right motor"""
        self.send("TUNER")

    def set_pid(self, kp=None, ki=None, kd=None):
        """Set PID gains"""
        if kp is not None:
            self.send(f"KP {kp}")
        if ki is not None:
            self.send(f"KI {ki}")
        if kd is not None:
            self.send(f"KD {kd}")

    def save(self):
        """Save PID values to EEPROM"""
        self.send("SAVE")

    def load(self):
        """Load PID values from EEPROM"""
        self.send("LOAD")

    def wait(self, seconds):
        """Wait for specified seconds"""
        print(f"Waiting {seconds} seconds...")
        time.sleep(seconds)

    def close(self):
        """Close serial connection"""
        self.running = False
        if hasattr(self, 'read_thread'):
            self.read_thread.join(timeout=1)
        self.ser.close()
        print("Connection closed")


def interactive_mode(tester):
    """Interactive command mode"""
    print("\n=== Interactive Mode ===")
    print("Commands:")
    print("  vg <m/s>       - Set both motors to velocity")
    print("  vl <m/s>       - Set left motor velocity")
    print("  vr <m/s>       - Set right motor velocity")
    print("  go <pwm>       - Set both motors PWM")
    print("  l <pwm>        - Set left motor PWM")
    print("  r <pwm>        - Set right motor PWM")
    print("  stop           - Stop motors")
    print("  zero           - Zero encoders")
    print("  tunel          - Auto-tune left motor")
    print("  tuner          - Auto-tune right motor")
    print("  kp <value>     - Set Kp gain")
    print("  ki <value>     - Set Ki gain")
    print("  kd <value>     - Set Kd gain")
    print("  save           - Save to EEPROM")
    print("  load           - Load from EEPROM")
    print("  wait <sec>     - Wait for seconds")
    print("  quit           - Exit")
    print()

    try:
        while True:
            cmd = input("> ").strip().lower()

            if not cmd:
                continue

            parts = cmd.split()
            command = parts[0]

            if command == 'quit' or command == 'exit':
                break
            elif command == 'vg' and len(parts) == 2:
                tester.velocity(float(parts[1]))
            elif command == 'vl' and len(parts) == 2:
                tester.send(f"VL {parts[1]}")
            elif command == 'vr' and len(parts) == 2:
                tester.send(f"VR {parts[1]}")
            elif command == 'go' and len(parts) == 2:
                tester.pwm(int(parts[1]))
            elif command == 'l' and len(parts) == 2:
                tester.send(f"L {parts[1]}")
            elif command == 'r' and len(parts) == 2:
                tester.send(f"R {parts[1]}")
            elif command == 'stop':
                tester.stop()
            elif command == 'zero':
                tester.zero()
            elif command == 'tunel':
                tester.tune_left()
            elif command == 'tuner':
                tester.tune_right()
            elif command == 'kp' and len(parts) == 2:
                tester.set_pid(kp=float(parts[1]))
            elif command == 'ki' and len(parts) == 2:
                tester.set_pid(ki=float(parts[1]))
            elif command == 'kd' and len(parts) == 2:
                tester.set_pid(kd=float(parts[1]))
            elif command == 'save':
                tester.save()
            elif command == 'load':
                tester.load()
            elif command == 'wait' and len(parts) == 2:
                tester.wait(float(parts[1]))
            else:
                # Send raw command
                tester.send(' '.join(parts).upper())

    except KeyboardInterrupt:
        print("\nInterrupted")
    except EOFError:
        print("\nEOF")


def test_sequence(tester):
    """Run automated test sequence"""
    print("\n=== Running Automated Test Sequence ===\n")

    print("1. Stopping motors and zeroing encoders...")
    tester.stop()
    tester.wait(1)
    tester.zero()
    tester.wait(2)

    print("\n2. Testing velocity control: VG 0.3 m/s")
    tester.velocity(0.3)
    tester.wait(5)

    print("\n3. Changing velocity: VG 0.5 m/s")
    tester.velocity(0.5)
    tester.wait(5)

    print("\n4. Stopping motors: VG 0")
    tester.velocity(0.0)
    tester.wait(3)

    print("\n5. Test complete, stopping...")
    tester.stop()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Arduino motor controller test script')
    parser.add_argument('-p', '--port', default='/dev/cu.usbmodem212201',
                        help='Serial port (default: /dev/cu.usbmodem212201)')
    parser.add_argument('-b', '--baud', type=int, default=9600,
                        help='Baud rate (default: 9600)')
    parser.add_argument('-t', '--test', action='store_true',
                        help='Run automated test sequence')
    parser.add_argument('-c', '--command', type=str,
                        help='Send single command and exit')

    args = parser.parse_args()

    # Create tester instance
    tester = ArduinoTester(port=args.port, baud=args.baud)

    try:
        if args.command:
            # Single command mode
            tester.send(args.command)
            time.sleep(2)
        elif args.test:
            # Automated test mode
            test_sequence(tester)
        else:
            # Interactive mode
            interactive_mode(tester)

    finally:
        tester.close()
