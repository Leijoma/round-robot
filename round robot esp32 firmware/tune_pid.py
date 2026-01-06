#!/usr/bin/env python3
"""
Simple PID tuning script for two-wheel robot
Allows manual tuning and testing
"""
import serial
import time
import sys
import threading

class PIDTuner:
    def __init__(self, port='/dev/cu.usbmodem212201', baud=9600):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2.5)

        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

        print("Connected to Arduino")
        time.sleep(0.5)

    def _read_loop(self):
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"  {line}")
            except:
                break
            time.sleep(0.01)

    def send(self, cmd):
        self.ser.write(f"{cmd}\n".encode('utf-8'))
        time.sleep(0.05)

    def set_gains(self, kp, ki, kd=0.0):
        """Set PID gains"""
        print(f"\nSetting gains: Kp={kp}, Ki={ki}, Kd={kd}")
        self.send(f"KP {kp}")
        time.sleep(0.1)
        self.send(f"KI {ki}")
        time.sleep(0.1)
        self.send(f"KD {kd}")
        time.sleep(0.1)

    def set_deadband(self, motor, direction, pwm):
        """Set deadband for motor/direction
        motor: 'L' or 'R'
        direction: 'F' or 'R' (forward/reverse)
        """
        print(f"\nSetting deadband: {motor} {direction} {pwm}")
        self.send(f"DB {motor} {direction} {pwm}")
        time.sleep(0.1)

    def test_velocity(self, vel_ms, duration=5):
        """Test velocity command"""
        print(f"\nTesting VG {vel_ms} for {duration} seconds...")
        self.send("ZERO")
        time.sleep(0.5)
        self.send(f"VG {vel_ms}")
        time.sleep(duration)
        self.send("STOP")
        time.sleep(0.5)

    def close(self):
        self.running = False
        if hasattr(self, 'read_thread'):
            self.read_thread.join(timeout=1)
        self.ser.close()


def interactive_tuning():
    """Interactive PID tuning session"""
    tuner = PIDTuner()

    print("\n" + "="*60)
    print("PID TUNING GUIDE")
    print("="*60)
    print("""
This script helps you tune the PID controller for velocity control.

TUNING PROCESS:
1. Start with Kp only (Ki=0, Kd=0)
2. Increase Kp until robot reaches target speed quickly
   - Too low: slow response
   - Too high: oscillation
   - Typical range: 0.05 - 0.2

3. Add Ki for steady-state error
   - Helps reach exact target velocity
   - Too high: overshoot and oscillation
   - Typical range: 0.005 - 0.05

4. Adjust deadband if needed
   - Default: 20 PWM
   - Increase if motors don't start smoothly
   - Typical range: 15 - 30

COMMANDS:
  g <kp> <ki> [kd]  - Set PID gains
  t <m/s>           - Test velocity
  d <motor> <dir> <pwm> - Set deadband (e.g., d L F 25)
  s                 - Stop motors
  q                 - Quit

EXAMPLE SESSION:
  > g 0.1 0          # Start with P only
  > t 0.3            # Test 0.3 m/s
  > g 0.15 0         # Increase Kp
  > t 0.3
  > g 0.15 0.01      # Add some I
  > t 0.3
""")
    print("="*60)

    try:
        while True:
            cmd = input("\n> ").strip().lower()

            if not cmd:
                continue

            parts = cmd.split()

            if parts[0] == 'q' or parts[0] == 'quit':
                break

            elif parts[0] == 'g' and len(parts) >= 3:
                kp = float(parts[1])
                ki = float(parts[2])
                kd = float(parts[3]) if len(parts) > 3 else 0.0
                tuner.set_gains(kp, ki, kd)

            elif parts[0] == 't' and len(parts) == 2:
                vel = float(parts[1])
                duration = 5
                tuner.test_velocity(vel, duration)

            elif parts[0] == 'd' and len(parts) == 4:
                motor = parts[1].upper()
                direction = parts[2].upper()
                pwm = float(parts[3])
                tuner.set_deadband(motor, direction, pwm)

            elif parts[0] == 's':
                tuner.send("STOP")

            else:
                print("Unknown command. Type 'q' to quit.")

    except KeyboardInterrupt:
        print("\nInterrupted")
    except EOFError:
        print("\nEOF")
    finally:
        tuner.send("STOP")
        tuner.close()


if __name__ == "__main__":
    interactive_tuning()
