#!/usr/bin/env python3
"""
Motor Calibration Script

Finds minimum start PWM for each motor to calibrate deadband compensation.
Different internal friction between motors causes asynchronous start - this
script determines per-motor deadband values to ensure synchronized movement.

Usage:
    python3 motor_calibration.py --host 192.168.68.52

Output:
    - Per-motor start velocities (m/s) and recommended deadband values
    - CSV file with raw calibration data
"""

import argparse
import time
import csv
from datetime import datetime
from robotlink import RobotLink, MessageType, OdomPayload

# Calibration parameters
VELOCITY_START = 0.0      # m/s - starting velocity
VELOCITY_END = 0.25       # m/s - maximum test velocity
VELOCITY_STEP = 0.01      # m/s - velocity increment
SETTLE_TIME = 0.8         # seconds - time to wait at each velocity
MIN_MOVEMENT_TICKS = 5    # minimum encoder ticks to consider "moving"

# Robot constants (from Arduino firmware)
WHEEL_DIAMETER = 0.082    # meters
TICKS_PER_REV = 360.0     # encoder ticks per revolution
METERS_PER_TICK = (3.14159 * WHEEL_DIAMETER) / TICKS_PER_REV


class MotorCalibrator:
    """Calibrate motor deadband values"""

    def __init__(self, host: str, port: int = 5000):
        self.host = host
        self.port = port
        self.robot = RobotLink(host=host, port=port)

        # Odometry data
        self.encoder_left = 0
        self.encoder_right = 0
        self.last_odom_time = 0

        # Calibration results
        self.results = []

        # Register callbacks
        self.robot.register_callback(MessageType.MSG_ODOM, self._on_odom)

    def _on_odom(self, payload: bytes):
        """Handle incoming odometry messages"""
        odom = OdomPayload.unpack(payload)

        # Accumulate encoder counts
        self.encoder_left += odom.delta_left
        self.encoder_right += odom.delta_right
        self.last_odom_time = time.time()

    def connect(self):
        """Connect to robot and initialize"""
        print(f"Connecting to robot at {self.host}:{self.port}...")

        # Enable odometry streaming at 10 Hz
        self.robot.enable_stream(True, 100)

        # Wait for first odometry message
        print("Waiting for odometry data...")
        start = time.time()
        while time.time() - start < 3.0:
            self.robot.process_messages(timeout=0.1)
            if self.last_odom_time > 0:
                print("✓ Connected and receiving data")
                return True

        print("✗ Failed to receive odometry data")
        return False

    def zero_encoders(self):
        """Reset encoder counts"""
        self.encoder_left = 0
        self.encoder_right = 0
        print("Encoders zeroed")

    def test_motor(self, motor: str, direction: str, velocity: float, duration: float) -> tuple:
        """
        Test a single motor at a specific velocity.

        Args:
            motor: "left" or "right"
            direction: "forward" or "reverse"
            velocity: Target velocity in m/s (absolute value)
            duration: Test duration in seconds

        Returns:
            (velocity_set, encoder_ticks_moved)
        """
        # Set velocity (positive for forward, negative for reverse)
        vel = velocity if direction == "forward" else -velocity

        if motor == "left":
            self.robot.set_velocity(vel, 0.0)
        else:
            self.robot.set_velocity(0.0, vel)

        # Zero encoders before test
        self.zero_encoders()

        # Wait and collect data
        start_time = time.time()
        while time.time() - start_time < duration:
            self.robot.process_messages(timeout=0.05)
            time.sleep(0.01)

        # Stop motor
        self.robot.stop()
        time.sleep(0.1)
        self.robot.process_messages(timeout=0.1)

        # Get final encoder count
        if motor == "left":
            ticks = abs(self.encoder_left)
        else:
            ticks = abs(self.encoder_right)

        return (velocity, ticks)

    def calibrate_motor(self, motor: str, direction: str):
        """
        Calibrate a single motor in one direction.

        Args:
            motor: "left" or "right"
            direction: "forward" or "reverse"
        """
        print(f"\n{'='*70}")
        print(f"Calibrating {motor.upper()} motor - {direction.upper()}")
        print(f"{'='*70}\n")

        velocity = VELOCITY_START
        start_velocity = None

        while velocity <= VELOCITY_END:
            print(f"Testing velocity: {velocity:.3f} m/s...", end=" ", flush=True)

            # Test motor
            vel_set, ticks = self.test_motor(motor, direction, velocity, SETTLE_TIME)

            # Record result
            result = {
                'motor': motor,
                'direction': direction,
                'velocity_mps': vel_set,
                'ticks': ticks,
                'distance_mm': ticks * METERS_PER_TICK * 1000,
                'moving': ticks >= MIN_MOVEMENT_TICKS
            }
            self.results.append(result)

            print(f"{ticks:4d} ticks ({result['distance_mm']:.1f}mm)", end="")

            # Check if motor started moving
            if ticks >= MIN_MOVEMENT_TICKS:
                if start_velocity is None:
                    start_velocity = velocity
                    print(f"  ← MOTOR STARTED!")
                else:
                    print(f"  ✓ moving")
            else:
                print(f"  (still)")

            velocity += VELOCITY_STEP

        # Calculate recommended deadband
        if start_velocity is not None:
            # Recommend slightly higher than start velocity for reliable operation
            recommended = start_velocity + VELOCITY_STEP
            print(f"\n→ First movement at: {start_velocity:.3f} m/s")
            print(f"→ Recommended minimum velocity: {recommended:.3f} m/s")

            # Convert to approximate PWM (very rough estimate)
            # Assuming full speed (0.2 m/s) ≈ PWM 100
            pwm_estimate = int((start_velocity / 0.2) * 100)
            print(f"→ Estimated start PWM: ~{pwm_estimate}")

            return (start_velocity, recommended, pwm_estimate)
        else:
            print(f"\n✗ Motor did not start moving (max tested: {VELOCITY_END:.3f} m/s)")
            return (None, None, None)

    def generate_report(self):
        """Generate calibration report"""
        print(f"\n{'='*70}")
        print("MOTOR CALIBRATION REPORT")
        print(f"{'='*70}\n")

        # Group results by motor and direction
        motors = {}
        for result in self.results:
            key = (result['motor'], result['direction'])
            if key not in motors:
                motors[key] = []
            motors[key].append(result)

        # Find start velocity for each motor/direction
        summary = {}
        for (motor, direction), data in motors.items():
            # Find first moving sample
            moving = [d for d in data if d['moving']]
            if moving:
                first_moving = moving[0]
                start_vel = first_moving['velocity_mps']
                recommended_vel = start_vel + VELOCITY_STEP
                pwm_est = int((start_vel / 0.2) * 100)

                summary[(motor, direction)] = {
                    'start_velocity': start_vel,
                    'recommended_velocity': recommended_vel,
                    'pwm_estimate': pwm_est
                }
            else:
                summary[(motor, direction)] = {
                    'start_velocity': None,
                    'recommended_velocity': None,
                    'pwm_estimate': None
                }

        # Print summary
        for motor in ['left', 'right']:
            print(f"{motor.upper()} MOTOR:")

            # Forward
            fwd = summary.get((motor, 'forward'))
            if fwd and fwd['start_velocity'] is not None:
                print(f"  Forward:")
                print(f"    Start velocity:       {fwd['start_velocity']:.3f} m/s")
                print(f"    Recommended velocity: {fwd['recommended_velocity']:.3f} m/s")
                print(f"    Estimated PWM:        ~{fwd['pwm_estimate']}")
            else:
                print(f"  Forward: Failed to start")

            # Reverse
            rev = summary.get((motor, 'reverse'))
            if rev and rev['start_velocity'] is not None:
                print(f"  Reverse:")
                print(f"    Start velocity:       {rev['start_velocity']:.3f} m/s")
                print(f"    Recommended velocity: {rev['recommended_velocity']:.3f} m/s")
                print(f"    Estimated PWM:        ~{rev['pwm_estimate']}")
            else:
                print(f"  Reverse: Failed to start")

            print()

        # Deadband recommendations for Arduino firmware
        print("RECOMMENDED ARDUINO DEADBAND VALUES:")
        print("(Add these to Arduino firmware after testing)")
        print()

        left_fwd = summary.get(('left', 'forward'))
        left_rev = summary.get(('left', 'reverse'))
        right_fwd = summary.get(('right', 'forward'))
        right_rev = summary.get(('right', 'reverse'))

        if all(x and x['pwm_estimate'] is not None for x in [left_fwd, left_rev, right_fwd, right_rev]):
            print(f"pidLeft.deadbandForward = {left_fwd['pwm_estimate'] + 5}.0f;")
            print(f"pidLeft.deadbandReverse = {left_rev['pwm_estimate'] + 5}.0f;")
            print(f"pidRight.deadbandForward = {right_fwd['pwm_estimate'] + 5}.0f;")
            print(f"pidRight.deadbandReverse = {right_rev['pwm_estimate'] + 5}.0f;")
        else:
            print("✗ Some motors failed to start - manual tuning required")

    def save_csv(self, filename: str = None):
        """Save calibration data to CSV"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"motor_calibration_{timestamp}.csv"

        with open(filename, 'w', newline='') as csvfile:
            fieldnames = ['motor', 'direction', 'velocity_mps', 'ticks', 'distance_mm', 'moving']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writeheader()
            for result in self.results:
                writer.writerow(result)

        print(f"\n→ Data saved to: {filename}")

    def run(self):
        """Run full calibration sequence"""
        print("\n" + "="*70)
        print("MOTOR CALIBRATION")
        print("="*70)
        print("\nThis script will test each motor individually to find the")
        print("minimum velocity/PWM required to overcome static friction.")
        print("\nWARNING: Ensure robot is on blocks or has free space to move!")
        print()

        input("Press ENTER to begin calibration...")

        # Connect to robot
        if not self.connect():
            print("\n✗ Failed to connect to robot")
            return False

        # Stop any motion
        self.robot.stop()
        time.sleep(0.5)

        # Calibrate each motor in each direction
        self.calibrate_motor('left', 'forward')
        time.sleep(1.0)

        self.calibrate_motor('left', 'reverse')
        time.sleep(1.0)

        self.calibrate_motor('right', 'forward')
        time.sleep(1.0)

        self.calibrate_motor('right', 'reverse')
        time.sleep(1.0)

        # Generate report
        self.generate_report()

        # Save data
        self.save_csv()

        # Stop robot
        self.robot.stop()
        self.robot.close()

        print("\n✓ Calibration complete!")
        return True


def main():
    parser = argparse.ArgumentParser(description='Calibrate motor deadband values')
    parser.add_argument('--host', type=str, default='192.168.68.52',
                        help='ESP32 IP address (default: 192.168.68.52)')
    parser.add_argument('--port', type=int, default=5000,
                        help='ESP32 UDP port (default: 5000)')

    args = parser.parse_args()

    calibrator = MotorCalibrator(host=args.host, port=args.port)
    try:
        calibrator.run()
    except KeyboardInterrupt:
        print("\n\n✗ Calibration interrupted by user")
        calibrator.robot.stop()
        calibrator.robot.close()
    except Exception as e:
        print(f"\n✗ Error during calibration: {e}")
        import traceback
        traceback.print_exc()
        calibrator.robot.stop()
        calibrator.robot.close()


if __name__ == '__main__':
    main()
