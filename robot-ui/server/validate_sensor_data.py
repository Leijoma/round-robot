#!/usr/bin/env python3
"""
Sensor Data Validation Script

Collects and analyzes sensor data from the robot to verify:
- Odometry frequency (target: 10 Hz ±0.5 Hz)
- LIDAR scan frequency (target: 4 Hz ±0.2 Hz)
- Timestamp jitter and latency
- Encoder counts per sample at different speeds
- Stillness noise measurement

Usage:
    python3 validate_sensor_data.py --host 192.168.68.52 --duration 60 --stillness-test
    python3 validate_sensor_data.py --help
"""

import sys
import os
import time
import argparse
import numpy as np
from datetime import datetime
from collections import defaultdict
from typing import List, Dict, Tuple, Optional

# Add current directory to path for robotlink import
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robotlink import RobotLink, MessageType, OdomPayload, LidarScanPayload


class SensorValidator:
    """Validates sensor data quality and timing"""

    def __init__(self, esp32_host: str, esp32_port: int = 5000):
        """Initialize validator and connect to ESP32

        Args:
            esp32_host: ESP32 IP address
            esp32_port: ESP32 UDP port
        """
        self.host = esp32_host
        self.port = esp32_port
        self.robot: Optional[RobotLink] = None

        # Data storage
        self.odom_data: List[Dict] = []
        self.lidar_data: List[Dict] = []

        # Timing data
        self.odom_timestamps: List[float] = []
        self.lidar_timestamps: List[float] = []

        # Encoder data
        self.encoder_deltas_left: List[int] = []
        self.encoder_deltas_right: List[int] = []

        # Start time (wall clock)
        self.start_time: Optional[float] = None

        print(f"Connecting to ESP32 at {esp32_host}:{esp32_port}...")

    def connect(self) -> bool:
        """Connect to robot and enable data streaming

        Returns:
            True if connection successful
        """
        try:
            self.robot = RobotLink(host=self.host, port=self.port)
            time.sleep(0.5)

            # Enable odometry at 10 Hz
            self.robot.enable_stream(True, interval_ms=100)

            # Enable LIDAR
            self.robot.lidar_enable(True)
            self.robot.lidar_set_rpm(240)

            print("✓ Connected to ESP32")
            print("✓ Odometry streaming enabled (10 Hz)")
            print("✓ LIDAR enabled (240 RPM)")

            return True

        except Exception as e:
            print(f"✗ Connection failed: {e}")
            return False

    def collect_data(self, duration_sec: int, verbose: bool = True) -> bool:
        """Collect sensor data for specified duration

        Args:
            duration_sec: How long to collect data (seconds)
            verbose: Print progress updates

        Returns:
            True if data collection successful
        """
        if not self.robot:
            print("✗ Not connected to robot")
            return False

        print(f"\nCollecting data for {duration_sec} seconds...")
        print("Press Ctrl+C to stop early\n")

        self.start_time = time.time()
        last_print = time.time()

        try:
            while (time.time() - self.start_time) < duration_sec:
                # Poll for messages
                result = self.robot.receive_frame()

                if result is None:
                    time.sleep(0.001)  # Prevent CPU spinning
                    continue

                msg_type, payload = result
                wall_time = time.time()

                # Handle odometry messages
                if msg_type == MessageType.MSG_ODOM:
                    try:
                        odom = OdomPayload.unpack(payload)

                        self.odom_data.append({
                            'wall_time': wall_time,
                            'timestamp': odom.timestamp,
                            'delta_left': odom.delta_left,
                            'delta_right': odom.delta_right,
                            'x_mm': odom.x_mm,
                            'y_mm': odom.y_mm,
                            'theta_mrad': odom.theta_mrad
                        })

                        self.odom_timestamps.append(odom.timestamp)
                        self.encoder_deltas_left.append(odom.delta_left)
                        self.encoder_deltas_right.append(odom.delta_right)

                    except Exception as e:
                        print(f"Error parsing odometry: {e}")

                # Handle LIDAR scan messages
                elif msg_type == MessageType.MSG_LIDAR_SCAN:
                    try:
                        lidar_scan = LidarScanPayload.unpack(payload)

                        self.lidar_data.append({
                            'wall_time': wall_time,
                            'timestamp': lidar_scan.timestamp,
                            'rpm': lidar_scan.rpm,
                            'num_readings': len(lidar_scan.readings),
                            'start_angle': lidar_scan.start_angle
                        })

                        self.lidar_timestamps.append(lidar_scan.timestamp)

                    except Exception as e:
                        print(f"Error parsing LIDAR: {e}")

                # Print progress every 5 seconds
                if verbose and (time.time() - last_print) >= 5.0:
                    elapsed = time.time() - self.start_time
                    print(f"  {elapsed:.0f}s: {len(self.odom_data)} odom samples, "
                          f"{len(self.lidar_data)} LIDAR scans")
                    last_print = time.time()

            elapsed = time.time() - self.start_time
            print(f"\n✓ Data collection complete ({elapsed:.1f} seconds)")
            print(f"  Odometry samples: {len(self.odom_data)}")
            print(f"  LIDAR scans: {len(self.lidar_data)}")

            return True

        except KeyboardInterrupt:
            elapsed = time.time() - self.start_time
            print(f"\n\nData collection stopped by user ({elapsed:.1f} seconds)")
            print(f"  Odometry samples: {len(self.odom_data)}")
            print(f"  LIDAR scans: {len(self.lidar_data)}")
            return True

        except Exception as e:
            print(f"\n✗ Error during data collection: {e}")
            import traceback
            traceback.print_exc()
            return False

    def analyze_timing(self) -> Dict:
        """Analyze timing characteristics of sensor data

        Returns:
            Dictionary with timing statistics
        """
        results = {}

        # Odometry timing analysis
        if len(self.odom_timestamps) > 1:
            # Calculate intervals between samples (in milliseconds)
            odom_intervals = np.diff(self.odom_timestamps)

            # Calculate rate (Hz)
            odom_rates = 1000.0 / odom_intervals  # Convert ms to Hz

            results['odom'] = {
                'samples': len(self.odom_timestamps),
                'rate_mean_hz': np.mean(odom_rates),
                'rate_std_hz': np.std(odom_rates),
                'rate_min_hz': np.min(odom_rates),
                'rate_max_hz': np.max(odom_rates),
                'interval_mean_ms': np.mean(odom_intervals),
                'interval_std_ms': np.std(odom_intervals),
                'jitter_ms': np.std(odom_intervals)
            }
        else:
            results['odom'] = {'error': 'Insufficient data'}

        # LIDAR timing analysis
        if len(self.lidar_timestamps) > 1:
            lidar_intervals = np.diff(self.lidar_timestamps)
            lidar_rates = 1000.0 / lidar_intervals

            results['lidar'] = {
                'scans': len(self.lidar_timestamps),
                'rate_mean_hz': np.mean(lidar_rates),
                'rate_std_hz': np.std(lidar_rates),
                'rate_min_hz': np.min(lidar_rates),
                'rate_max_hz': np.max(lidar_rates),
                'interval_mean_ms': np.mean(lidar_intervals),
                'interval_std_ms': np.std(lidar_intervals),
                'jitter_ms': np.std(lidar_intervals)
            }

            # Calculate actual RPM from LIDAR data
            if self.lidar_data:
                rpms = [scan['rpm'] for scan in self.lidar_data]
                results['lidar']['rpm_mean'] = np.mean(rpms)
                results['lidar']['rpm_std'] = np.std(rpms)
        else:
            results['lidar'] = {'error': 'Insufficient data'}

        return results

    def analyze_encoder_data(self) -> Dict:
        """Analyze encoder statistics

        Returns:
            Dictionary with encoder statistics
        """
        results = {}

        if len(self.encoder_deltas_left) > 0:
            # Absolute values (for speed calculation)
            abs_left = np.abs(self.encoder_deltas_left)
            abs_right = np.abs(self.encoder_deltas_right)

            results['left'] = {
                'mean_ticks_per_sample': np.mean(abs_left),
                'std_ticks_per_sample': np.std(abs_left),
                'max_ticks_per_sample': np.max(abs_left),
                'total_ticks': np.sum(self.encoder_deltas_left)
            }

            results['right'] = {
                'mean_ticks_per_sample': np.mean(abs_right),
                'std_ticks_per_sample': np.std(abs_right),
                'max_ticks_per_sample': np.max(abs_right),
                'total_ticks': np.sum(self.encoder_deltas_right)
            }

            # Calculate average speed (m/s)
            # 360 ticks/rev, wheel diameter 82mm
            METERS_PER_TICK = (np.pi * 0.082) / 360.0
            SAMPLE_INTERVAL_SEC = 0.1  # 10 Hz

            avg_speed_left = results['left']['mean_ticks_per_sample'] * METERS_PER_TICK / SAMPLE_INTERVAL_SEC
            avg_speed_right = results['right']['mean_ticks_per_sample'] * METERS_PER_TICK / SAMPLE_INTERVAL_SEC

            results['avg_speed_left_ms'] = avg_speed_left
            results['avg_speed_right_ms'] = avg_speed_right
            results['avg_speed_ms'] = (avg_speed_left + avg_speed_right) / 2.0

        else:
            results['error'] = 'No encoder data'

        return results

    def analyze_stillness_noise(self, duration_sec: int = 10) -> Dict:
        """Measure encoder noise when robot is stationary

        Args:
            duration_sec: How long to measure (seconds)

        Returns:
            Dictionary with stillness noise statistics
        """
        print(f"\n{'='*60}")
        print("STILLNESS NOISE TEST")
        print(f"{'='*60}")
        print("Ensure robot is STATIONARY (not moving)")
        print(f"Collecting data for {duration_sec} seconds...")

        # Clear previous data
        self.encoder_deltas_left.clear()
        self.encoder_deltas_right.clear()

        # Collect data
        if not self.collect_data(duration_sec, verbose=False):
            return {'error': 'Data collection failed'}

        # Analyze noise
        results = {
            'duration_sec': duration_sec,
            'samples': len(self.encoder_deltas_left),
            'left_total_drift': sum(self.encoder_deltas_left),
            'right_total_drift': sum(self.encoder_deltas_right),
            'left_max_single_tick': max(abs(d) for d in self.encoder_deltas_left) if self.encoder_deltas_left else 0,
            'right_max_single_tick': max(abs(d) for d in self.encoder_deltas_right) if self.encoder_deltas_right else 0,
        }

        # Determine noise level
        max_drift = max(abs(results['left_total_drift']), abs(results['right_total_drift']))

        if max_drift < 5:
            results['assessment'] = 'EXCELLENT'
        elif max_drift < 10:
            results['assessment'] = 'GOOD'
        elif max_drift < 20:
            results['assessment'] = 'ACCEPTABLE'
        else:
            results['assessment'] = 'POOR'

        return results

    def generate_report(self, timing_results: Dict, encoder_results: Dict,
                       stillness_results: Optional[Dict] = None) -> str:
        """Generate human-readable validation report

        Args:
            timing_results: Output from analyze_timing()
            encoder_results: Output from analyze_encoder_data()
            stillness_results: Optional output from analyze_stillness_noise()

        Returns:
            Report as string
        """
        report = []
        report.append("=" * 70)
        report.append("SENSOR DATA VALIDATION REPORT")
        report.append("=" * 70)
        report.append(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        report.append(f"ESP32: {self.host}:{self.port}")

        if self.start_time:
            elapsed = time.time() - self.start_time
            report.append(f"Test Duration: {elapsed:.1f} seconds")

        report.append("")

        # Odometry section
        report.append("ODOMETRY:")
        if 'odom' in timing_results and 'error' not in timing_results['odom']:
            odom = timing_results['odom']
            report.append(f"  Samples:  {odom['samples']}")
            report.append(f"  Rate:     {odom['rate_mean_hz']:.2f} Hz (±{odom['rate_std_hz']:.2f} Hz)")
            report.append(f"            Min: {odom['rate_min_hz']:.2f} Hz, Max: {odom['rate_max_hz']:.2f} Hz")
            report.append(f"  Interval: {odom['interval_mean_ms']:.1f} ms (±{odom['interval_std_ms']:.1f} ms)")
            report.append(f"  Jitter:   {odom['jitter_ms']:.2f} ms")

            # Check if within spec
            if 9.5 <= odom['rate_mean_hz'] <= 10.5:
                report.append("  Status:   ✓ WITHIN SPEC (10 Hz ±0.5 Hz)")
            else:
                report.append(f"  Status:   ✗ OUT OF SPEC (expected 10 Hz ±0.5 Hz)")
        else:
            report.append("  ✗ Insufficient data")

        report.append("")

        # LIDAR section
        report.append("LIDAR:")
        if 'lidar' in timing_results and 'error' not in timing_results['lidar']:
            lidar = timing_results['lidar']
            report.append(f"  Scans:    {lidar['scans']}")
            report.append(f"  Rate:     {lidar['rate_mean_hz']:.2f} Hz (±{lidar['rate_std_hz']:.2f} Hz)")
            report.append(f"            Min: {lidar['rate_min_hz']:.2f} Hz, Max: {lidar['rate_max_hz']:.2f} Hz")
            report.append(f"  Interval: {lidar['interval_mean_ms']:.1f} ms (±{lidar['interval_std_ms']:.1f} ms)")
            report.append(f"  RPM:      {lidar['rpm_mean']:.1f} RPM (±{lidar['rpm_std']:.1f} RPM)")

            # Check if within spec
            if 3.8 <= lidar['rate_mean_hz'] <= 4.2:
                report.append("  Status:   ✓ WITHIN SPEC (4 Hz ±0.2 Hz)")
            else:
                report.append(f"  Status:   ✗ OUT OF SPEC (expected 4 Hz ±0.2 Hz)")
        else:
            report.append("  ✗ Insufficient data")

        report.append("")

        # Encoder data section
        report.append("ENCODER DATA:")
        if 'error' not in encoder_results:
            report.append(f"  Left wheel:")
            report.append(f"    Avg ticks/sample: {encoder_results['left']['mean_ticks_per_sample']:.1f} "
                         f"(±{encoder_results['left']['std_ticks_per_sample']:.1f})")
            report.append(f"    Max ticks/sample: {encoder_results['left']['max_ticks_per_sample']}")
            report.append(f"    Total ticks:      {encoder_results['left']['total_ticks']}")

            report.append(f"  Right wheel:")
            report.append(f"    Avg ticks/sample: {encoder_results['right']['mean_ticks_per_sample']:.1f} "
                         f"(±{encoder_results['right']['std_ticks_per_sample']:.1f})")
            report.append(f"    Max ticks/sample: {encoder_results['right']['max_ticks_per_sample']}")
            report.append(f"    Total ticks:      {encoder_results['right']['total_ticks']}")

            report.append(f"  Average speed:    {encoder_results['avg_speed_ms']:.3f} m/s")
            report.append(f"    Left:  {encoder_results['avg_speed_left_ms']:.3f} m/s")
            report.append(f"    Right: {encoder_results['avg_speed_right_ms']:.3f} m/s")

            # Check encoder resolution at target speed (0.15-0.2 m/s)
            if encoder_results['avg_speed_ms'] > 0.01:  # Only if moving
                avg_ticks = (encoder_results['left']['mean_ticks_per_sample'] +
                           encoder_results['right']['mean_ticks_per_sample']) / 2.0

                if 15 <= avg_ticks <= 35:
                    report.append(f"  Resolution:       ✓ GOOD ({avg_ticks:.1f} ticks/sample at 10 Hz)")
                else:
                    report.append(f"  Resolution:       ⚠ Suboptimal ({avg_ticks:.1f} ticks/sample)")
        else:
            report.append("  ✗ No encoder data")

        report.append("")

        # Stillness test section
        if stillness_results:
            report.append("STILLNESS TEST:")
            if 'error' not in stillness_results:
                report.append(f"  Duration:         {stillness_results['duration_sec']} seconds")
                report.append(f"  Samples:          {stillness_results['samples']}")
                report.append(f"  Left drift:       {stillness_results['left_total_drift']} ticks")
                report.append(f"  Right drift:      {stillness_results['right_total_drift']} ticks")
                report.append(f"  Max single tick:  L={stillness_results['left_max_single_tick']}, "
                             f"R={stillness_results['right_max_single_tick']}")
                report.append(f"  Assessment:       {stillness_results['assessment']}")

                if stillness_results['assessment'] in ['EXCELLENT', 'GOOD']:
                    report.append("  Status:           ✓ ACCEPTABLE NOISE LEVEL")
                else:
                    report.append("  Status:           ⚠ HIGH NOISE LEVEL")
            else:
                report.append("  ✗ Test failed")

            report.append("")

        report.append("=" * 70)

        return "\n".join(report)

    def save_csv(self, filename: str) -> bool:
        """Save raw data to CSV file

        Args:
            filename: Output CSV filename

        Returns:
            True if save successful
        """
        try:
            import csv

            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)

                # Write odometry data
                writer.writerow(['ODOMETRY'])
                writer.writerow(['wall_time', 'timestamp', 'delta_left', 'delta_right',
                               'x_mm', 'y_mm', 'theta_mrad'])

                for odom in self.odom_data:
                    writer.writerow([
                        odom['wall_time'],
                        odom['timestamp'],
                        odom['delta_left'],
                        odom['delta_right'],
                        odom['x_mm'],
                        odom['y_mm'],
                        odom['theta_mrad']
                    ])

                # Write LIDAR data
                writer.writerow([])
                writer.writerow(['LIDAR'])
                writer.writerow(['wall_time', 'timestamp', 'rpm', 'num_readings', 'start_angle'])

                for lidar in self.lidar_data:
                    writer.writerow([
                        lidar['wall_time'],
                        lidar['timestamp'],
                        lidar['rpm'],
                        lidar['num_readings'],
                        lidar['start_angle']
                    ])

            print(f"\n✓ Data saved to: {filename}")
            return True

        except Exception as e:
            print(f"\n✗ Failed to save CSV: {e}")
            return False

    def close(self):
        """Clean up connection"""
        if self.robot:
            self.robot.close()


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='Validate robot sensor data quality and timing',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic validation test (60 seconds)
  python3 validate_sensor_data.py --host 192.168.68.52 --duration 60

  # With stillness test
  python3 validate_sensor_data.py --host 192.168.68.52 --duration 60 --stillness-test

  # Save data to CSV
  python3 validate_sensor_data.py --host 192.168.68.52 --duration 30 --csv my_test.csv
        """
    )

    parser.add_argument('--host', default='192.168.68.52',
                       help='ESP32 IP address (default: 192.168.68.52)')
    parser.add_argument('--port', type=int, default=5000,
                       help='ESP32 UDP port (default: 5000)')
    parser.add_argument('--duration', type=int, default=60,
                       help='Data collection duration in seconds (default: 60)')
    parser.add_argument('--stillness-test', action='store_true',
                       help='Run stillness noise test (10 seconds)')
    parser.add_argument('--stillness-duration', type=int, default=10,
                       help='Stillness test duration in seconds (default: 10)')
    parser.add_argument('--csv', type=str,
                       help='Save raw data to CSV file')

    args = parser.parse_args()

    # Create validator
    validator = SensorValidator(args.host, args.port)

    # Connect
    if not validator.connect():
        return 1

    # Collect data
    print("\n" + "="*60)
    print("MAIN DATA COLLECTION")
    print("="*60)

    if not validator.collect_data(args.duration):
        validator.close()
        return 1

    # Analyze timing
    print("\nAnalyzing data...")
    timing_results = validator.analyze_timing()
    encoder_results = validator.analyze_encoder_data()

    # Stillness test (optional)
    stillness_results = None
    if args.stillness_test:
        stillness_results = validator.analyze_stillness_noise(args.stillness_duration)

    # Generate and print report
    report = validator.generate_report(timing_results, encoder_results, stillness_results)
    print("\n" + report)

    # Save CSV (optional)
    if args.csv:
        filename = args.csv
    else:
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        filename = f'sensor_validation_{timestamp}.csv'

    validator.save_csv(filename)

    # Clean up
    validator.close()

    return 0


if __name__ == '__main__':
    sys.exit(main())
