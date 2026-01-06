#!/usr/bin/env python3
"""
Test script to capture all angle measurements from Neato Lidar
Shows the full 360-degree scan data
"""

import serial
import time
import sys

PORT = '/dev/cu.usbserial-0001'
BAUD = 115200

def main():
    print(f"Opening serial port {PORT}...")

    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)

        print("Connected! Sending 's' to start motor...\n")
        ser.write(b's')
        time.sleep(1)

        print("Collecting one full revolution of data...\n")
        print("=" * 80)

        # Collect data for 5 seconds to get multiple revolutions
        angles = {}
        start_time = time.time()

        while time.time() - start_time < 10:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                # Look for DATA lines
                if '[DATA]' in line and 'Angle:' in line:
                    try:
                        # Parse: [DATA] Angle: 180° | Dist: 82 mm | ...
                        parts = line.split('|')
                        angle_part = parts[0].split(':')[2].strip().replace('°', '')
                        dist_part = parts[1].split(':')[1].strip().replace('mm', '')

                        angle = int(angle_part)
                        dist = int(dist_part)

                        # Store the measurement
                        if angle not in angles or dist != 80:  # Prefer non-80mm readings
                            angles[angle] = dist

                    except:
                        pass

        # Print results
        print("\nMeasurements by angle (full 360°):")
        print("=" * 80)

        for angle in sorted(angles.keys()):
            dist = angles[angle]
            bar = '#' * (dist // 10)  # Visual bar
            print(f"{angle:3d}° : {dist:4d} mm | {bar}")

        print("\n" + "=" * 80)
        print(f"\nTotal unique angles measured: {len(angles)}")
        print(f"Distance range: {min(angles.values())} - {max(angles.values())} mm")

        # Check if all readings are the same
        unique_distances = set(angles.values())
        if len(unique_distances) == 1:
            print(f"\n⚠️  WARNING: All readings are {list(unique_distances)[0]} mm!")
            print("This suggests the Lidar may not be reading properly.")
        elif len(unique_distances) < 5:
            print(f"\n⚠️  WARNING: Only {len(unique_distances)} unique distance values!")
            print(f"Values: {sorted(unique_distances)}")
        else:
            print(f"\n✓ Found {len(unique_distances)} different distance values - looks good!")

        ser.close()

    except Exception as e:
        print(f"Error: {e}")
        return 1

    return 0

if __name__ == '__main__':
    sys.exit(main())
