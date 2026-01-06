#!/usr/bin/env python3
"""Quick motor test after fixing inversion"""

import socket
import struct
import time

ESP32_HOST = '192.168.68.52'
ESP32_PORT = 5000

SOF0, SOF1 = 0xAA, 0x55
MSG_ODOM = 0x01
MSG_SET_VEL = 0x10
MSG_STOP = 0x1A

WHEEL_DIAMETER = 0.082
TICKS_PER_REV = 360
DT = 0.2

def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc

def send_frame(sock, msg_type, payload=b''):
    frame = bytes([SOF0, SOF1, msg_type, len(payload)]) + payload
    frame += struct.pack('<H', crc16(frame[2:]))
    sock.sendto(frame, (ESP32_HOST, ESP32_PORT))

print("Connecting...")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.5)

# Ping
send_frame(sock, 0x7E)
time.sleep(2)

print("\n🧪 Testing velocities with FIXED motor inversion:")
print("="*60)

test_vels = [0.20, 0.30, 0.45]

for target_vel in test_vels:
    print(f"\n🎯 Testing {target_vel} m/s...")

    # Send velocity
    payload = struct.pack('<ff', target_vel, target_vel)
    send_frame(sock, MSG_SET_VEL, payload)

    # Collect samples
    samples_left = []
    samples_right = []
    start = time.time()

    while time.time() - start < 3.0:
        try:
            data, _ = sock.recvfrom(1024)
            if len(data) >= 6 and data[0] == SOF0 and data[1] == SOF1:
                if data[2] == MSG_ODOM and data[3] >= 18:
                    payload = data[4:4+data[3]]
                    timestamp, delta_l, delta_r, x, y, theta = struct.unpack('<Ihhiih', payload[:18])

                    # Calculate velocity
                    circumference = WHEEL_DIAMETER * 3.14159
                    vel_l = (delta_l / TICKS_PER_REV) * circumference / DT
                    vel_r = (delta_r / TICKS_PER_REV) * circumference / DT

                    samples_left.append(vel_l)
                    samples_right.append(vel_r)
        except socket.timeout:
            pass

    # Stop
    send_frame(sock, MSG_STOP)
    time.sleep(0.5)

    if samples_left:
        avg_l = sum(samples_left) / len(samples_left)
        avg_r = sum(samples_right) / len(samples_right)
        avg = (avg_l + avg_r) / 2

        error_pct = abs(avg - target_vel) / target_vel * 100

        # Check if both moving same direction
        same_dir = (avg_l * avg_r) > 0  # Both positive or both negative

        print(f"  Left:  {avg_l:+.3f} m/s")
        print(f"  Right: {avg_r:+.3f} m/s")
        print(f"  Avg:   {avg:+.3f} m/s (target: {target_vel:.2f})")
        print(f"  Error: {error_pct:.1f}%")

        if same_dir:
            if abs(avg_l) > 0.01 and abs(avg_r) > 0.01:
                print(f"  ✅ Both motors moving SAME direction!")
            else:
                print(f"  ⚠️  Motors barely moving")
        else:
            print(f"  ❌ Motors moving OPPOSITE directions!")
    else:
        print("  ❌ No data received")

print("\n" + "="*60)
sock.close()
