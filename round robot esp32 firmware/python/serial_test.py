#!/usr/bin/env python3
"""Quick test - Send CMD_VEL over serial and check if motors respond"""

import serial
import struct
import time

# Protocol Constants
SOF0 = 0xAA
SOF1 = 0x55

# Message types
MSG_ODOM = 0x01
MSG_CMD_VEL = 0x02
MSG_ENABLE_STREAM = 0x14
MSG_STOP = 0x15

def crc16(data: bytes) -> int:
    """Calculate CRC-16/CCITT-FALSE"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
            crc &= 0xFFFF
    return crc

def send_frame(ser, msg_type, payload=b''):
    """Send a RobotLink frame"""
    frame = bytes([SOF0, SOF1, msg_type, len(payload)]) + payload
    crc = crc16(frame[2:])  # CRC over TYPE+LEN+PAYLOAD
    frame += struct.pack('<H', crc)
    ser.write(frame)
    return len(frame)

def read_frame(ser, timeout=0.1):
    """Try to read a RobotLink frame"""
    ser.timeout = timeout

    # Look for SOF
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == SOF0:
            b2 = ser.read(1)
            if b2 and b2[0] == SOF1:
                break

    # Read header
    header = ser.read(2)
    if len(header) < 2:
        return None

    msg_type = header[0]
    length = header[1]

    # Read payload + CRC
    rest = ser.read(length + 2)
    if len(rest) < length + 2:
        return None

    payload = rest[:length]
    crc_rx = struct.unpack('<H', rest[length:length+2])[0]

    # Verify CRC
    crc_calc = crc16(bytes([msg_type, length]) + payload)
    if crc_rx != crc_calc:
        print(f"  ⚠ CRC mismatch: rx={crc_rx:04x} calc={crc_calc:04x}")
        return None

    return (msg_type, payload)

# Connect to ESP32
print('Serial Motor Test')
print('=' * 50)

ser = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=0.5)
time.sleep(2)  # Wait for ESP32 to boot

# Clear any startup messages
ser.reset_input_buffer()

print('\n1. Enabling odometry stream...')
send_frame(ser, MSG_ENABLE_STREAM, struct.pack('<BH', 1, 50))  # enable, 50ms interval
time.sleep(0.5)

# Collect some odometry
print('2. Waiting for odometry...')
odom_count = 0
pwm_values = []

for _ in range(60):  # 3 seconds
    frame = read_frame(ser, timeout=0.05)
    if frame:
        msg_type, payload = frame
        if msg_type == MSG_ODOM and len(payload) >= 24:
            # Unpack: encoder_left, encoder_right, vel_left, vel_right, pwm_left, pwm_right, timestamp
            vals = struct.unpack('<iiffhhI', payload)
            odom_count += 1
            pwm_values.append((vals[4], vals[5]))  # pwm_left, pwm_right
            if odom_count <= 3:
                print(f"  Odom #{odom_count}: Enc L={vals[0]} R={vals[1]}, PWM L={vals[4]} R={vals[5]}")

if odom_count == 0:
    print('✗ No odometry received - Arduino not responding')
    ser.close()
    exit(1)

print(f'✓ Receiving odometry ({odom_count} messages)')

# Send CMD_VEL
print('\n3. Sending CMD_VEL (0x02): 0.2 m/s forward...')
v_mm_s = int(0.2 * 1000)  # 200 mm/s
w_mrad_s = 0
payload = struct.pack('<hh', v_mm_s, w_mrad_s)
send_frame(ser, MSG_CMD_VEL, payload)
time.sleep(0.5)

# Monitor PWM response
pwm_values.clear()
for _ in range(60):  # 3 seconds
    frame = read_frame(ser, timeout=0.05)
    if frame:
        msg_type, payload = frame
        if msg_type == MSG_ODOM and len(payload) >= 24:
            vals = struct.unpack('<iiffhhI', payload)
            pwm_values.append((vals[4], vals[5]))

# Check PWM
max_pwm = max([abs(p[0]) for p in pwm_values] + [abs(p[1]) for p in pwm_values]) if pwm_values else 0

print('\n4. Results:')
if max_pwm > 0:
    print(f'✓ PWM changed! Max PWM = {max_pwm}')
    print('  → Motors are responding to CMD_VEL!')
else:
    print(f'✗ PWM stayed at 0')
    print('  → Motors NOT responding')

# Stop
send_frame(ser, MSG_STOP)
print('\nSent STOP command')

ser.close()
