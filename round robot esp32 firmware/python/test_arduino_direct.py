#!/usr/bin/env python3
"""Test Arduino directly via USB to verify CMD_VEL works"""

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
MSG_STOP = 0x1A

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
    print(f"TX: Type=0x{msg_type:02X} Len={len(payload)} Payload={payload.hex()}")
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

# Connect to Arduino directly via USB
print('Arduino Direct Motor Test')
print('=' * 50)

ser = serial.Serial('/dev/cu.usbmodem212201', 115200, timeout=0.5)
time.sleep(2)  # Wait for Arduino to boot

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
        if msg_type == MSG_ODOM:
            odom_count += 1
            # Parse odometry based on PROTOCOL.md (18 bytes)
            if len(payload) >= 18:
                vals = struct.unpack('<IhhIIh', payload)  # t_ms, dL, dR, x_mm, y_mm, th_mrad
                if odom_count <= 3:
                    print(f"  Odom #{odom_count}: t={vals[0]}ms dL={vals[1]} dR={vals[2]}")

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

# Monitor for response
print('4. Monitoring for motor response...')
for _ in range(40):  # 2 seconds
    frame = read_frame(ser, timeout=0.05)
    if frame:
        msg_type, payload = frame
        print(f"  RX: Type=0x{msg_type:02X} Len={len(payload)}")

# Stop
print('\n5. Sending STOP command...')
send_frame(ser, MSG_STOP)
time.sleep(0.5)

print('\nTest complete!')
ser.close()
