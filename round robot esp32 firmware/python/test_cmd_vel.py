#!/usr/bin/env python3
"""Test CMD_VEL - check if encoder ticks change when motors run"""

import serial, struct, time

def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc

def send_frame(ser, msg_type, payload=b''):
    frame = bytes([0xAA, 0x55, msg_type, len(payload)]) + payload
    crc = crc16(frame[2:])
    ser.write(frame + struct.pack('<H', crc))

def read_frame(ser, timeout=0.1):
    ser.timeout = timeout
    while True:
        b = ser.read(1)
        if not b: return None
        if b[0] == 0xAA:
            b2 = ser.read(1)
            if b2 and b2[0] == 0x55: break
    header = ser.read(2)
    if len(header) < 2: return None
    msg_type, length = header[0], header[1]
    rest = ser.read(length + 2)
    if len(rest) < length + 2: return None
    return (msg_type, rest[:length])

print('CMD_VEL Motor Test')
print('=' * 60)

ser = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=1)
time.sleep(1)
ser.reset_input_buffer()

# Enable odometry stream
print('\n1. Enabling odometry stream...')
send_frame(ser, 0x14, struct.pack('<BH', 1, 50))  # ENABLE_STREAM
time.sleep(0.3)

# Collect baseline odometry (motors stopped)
print('2. Collecting baseline (motors stopped)...')
baseline_ticks = []
for _ in range(20):  # 1 second
    f = read_frame(ser, 0.06)
    if f and f[0] == 0x01 and len(f[1]) == 18:
        vals = struct.unpack('<IhhIIh', f[1])
        baseline_ticks.append((vals[1], vals[2]))  # dL, dR

total_baseline = sum(abs(dL) + abs(dR) for dL, dR in baseline_ticks)
print(f'  Baseline total ticks: {total_baseline}')

# Send CMD_VEL
print('\n3. Sending CMD_VEL (0x02): 0.3 m/s forward...')
v_mm_s = int(0.3 * 1000)  # 300 mm/s
w_mrad_s = 0
send_frame(ser, 0x02, struct.pack('<hh', v_mm_s, w_mrad_s))  # CMD_VEL
time.sleep(0.1)

# Monitor encoder changes
print('4. Monitoring encoders for 3 seconds...')
test_ticks = []
max_tick_seen = 0
for i in range(60):  # 3 seconds
    f = read_frame(ser, 0.06)
    if f and f[0] == 0x01 and len(f[1]) == 18:
        vals = struct.unpack('<IhhIIh', f[1])
        dL, dR = vals[1], vals[2]
        test_ticks.append((dL, dR))
        tick_sum = abs(dL) + abs(dR)
        if tick_sum > max_tick_seen:
            max_tick_seen = tick_sum
            if tick_sum > 0:
                print(f'  #{i+1}: dL={dL:4d} dR={dR:4d} ← MOTION DETECTED!')

total_test = sum(abs(dL) + abs(dR) for dL, dR in test_ticks)
print(f'\nTotal encoder ticks during test: {total_test}')

# Stop motors
print('\n5. Sending STOP...')
send_frame(ser, 0x1A)  # STOP
time.sleep(0.5)

# Results
print('\n' + '=' * 60)
print('RESULTS:')
print('=' * 60)
if total_test > total_baseline + 10:  # threshold for noise
    print(f'✓ SUCCESS! Encoders changed: {total_test} ticks')
    print('  → Motors ARE responding to CMD_VEL!')
    print('  → RobotLink motor control is WORKING!')
else:
    print(f'✗ FAILED: Encoders did not change significantly')
    print(f'  Baseline: {total_baseline} ticks, Test: {total_test} ticks')
    print('  → Motors NOT responding to CMD_VEL')

ser.close()
