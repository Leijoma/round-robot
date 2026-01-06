#!/usr/bin/env python3
import serial, struct, time

def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc

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

ser = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=1)
time.sleep(1)
ser.reset_input_buffer()

# Enable stream
frame = bytes([0xAA, 0x55, 0x14, 0x03]) + struct.pack('<BH', 1, 50)
crc = crc16(frame[2:])
ser.write(frame + struct.pack('<H', crc))
time.sleep(0.2)

print('Listening for odometry...')
for i in range(10):
    f = read_frame(ser, 0.1)
    if f:
        print(f'Frame #{i+1}: Type=0x{f[0]:02X} Len={len(f[1])} Payload={f[1].hex()}')
        if f[0] == 0x01:  # ODOM
            if len(f[1]) == 18:
                vals = struct.unpack('<IhhIIh', f[1])
                print(f'  → PROTOCOL.md format: t={vals[0]}ms dL={vals[1]} dR={vals[2]} x={vals[3]}mm y={vals[4]}mm th={vals[5]}mrad')
            elif len(f[1]) >= 24:
                vals = struct.unpack('<iiffhhI', f[1])
                print(f'  → Extended format: Enc L={vals[0]} R={vals[1]} PWM L={vals[4]} R={vals[5]}')

ser.close()
