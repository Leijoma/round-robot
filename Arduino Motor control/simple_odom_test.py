#!/usr/bin/env python3
"""
Simple ODOM test - just wait and listen for ODOM messages
"""

import socket
import struct
import time

ESP32_HOST = '192.168.68.52'
ESP32_PORT = 5000

SOF0, SOF1 = 0xAA, 0x55
MSG_ODOM = 0x01

def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc

print("Connecting to ESP32...")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.5)

# Send ping to establish connection
frame = bytes([SOF0, SOF1, 0x7E, 0])  # PING
frame += struct.pack('<H', crc16(frame[2:]))
sock.sendto(frame, (ESP32_HOST, ESP32_PORT))

print("Waiting for ODOM messages for 30 seconds...")
print("(ESP32 should auto-enable streaming at 200ms = 5 Hz)")
print()

start = time.time()
odom_count = 0
last_print = start

while time.time() - start < 30:
    try:
        data, addr = sock.recvfrom(1024)

        if len(data) >= 6 and data[0] == SOF0 and data[1] == SOF1:
            msg_type = data[2]
            payload_len = data[3]

            if msg_type == MSG_ODOM:
                odom_count += 1

                # Parse ODOM payload
                payload = data[4:4+payload_len]
                if len(payload) >= 18:
                    timestamp, delta_l, delta_r, x, y, theta = struct.unpack('<Ihhiih', payload[:18])

                    # Print every 5th message
                    if odom_count % 5 == 1:
                        print(f"ODOM #{odom_count:4d}: delta_L={delta_l:+4d} delta_R={delta_r:+4d} "
                              f"x={x:6d}mm y={y:6d}mm θ={theta:5d}mrad t={timestamp}ms")

            # Print summary every 5 seconds
            if time.time() - last_print >= 5:
                print(f"\n[{int(time.time() - start)}s] Received {odom_count} ODOM messages ({odom_count/5:.1f} Hz)\n")
                last_print = time.time()

    except socket.timeout:
        pass

print(f"\nTotal ODOM messages received: {odom_count}")
if odom_count > 0:
    print(f"Average rate: {odom_count / 30:.1f} Hz")
    print("✅ ODOM streaming is working!")
else:
    print("❌ NO ODOM DATA - Something is wrong!")

sock.close()
