#!/usr/bin/env python3
"""Send command directly to ESP32 and monitor both ESP32 and Arduino"""

import socket
import struct
import time

ESP32_HOST = '192.168.68.52'
ESP32_PORT = 5000

SOF0, SOF1 = 0xAA, 0x55
MSG_SET_VEL = 0x10
MSG_STOP = 0x1A

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
    print(f"Sent: Type=0x{msg_type:02x} Len={len(payload)}")

print("Direct Command Test")
print("="*60)
print()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.5)

print("1. Sending SET_VEL command (0.25 m/s)...")
payload = struct.pack('<ff', 0.25, 0.25)
send_frame(sock, MSG_SET_VEL, payload)
print("   → Check ESP32 serial: Should see 'Host -> Arduino: SET_VEL'")
print("   → Check Arduino serial: Should see 'RX: Type=0x10' and 'SET_VEL: L=0.25 R=0.25'")
print()

time.sleep(3)

print("2. Sending STOP command...")
send_frame(sock, MSG_STOP)
print("   → Check ESP32 serial: Should see 'Host -> Arduino: STOP'")
print("   → Check Arduino serial: Should see 'RX: Type=0x1A' and 'STOP'")
print()

sock.close()
print("Test complete!")
print()
print("What did you see?")
print("- Did ESP32 show the forwarding messages?")
print("- Did Arduino receive the commands?")
