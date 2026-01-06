#!/usr/bin/env python3
"""
Communication Debugger - Shows exactly what's happening between Host <-> ESP32 <-> Arduino
"""

import socket
import struct
import time
import serial
import threading

# Configuration
ESP32_HOST = '192.168.68.52'
ESP32_PORT = 5000
ARDUINO_DEBUG_PORT = '/dev/tty.usbmodem212201'
ESP32_DEBUG_PORT = '/dev/cu.usbserial-0001'  # ESP32 USB serial

# Protocol
SOF0, SOF1 = 0xAA, 0x55
MSG_ODOM = 0x01
MSG_SET_VEL = 0x10
MSG_ENABLE_STREAM = 0x14
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
    return frame


def monitor_serial(port, baud, name, color):
    """Monitor a serial port and print output with color"""
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2)  # Wait for reset
        print(f"\033[{color}m✓ {name} connected on {port}\033[0m")

        while True:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # Filter out TX ODOM spam
                    if "TX ODOM" not in line:
                        print(f"\033[{color}m[{name}]\033[0m {line}")
            time.sleep(0.01)
    except Exception as e:
        print(f"\033[{color}m⚠️  {name} not available: {e}\033[0m")


def main():
    print("\n" + "="*80)
    print("COMMUNICATION DEBUGGER")
    print("="*80)
    print("This will show EXACTLY what's happening in the communication chain:")
    print("  HOST (this script) -> ESP32 (UDP) -> Arduino (Serial)")
    print("="*80 + "\n")

    # Start serial monitors
    arduino_thread = threading.Thread(
        target=monitor_serial,
        args=(ARDUINO_DEBUG_PORT, 115200, "Arduino", "32"),  # Green
        daemon=True
    )
    arduino_thread.start()

    esp32_thread = threading.Thread(
        target=monitor_serial,
        args=(ESP32_DEBUG_PORT, 115200, "ESP32  ", "36"),  # Cyan
        daemon=True
    )
    esp32_thread.start()

    time.sleep(3)

    # Create UDP socket
    print("\033[33m[HOST  ]\033[0m Creating UDP socket...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)
    print(f"\033[33m[HOST  ]\033[0m Connected to ESP32 at {ESP32_HOST}:{ESP32_PORT}\n")

    try:
        # Enable streaming
        print("\033[33m[HOST  ]\033[0m → Sending ENABLE_STREAM...")
        payload = struct.pack('<BH', 1, 50)  # enable=1, interval=50ms
        send_frame(sock, MSG_ENABLE_STREAM, payload)
        time.sleep(2)

        # Listen for ODOM messages
        print("\033[33m[HOST  ]\033[0m Listening for ODOM data for 5 seconds...")
        odom_count = 0
        start = time.time()

        while time.time() - start < 5:
            try:
                data, addr = sock.recvfrom(1024)
                if len(data) >= 6 and data[0] == SOF0 and data[1] == SOF1:
                    msg_type = data[2]
                    payload_len = data[3]

                    if msg_type == MSG_ODOM:
                        odom_count += 1
                        if odom_count % 10 == 1:  # Print every 10th
                            print(f"\033[33m[HOST  ]\033[0m ← Received ODOM #{odom_count} (payload len={payload_len})")
            except socket.timeout:
                pass

        if odom_count == 0:
            print("\033[31m[HOST  ]\033[0m ❌ NO ODOM DATA RECEIVED!\033[0m")
        else:
            print(f"\033[33m[HOST  ]\033[0m ✓ Received {odom_count} ODOM messages")

        # Send some velocity commands
        test_vels = [0.20, 0.30, 0.45]

        for vel in test_vels:
            print(f"\n\033[33m[HOST  ]\033[0m → Sending SET_VEL {vel} m/s...")
            payload = struct.pack('<ff', vel, vel)
            send_frame(sock, MSG_SET_VEL, payload)
            time.sleep(2)

            print(f"\033[33m[HOST  ]\033[0m → Sending STOP...")
            send_frame(sock, MSG_STOP)
            time.sleep(1)

        print("\n\033[33m[HOST  ]\033[0m Test complete. Monitoring for 5 more seconds...")
        time.sleep(5)

    except KeyboardInterrupt:
        print("\n\033[31m⚠️  Interrupted\033[0m")
    finally:
        sock.close()
        print("\033[33m[HOST  ]\033[0m Closed")


if __name__ == '__main__':
    main()
