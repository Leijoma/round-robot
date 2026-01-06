#!/usr/bin/env python3
"""
Loopback Test - SoftwareSerial A0/A1
Tests RobotLink protocol with loopback connection (A0 -> A1)
Monitors debug output on USB Serial
"""

import serial
import struct
import time
import threading

def crc16_ccitt_false(data):
    """Calculate CRC16-CCITT-FALSE"""
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

def encode_frame(msg_type, payload):
    """Encode a RobotLink frame"""
    frame = bytearray()
    frame.append(0xAA)  # SOF0
    frame.append(0x55)  # SOF1
    frame.append(msg_type)
    frame.append(len(payload))
    frame.extend(payload)

    # Calculate CRC over TYPE, LEN, PAYLOAD
    crc_data = bytes([msg_type, len(payload)]) + payload
    crc = crc16_ccitt_false(crc_data)
    frame.append(crc & 0xFF)        # CRC low
    frame.append((crc >> 8) & 0xFF) # CRC high

    return frame

def monitor_usb_debug(port, stop_event):
    """Monitor USB serial debug output in background thread"""
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
        print(f"[DEBUG] Connected to USB: {port}")
        print("[DEBUG] " + "="*50)

        while not stop_event.is_set():
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    print(f"[DEBUG] {line}")

        ser.close()
    except Exception as e:
        print(f"[DEBUG] Error: {e}")

def test_loopback(softserial_port, usb_port):
    """Test loopback on SoftwareSerial"""

    print("="*60)
    print("RobotLink Loopback Test - SoftwareSerial A0/A1")
    print("="*60)
    print(f"\nUSB Debug Port: {usb_port} @ 115200")
    print(f"SoftSerial Port: {softserial_port} @ 9600")
    print("\nNOTE: A0 and A1 should be connected (loopback)")
    print("="*60)

    # Start USB debug monitor in background
    stop_event = threading.Event()
    debug_thread = threading.Thread(target=monitor_usb_debug, args=(usb_port, stop_event))
    debug_thread.daemon = True
    debug_thread.start()

    time.sleep(2)  # Let debug output settle

    print("\n[TEST] Opening SoftwareSerial connection...")

    try:
        # Open SoftwareSerial connection
        soft = serial.Serial(softserial_port, 9600, timeout=1)
        print(f"[TEST] Connected to SoftwareSerial: {softserial_port}")

        time.sleep(0.5)

        # Test 1: PING
        print("\n" + "="*60)
        print("[TEST 1] Sending PING message")
        print("="*60)

        timestamp = int(time.time() * 1000) & 0xFFFFFFFF
        payload = struct.pack('<I', timestamp)
        frame = encode_frame(0x7E, payload)

        print(f"[TEST] TX: {frame.hex(' ')}")
        soft.write(frame)

        time.sleep(0.5)

        # Check for PONG
        if soft.in_waiting > 0:
            response = soft.read(soft.in_waiting)
            print(f"[TEST] RX: {response.hex(' ')}")
            if len(response) >= 2 and response[0] == 0xAA and response[1] == 0x55:
                if response[2] == 0x7F:  # PONG
                    print("[TEST] ✓ PONG received!")
                else:
                    print(f"[TEST] Received message type: 0x{response[2]:02x}")
        else:
            print("[TEST] ⚠ No response received")

        time.sleep(1)

        # Test 2: SET_VEL
        print("\n" + "="*60)
        print("[TEST 2] Sending SET_VEL message (0.15 m/s)")
        print("="*60)

        payload = struct.pack('<ff', 0.15, 0.15)
        frame = encode_frame(0x10, payload)

        print(f"[TEST] TX: {frame.hex(' ')}")
        soft.write(frame)

        time.sleep(0.5)

        # Test 3: STOP
        print("\n" + "="*60)
        print("[TEST 3] Sending STOP message")
        print("="*60)

        frame = encode_frame(0x1A, b'')

        print(f"[TEST] TX: {frame.hex(' ')}")
        soft.write(frame)

        time.sleep(0.5)

        # Test 4: Enable streaming
        print("\n" + "="*60)
        print("[TEST 4] Enable odometry streaming")
        print("="*60)

        payload = struct.pack('<BH', 1, 100)  # enable=1, interval=100ms
        frame = encode_frame(0x14, payload)

        print(f"[TEST] TX: {frame.hex(' ')}")
        soft.write(frame)

        print("\n[TEST] Waiting for ODOM messages (5 seconds)...")
        time.sleep(5)

        # Check for ODOM
        odom_count = 0
        if soft.in_waiting > 0:
            data = soft.read(soft.in_waiting)
            print(f"[TEST] Received {len(data)} bytes")

            # Count ODOM frames
            i = 0
            while i < len(data) - 1:
                if data[i] == 0xAA and data[i+1] == 0x55:
                    if i + 2 < len(data) and data[i+2] == 0x01:
                        odom_count += 1
                i += 1

            print(f"[TEST] ✓ Received {odom_count} ODOM messages")

        # Disable streaming
        print("\n[TEST] Disabling streaming...")
        payload = struct.pack('<BH', 0, 0)  # disable
        frame = encode_frame(0x14, payload)
        soft.write(frame)

        time.sleep(0.5)

        # Test 5: GET_CONFIG
        print("\n" + "="*60)
        print("[TEST 5] Request configuration")
        print("="*60)

        frame = encode_frame(0x15, b'')

        print(f"[TEST] TX: {frame.hex(' ')}")
        soft.write(frame)

        time.sleep(0.5)

        # Check for CONFIG_RESP
        if soft.in_waiting > 0:
            response = soft.read(soft.in_waiting)
            print(f"[TEST] RX: {response.hex(' ')}")

            if len(response) >= 3 and response[0] == 0xAA and response[1] == 0x55:
                if response[2] == 0x16:  # CONFIG_RESP
                    print("[TEST] ✓ CONFIG_RESP received!")

                    # Parse config
                    if len(response) >= 26:
                        payload = response[4:24]
                        wheel_diam = struct.unpack('<f', payload[0:4])[0]
                        wheelbase = struct.unpack('<f', payload[4:8])[0]
                        ticks = struct.unpack('<f', payload[8:12])[0]

                        print(f"[TEST]   Wheel diameter: {wheel_diam*1000:.1f} mm")
                        print(f"[TEST]   Wheelbase: {wheelbase*1000:.1f} mm")
                        print(f"[TEST]   Ticks/rev: {ticks:.0f}")
        else:
            print("[TEST] ⚠ No response received")

        time.sleep(1)

        print("\n" + "="*60)
        print("TEST SUMMARY")
        print("="*60)
        print("✓ All test commands sent successfully")
        print("✓ Check debug output above for command reception")
        print("\nIf you see 'RX: Type=...' messages in debug output,")
        print("the loopback is working correctly!")
        print("="*60)

        soft.close()

    except Exception as e:
        print(f"\n[ERROR] Test failed: {e}")
    finally:
        stop_event.set()
        debug_thread.join(timeout=1)

if __name__ == "__main__":
    import sys

    # Auto-detect ports
    import serial.tools.list_ports

    ports = list(serial.tools.list_ports.comports())

    print("Available ports:")
    for i, port in enumerate(ports):
        print(f"  [{i}] {port.device} - {port.description}")

    # Find Arduino (USB debug)
    usb_port = None
    soft_port = None

    for p in ports:
        if 'usbmodem' in p.device or 'Arduino' in p.description:
            usb_port = p.device
            print(f"\nAuto-selected USB (debug): {usb_port}")
            break

    if not usb_port:
        print("\n⚠ Could not auto-detect Arduino USB port")
        sys.exit(1)

    # For loopback test, SoftwareSerial data will loop back internally
    # We'll use the same USB port for sending commands
    # (In reality, you'd have a separate FTDI on A0/A1)

    print("\n⚠ LOOPBACK MODE:")
    print("  With A0-A1 connected, commands sent by Arduino will be")
    print("  received back by Arduino (echo test)")
    print("\nPress Enter to start test or Ctrl+C to cancel...")

    try:
        input()
    except KeyboardInterrupt:
        print("\nTest cancelled")
        sys.exit(0)

    # For loopback, we need an external USB-serial adapter on A0/A1
    # Ask user for the port
    print("\nEnter the port for SoftwareSerial (A0/A1):")
    print("  e.g., /dev/cu.usbserial-XXXX")
    print("  or press Enter to skip and just monitor debug")

    soft_port = input("Port: ").strip()

    if not soft_port:
        print("\nMonitoring debug output only...")
        stop_event = threading.Event()
        try:
            monitor_usb_debug(usb_port, stop_event)
        except KeyboardInterrupt:
            print("\nStopped")
            stop_event.set()
    else:
        test_loopback(soft_port, usb_port)
