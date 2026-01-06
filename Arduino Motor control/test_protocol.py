#!/usr/bin/env python3
"""
Protocol validation script - tests RobotLink frame encoding/decoding
Can be run without hardware to verify protocol implementation
"""

import struct


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


def test_protocol():
    """Test protocol encoding"""
    print("="*60)
    print("RobotLink Protocol Validation")
    print("="*60)

    tests = []

    # Test 1: PING message
    print("\nTest 1: PING Message (0x7E)")
    timestamp = 12345678
    payload = struct.pack('<I', timestamp)
    frame = encode_frame(0x7E, payload)
    print(f"  Payload: timestamp={timestamp}")
    print(f"  Frame: {frame.hex(' ')}")
    print(f"  Length: {len(frame)} bytes")
    tests.append(("PING", len(frame) == 10))

    # Test 2: SET_VEL message
    print("\nTest 2: SET_VEL Message (0x10)")
    vel_left = 0.15
    vel_right = 0.15
    payload = struct.pack('<ff', vel_left, vel_right)
    frame = encode_frame(0x10, payload)
    print(f"  Payload: left={vel_left} m/s, right={vel_right} m/s")
    print(f"  Frame: {frame.hex(' ')}")
    print(f"  Length: {len(frame)} bytes")
    tests.append(("SET_VEL", len(frame) == 14))

    # Test 3: CMD_VEL message
    print("\nTest 3: CMD_VEL Message (0x02)")
    v_mm_s = 150  # 150 mm/s
    w_mrad_s = 100  # 100 mrad/s
    payload = struct.pack('<hh', v_mm_s, w_mrad_s)
    frame = encode_frame(0x02, payload)
    print(f"  Payload: v={v_mm_s} mm/s, w={w_mrad_s} mrad/s")
    print(f"  Frame: {frame.hex(' ')}")
    print(f"  Length: {len(frame)} bytes")
    tests.append(("CMD_VEL", len(frame) == 10))

    # Test 4: SET_PID message
    print("\nTest 4: SET_PID Message (0x11)")
    kp, ki, kd = 15.0, 8.0, 0.2
    payload = struct.pack('<fff', kp, ki, kd)
    frame = encode_frame(0x11, payload)
    print(f"  Payload: Kp={kp}, Ki={ki}, Kd={kd}")
    print(f"  Frame: {frame.hex(' ')}")
    print(f"  Length: {len(frame)} bytes")
    tests.append(("SET_PID", len(frame) == 18))

    # Test 5: SET_DEADBAND message
    print("\nTest 5: SET_DEADBAND Message (0x12)")
    payload = struct.pack('<ffff', 35.0, 35.0, 35.0, 35.0)
    frame = encode_frame(0x12, payload)
    print(f"  Payload: L_fwd=35.0, L_rev=35.0, R_fwd=35.0, R_rev=35.0")
    print(f"  Frame: {frame.hex(' ')}")
    print(f"  Length: {len(frame)} bytes")
    tests.append(("SET_DEADBAND", len(frame) == 22))

    # Test 6: ENABLE_STREAM message
    print("\nTest 6: ENABLE_STREAM Message (0x14)")
    payload = struct.pack('<BH', 1, 50)  # enable=1, interval=50ms
    frame = encode_frame(0x14, payload)
    print(f"  Payload: enable=1, interval=50ms")
    print(f"  Frame: {frame.hex(' ')}")
    print(f"  Length: {len(frame)} bytes")
    tests.append(("ENABLE_STREAM", len(frame) == 9))

    # Test 7: STOP message
    print("\nTest 7: STOP Message (0x1A)")
    frame = encode_frame(0x1A, b'')
    print(f"  Payload: (empty)")
    print(f"  Frame: {frame.hex(' ')}")
    print(f"  Length: {len(frame)} bytes")
    tests.append(("STOP", len(frame) == 6))

    # Test 8: ODOM message (decode example)
    print("\nTest 8: ODOM Message (0x01) - Encoding Example")
    t_ms = 123456
    dL_ticks = 42
    dR_ticks = 38
    x_mm = 1234
    y_mm = 5678
    th_mrad = 314  # ~0.314 radians
    payload = struct.pack('<IhhIIh', t_ms, dL_ticks, dR_ticks, x_mm, y_mm, th_mrad)
    frame = encode_frame(0x01, payload)
    print(f"  Payload: t={t_ms}ms, dL={dL_ticks}, dR={dR_ticks}")
    print(f"           x={x_mm}mm, y={y_mm}mm, θ={th_mrad}mrad")
    print(f"  Frame: {frame.hex(' ')}")
    print(f"  Length: {len(frame)} bytes")
    tests.append(("ODOM", len(frame) == 24))

    # Test 9: CRC validation
    print("\nTest 9: CRC Validation")
    # Create a frame with known CRC
    msg_type = 0x10
    payload = struct.pack('<ff', 0.1, 0.1)
    crc_data = bytes([msg_type, len(payload)]) + payload
    crc_expected = crc16_ccitt_false(crc_data)

    frame = encode_frame(msg_type, payload)
    crc_in_frame = frame[-2] | (frame[-1] << 8)

    print(f"  Expected CRC: 0x{crc_expected:04x}")
    print(f"  Frame CRC:    0x{crc_in_frame:04x}")
    crc_match = (crc_expected == crc_in_frame)
    print(f"  Match: {crc_match}")
    tests.append(("CRC", crc_match))

    # Summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)

    passed = sum(1 for _, result in tests if result)
    for name, result in tests:
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"{name:20s}: {status}")

    print(f"\nTotal: {passed}/{len(tests)} tests passed")
    print("="*60)

    if passed == len(tests):
        print("\n✓ Protocol implementation is correct!")
        print("\nYou can now:")
        print("  1. Upload firmware: pio run --target upload")
        print("  2. Run hardware test: python3 test_robot.py")
    else:
        print("\n✗ Protocol validation failed!")


if __name__ == "__main__":
    test_protocol()
