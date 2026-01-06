#!/usr/bin/env python3
"""
Motor Control Optimizer V2 - Working version with proper ODOM handling
"""

import socket
import struct
import time
from collections import deque

# Configuration
ESP32_HOST = '192.168.68.52'
ESP32_PORT = 5000

# Protocol
SOF0, SOF1 = 0xAA, 0x55
MSG_ODOM = 0x01
MSG_SET_VEL = 0x10
MSG_SET_PID = 0x11
MSG_SET_DEADBAND = 0x12
MSG_STOP = 0x1A
MSG_SAVE_CONFIG = 0x17

# Robot config
WHEEL_DIAMETER = 0.082  # meters
TICKS_PER_REV = 360
DT = 0.2  # 200ms = 5 Hz


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


def receive_odom_batch(sock, duration=3.0):
    """Collect ODOM messages for a duration"""
    samples = []
    start = time.time()

    while time.time() - start < duration:
        try:
            data, _ = sock.recvfrom(1024)

            if len(data) >= 6 and data[0] == SOF0 and data[1] == SOF1:
                if data[2] == MSG_ODOM and data[3] >= 18:
                    payload = data[4:4+data[3]]
                    timestamp, delta_l, delta_r, x, y, theta = struct.unpack('<Ihhiih', payload[:18])
                    samples.append({
                        'timestamp': timestamp,
                        'delta_l': delta_l,
                        'delta_r': delta_r,
                        'x_mm': x,
                        'y_mm': y,
                        'theta_mrad': theta
                    })
        except socket.timeout:
            pass

    return samples


def analyze_samples(samples, target_vel):
    """Analyze ODOM samples and calculate statistics"""
    if not samples:
        return None

    # Calculate velocities from deltas
    circumference = WHEEL_DIAMETER * 3.14159
    vels_left = [(s['delta_l'] / TICKS_PER_REV) * circumference / DT for s in samples]
    vels_right = [(s['delta_r'] / TICKS_PER_REV) * circumference / DT for s in samples]

    avg_vel_l = sum(vels_left) / len(vels_left)
    avg_vel_r = sum(vels_right) / len(vels_right)
    avg_vel = (avg_vel_l + avg_vel_r) / 2

    # Calculate variance and max deviation
    variance_l = sum((v - avg_vel_l)**2 for v in vels_left) / len(vels_left)
    variance_r = sum((v - avg_vel_r)**2 for v in vels_right) / len(vels_right)

    error = abs(avg_vel - target_vel)
    error_pct = (error / target_vel * 100) if target_vel > 0 else 0

    return {
        'samples': len(samples),
        'avg_vel_left': avg_vel_l,
        'avg_vel_right': avg_vel_r,
        'avg_vel': avg_vel,
        'variance_left': variance_l,
        'variance_right': variance_r,
        'error': error,
        'error_pct': error_pct,
        'is_moving': abs(avg_vel) > 0.01
    }


def set_pid(sock, kp, ki, kd):
    """Set PID parameters"""
    payload = struct.pack('<fff', kp, ki, kd)
    send_frame(sock, MSG_SET_PID, payload)
    print(f"  ⚙️  PID: Kp={kp:.1f} Ki={ki:.1f} Kd={kd:.1f}")
    time.sleep(0.5)


def set_deadband(sock, fwd, rev):
    """Set deadband"""
    payload = struct.pack('<ff', fwd, rev)
    send_frame(sock, MSG_SET_DEADBAND, payload)
    print(f"  ⚙️  Deadband: Fwd={fwd:.1f} Rev={rev:.1f}")
    time.sleep(0.5)


def set_velocity(sock, vel_l, vel_r):
    """Set velocity"""
    payload = struct.pack('<ff', vel_l, vel_r)
    send_frame(sock, MSG_SET_VEL, payload)


def stop(sock):
    """Stop motors"""
    send_frame(sock, MSG_STOP)


def test_velocity(sock, target_vel, duration=3.0):
    """Test a specific velocity"""
    print(f"\n{'='*70}")
    print(f"🎯 Testing velocity: {target_vel:.2f} m/s")
    print(f"{'='*70}")

    # Command velocity
    set_velocity(sock, target_vel, target_vel)
    print(f"  ▶️  Motors commanded to {target_vel:.2f} m/s")

    # Collect samples
    samples = receive_odom_batch(sock, duration)

    # Stop motors
    stop(sock)
    time.sleep(0.5)

    # Analyze
    if not samples:
        print("  ❌ No ODOM data received!")
        return None

    result = analyze_samples(samples, target_vel)

    print(f"\n  📊 Results:")
    print(f"    Samples:      {result['samples']}")
    print(f"    Avg Vel Left:  {result['avg_vel_left']:+.3f} m/s")
    print(f"    Avg Vel Right: {result['avg_vel_right']:+.3f} m/s")
    print(f"    Target:        {target_vel:.3f} m/s")
    print(f"    Error:         {result['error']:.3f} m/s ({result['error_pct']:.1f}%)")

    if not result['is_moving']:
        print("    ❌ Motors NOT moving!")
    elif result['error_pct'] > 30:
        print("    ⚠️  Large error - needs tuning")
    else:
        print("    ✅ Good performance")

    return result


def optimize_deadband(sock):
    """Find optimal deadband"""
    print("\n" + "="*70)
    print("🔧 PHASE 1: DEADBAND OPTIMIZATION")
    print("="*70)

    test_vel = 0.25  # Test at moderate speed
    deadbands = [40, 45, 50, 55, 60, 65, 70, 75, 80]

    best_db = 50
    best_result = None

    for db in deadbands:
        print(f"\n🔍 Testing deadband: {db}")
        set_deadband(sock, db, db)

        result = test_velocity(sock, test_vel, duration=3.0)

        if result and result['is_moving']:
            if result['error_pct'] < 25:  # Good enough
                print(f"  ✅ Deadband {db} works well!")
                best_db = db
                best_result = result
                break
            elif best_result is None or result['error'] < best_result['error']:
                best_db = db
                best_result = result

    if best_result and best_result['is_moving']:
        print(f"\n✅ Best deadband: {best_db}")
        return best_db
    else:
        print("\n⚠️  Using default deadband: 60")
        return 60


def optimize_pid(sock, deadband):
    """Find optimal PID"""
    print("\n" + "="*70)
    print("🔧 PHASE 2: PID OPTIMIZATION")
    print("="*70)

    set_deadband(sock, deadband, deadband)

    # Test at low speed where control is hardest
    test_vel = 0.20

    pid_configs = [
        (15.0, 8.0, 0.2),   # Current
        (20.0, 10.0, 0.3),  # More aggressive
        (25.0, 12.0, 0.4),  # Even more
        (30.0, 15.0, 0.5),  # Very aggressive
        (35.0, 18.0, 0.6),  # Maximum
    ]

    best_pid = pid_configs[0]
    best_error = float('inf')

    for kp, ki, kd in pid_configs:
        print(f"\n🔍 Testing PID: Kp={kp} Ki={ki} Kd={kd}")
        set_pid(sock, kp, ki, kd)

        result = test_velocity(sock, test_vel, duration=3.0)

        if result and result['is_moving'] and result['error'] < best_error:
            best_error = result['error']
            best_pid = (kp, ki, kd)
            print(f"  🌟 New best! Error: {result['error']:.3f} m/s")

    print(f"\n✅ Best PID: Kp={best_pid[0]} Ki={best_pid[1]} Kd={best_pid[2]}")
    return best_pid


def full_range_test(sock, pid, deadband):
    """Test full velocity range"""
    print("\n" + "="*70)
    print("🔧 PHASE 3: FULL VELOCITY RANGE TEST")
    print("="*70)

    kp, ki, kd = pid
    set_pid(sock, kp, ki, kd)
    set_deadband(sock, deadband, deadband)

    velocities = [0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50]

    print(f"\nOptimized settings:")
    print(f"  PID: Kp={kp} Ki={ki} Kd={kd}")
    print(f"  Deadband: {deadband}")

    results = []
    for vel in velocities:
        result = test_velocity(sock, vel, duration=2.5)
        if result:
            results.append((vel, result))
        time.sleep(1)

    # Summary
    print("\n" + "="*70)
    print("📋 SUMMARY")
    print("="*70)
    print("\n  Velocity     Actual     Error")
    print("  " + "-"*40)

    for vel, result in results:
        status = "✅" if result['is_moving'] and result['error_pct'] < 20 else "⚠️ "
        print(f"  {status} {vel:.2f} m/s → {result['avg_vel']:+.3f} m/s  ({result['error_pct']:5.1f}%)")

    return results


def main():
    print("\n" + "="*70)
    print("MOTOR CONTROL OPTIMIZER V2")
    print("="*70)

    # Connect
    print("\n📡 Connecting to ESP32...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)

    # Send PING to establish connection
    print("📤 Sending PING to establish connection...")
    send_frame(sock, 0x7E)  # PING
    time.sleep(1)

    # Verify ODOM is streaming
    print("🔍 Verifying ODOM stream...")
    samples = receive_odom_batch(sock, 3.0)
    if not samples:
        print("❌ No ODOM data! ESP32 may not be ready.")
        return

    print(f"✅ Receiving ODOM @ {len(samples)/2:.1f} Hz\n")

    try:
        # Phase 1: Optimize deadband
        optimal_db = optimize_deadband(sock)

        # Phase 2: Optimize PID
        optimal_pid = optimize_pid(sock, optimal_db)

        # Phase 3: Full range test
        full_range_test(sock, optimal_pid, optimal_db)

        # Final recommendations
        print("\n" + "="*70)
        print("🎉 OPTIMIZATION COMPLETE!")
        print("="*70)
        print("\n📋 RECOMMENDED SETTINGS:")
        print(f"  PID Kp: {optimal_pid[0]:.2f}")
        print(f"  PID Ki: {optimal_pid[1]:.2f}")
        print(f"  PID Kd: {optimal_pid[2]:.2f}")
        print(f"  Deadband Forward: {optimal_db:.2f}")
        print(f"  Deadband Reverse: {optimal_db:.2f}")

        # Offer to save
        print("\n💾 Saving configuration to EEPROM...")
        send_frame(sock, MSG_SAVE_CONFIG)
        time.sleep(1)
        print("✅ Configuration saved!")

    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted")
    finally:
        stop(sock)
        sock.close()


if __name__ == '__main__':
    main()
