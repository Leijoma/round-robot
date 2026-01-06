#!/usr/bin/env python3
"""
Final Motor Test - Tests CMD_VEL (0x02) command
Does NOT open serial port to avoid resetting ESP32
"""

from robotlink import RobotLink, MessageType, OdomPayload
import time

odom_readings = []

def on_odom(payload):
    odom = OdomPayload.unpack(payload)
    odom_readings.append({
        'time': time.time(),
        'left_enc': odom.encoder_left,
        'right_enc': odom.encoder_right,
        'left_vel': odom.vel_left,
        'right_vel': odom.vel_right,
        'left_pwm': odom.pwm_left,
        'right_pwm': odom.pwm_right
    })

robot = RobotLink(host='192.168.68.52', port=5000)
robot.register_callback(MessageType.MSG_ODOM, on_odom)

print('=' * 70)
print('FINAL MOTOR TEST - Using CMD_VEL (0x02) for Arduino')
print('=' * 70)

# Enable odometry
print('\n1. Enabling odometry stream...')
robot.enable_stream(True, 50)
time.sleep(2)

# Collect baseline
odom_readings.clear()
start = time.time()
while time.time() - start < 1:
    robot.process_messages(timeout=0.05)
    time.sleep(0.01)

if len(odom_readings) == 0:
    print('✗ ERROR: No odometry data received')
    print('  - Check ESP32 is powered and connected')
    print('  - Check Arduino is connected to ESP32')
    robot.close()
    exit(1)

baseline = odom_readings[-1]
print(f'   Baseline: Enc L={baseline["left_enc"]} R={baseline["right_enc"]}')
print(f'   Baseline: PWM L={baseline["left_pwm"]} R={baseline["right_pwm"]}')
print(f'   Data rate: {len(odom_readings)} Hz')

# Send CMD_VEL
print('\n2. Sending CMD_VEL (0x02): 0.2 m/s forward, 0 rad/s turn...')
print('   (Arduino expects: int16 v_mm_s, int16 w_mrad_s)')
robot.cmd_vel(0.2, 0.0)  # 200 mm/s forward, 0 mrad/s turn
time.sleep(0.5)

# Monitor for 4 seconds
print('   Monitoring for 4 seconds...\n')
odom_readings.clear()
start = time.time()
max_pwm_seen = 0

while time.time() - start < 4:
    robot.process_messages(timeout=0.05)
    if len(odom_readings) > 0:
        latest = odom_readings[-1]
        if abs(latest['left_pwm']) > abs(max_pwm_seen):
            max_pwm_seen = latest['left_pwm']

        # Print update every second
        elapsed = latest['time'] - start
        if len(odom_readings) % 20 == 0:  # ~every second at 20Hz
            print(f'   [{elapsed:.1f}s] Enc: L={latest["left_enc"]:6d} R={latest["right_enc"]:6d} | '
                  f'PWM: L={latest["left_pwm"]:4d} R={latest["right_pwm"]:4d}')
    time.sleep(0.01)

# Stop motors
print('\n3. Stopping motors...')
robot.stop()
time.sleep(0.5)

# Final reading
robot.process_messages(timeout=0.1)
if len(odom_readings) > 0:
    final = odom_readings[-1]

    enc_change_left = final['left_enc'] - baseline['left_enc']
    enc_change_right = final['right_enc'] - baseline['right_enc']

    print(f'   Final: Enc L={final["left_enc"]} R={final["right_enc"]}')
    print(f'   Final: PWM L={final["left_pwm"]} R={final["right_pwm"]}')
    print(f'   Change: L={enc_change_left:+d} R={enc_change_right:+d}')
    print(f'   Max PWM seen: {max_pwm_seen}')

    # Evaluate results
    print('\n' + '=' * 70)
    print('RESULTS')
    print('=' * 70)

    if max_pwm_seen != 0:
        print(f'✓ PWM WAS NON-ZERO (max={max_pwm_seen})')
        print('  → Arduino received and processed the CMD_VEL command!')
        print('  → PID controller is active')

        if abs(enc_change_left) > 100 or abs(enc_change_right) > 100:
            print(f'✓ ENCODERS CHANGED SIGNIFICANTLY')
            print(f'  → Motors moved: L={enc_change_left:+d}, R={enc_change_right:+d} counts')
            print('  → MOTORS ARE WORKING!')
        elif abs(enc_change_left) > 0 or abs(enc_change_right) > 0:
            print(f'⚠ ENCODERS CHANGED SLIGHTLY')
            print(f'  → Small movement: L={enc_change_left:+d}, R={enc_change_right:+d} counts')
            print('  → Possible issues: wheels lifted, low battery, or mechanical friction')
        else:
            print(f'⚠ NO ENCODER MOVEMENT despite non-zero PWM')
            print('  → Check: encoder wires, wheel coupling, mechanical blockage')
    elif abs(enc_change_left) > 50 or abs(enc_change_right) > 50:
        print(f'⚠ ENCODERS CHANGED but PWM=0')
        print(f'  → Movement: L={enc_change_left:+d}, R={enc_change_right:+d}')
        print('  → This is unexpected - external force moving robot?')
    else:
        print('✗ NO MOTOR RESPONSE')
        print(f'  → PWM stayed at 0')
        print(f'  → Encoders unchanged: L={enc_change_left:+d}, R={enc_change_right:+d}')
        print('\n  Possible causes:')
        print('  1. Arduino firmware doesn\'t understand MSG_CMD_VEL (0x02)')
        print('  2. ESP32 isn\'t forwarding the message to Arduino')
        print('  3. Arduino Serial connection issue (GPIO 26/27)')
        print('  4. Arduino motor controller firmware not loaded')

stats = robot.get_stats()
print(f'\n' + '=' * 70)
print(f'Communication: {stats["frames_sent"]} sent, {stats["frames_received"]} received, '
      f'{stats["crc_errors"]} CRC errors')
print('=' * 70)

robot.close()
