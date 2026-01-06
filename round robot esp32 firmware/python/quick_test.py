#!/usr/bin/env python3
"""Quick test - Send CMD_VEL and check if motors respond"""

from robotlink import RobotLink, MessageType, OdomPayload
import time

odom_count = 0
pwm_values = []

def on_odom(payload):
    global odom_count
    odom = OdomPayload.unpack(payload)
    odom_count += 1
    pwm_values.append((odom.pwm_left, odom.pwm_right))
    if odom_count <= 3:
        print(f"  Odom #{odom_count}: Enc L={odom.encoder_left} R={odom.encoder_right}, PWM L={odom.pwm_left} R={odom.pwm_right}")

robot = RobotLink(host='192.168.68.52', port=5000)
robot.register_callback(MessageType.MSG_ODOM, on_odom)

print('Quick Motor Test')
print('=' * 50)

# Enable stream
print('\n1. Enabling odometry...')
robot.enable_stream(True, 50)
time.sleep(2)

# Collect some data
for _ in range(30):
    robot.process_messages(timeout=0.05)
    time.sleep(0.02)

if odom_count == 0:
    print('✗ No odometry - ESP32 or Arduino not responding')
    robot.close()
    exit(1)

print(f'✓ Receiving odometry ({odom_count} messages)')

# Send CMD_VEL
print('\n2. Sending CMD_VEL (0x02): 0.2 m/s forward...')
robot.cmd_vel(0.2, 0.0)
time.sleep(0.5)

# Monitor
pwm_values.clear()
for _ in range(60):  # 3 seconds
    robot.process_messages(timeout=0.05)
    time.sleep(0.05)

# Check PWM
max_pwm = max([abs(p[0]) for p in pwm_values] + [abs(p[1]) for p in pwm_values]) if pwm_values else 0

print('\n3. Results:')
if max_pwm > 0:
    print(f'✓ PWM changed! Max PWM = {max_pwm}')
    print('  → Motors are responding to CMD_VEL!')
else:
    print(f'✗ PWM stayed at 0')
    print('  → Motors NOT responding')

# Stop
robot.stop()
stats = robot.get_stats()
print(f'\nStats: {stats["frames_sent"]} sent, {stats["frames_received"]} received')
robot.close()
