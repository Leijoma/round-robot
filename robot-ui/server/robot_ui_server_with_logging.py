"""
Robot Control UI Server - WITH COMPREHENSIVE LOGGING
Flask + SocketIO server for real-time robot control

This version adds extensive logging to debug motor control issues.
"""

from flask import Flask, send_from_directory
from flask_socketio import SocketIO, emit
import threading
import time
import sys
import os
from datetime import datetime

# Add server directory to Python path for robotlink import
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robotlink import RobotLink, MessageType, OdomPayload

# Configuration
ESP32_HOST = '192.168.68.52'
ESP32_PORT = 5000

# Flask app setup
app = Flask(__name__, static_folder='../static', static_url_path='')
app.config['SECRET_KEY'] = 'robot-ui-secret-2026'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global state
robot = None
robot_connected = False
running = False
esp32_thread = None

# Odometry state tracking
odom_state = {
    'encoder_left_total': 0,
    'encoder_right_total': 0,
    'last_timestamp': 0,
    'vel_left': 0.0,
    'vel_right': 0.0
}

# Command tracking for debugging
command_log = {
    'last_command': None,
    'last_command_time': 0,
    'commanded_vel_left': 0.0,
    'commanded_vel_right': 0.0
}

# Robot configuration
WHEEL_DIAMETER = 0.067  # meters (67mm)
TICKS_PER_REV = 48 * 120  # 48 encoder ticks * 120:1 gear ratio = 5760
METERS_PER_TICK = (WHEEL_DIAMETER * 3.14159) / TICKS_PER_REV

# Logging setup
log_file = open('/Users/magnus/Documents/PlatformIO/Projects/robot-ui/server/motor_debug.log', 'w')

def log_debug(message):
    """Write to both console and log file"""
    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
    log_msg = f'[{timestamp}] {message}'
    print(log_msg)
    log_file.write(log_msg + '\n')
    log_file.flush()

def init_robot_connection():
    """Initialize connection to ESP32"""
    global robot, robot_connected

    log_debug(f'\nConnecting to ESP32 at {ESP32_HOST}:{ESP32_PORT}...')

    try:
        robot = RobotLink(host=ESP32_HOST, port=ESP32_PORT)
        time.sleep(0.5)

        # Enable odometry streaming
        robot.enable_stream(True, interval_ms=50)  # 20 Hz
        log_debug('✓ ESP32 connection established')
        log_debug('✓ Odometry streaming enabled (20 Hz)')

        robot_connected = True
        return True

    except Exception as e:
        log_debug(f'✗ Failed to connect to ESP32: {e}')
        robot_connected = False
        return False


def esp32_communication_thread():
    """Background thread for ESP32 communication"""
    global running, robot, robot_connected, odom_state

    log_debug('ESP32 communication thread started')

    while running:
        if robot is None:
            time.sleep(1)
            continue

        try:
            # Poll for incoming messages from ESP32
            result = robot.receive_frame()

            if result is None:
                time.sleep(0.01)  # Prevent CPU spinning
                continue

            msg_type, payload = result

            # Handle odometry messages
            if msg_type == MessageType.MSG_ODOM:
                try:
                    odom = OdomPayload.unpack(payload)

                    # Update total encoder values
                    odom_state['encoder_left_total'] += odom.delta_left
                    odom_state['encoder_right_total'] += odom.delta_right

                    # Compute velocities from encoder deltas
                    if odom_state['last_timestamp'] > 0:
                        dt = (odom.timestamp - odom_state['last_timestamp']) / 1000.0
                        if dt > 0:
                            vel_left = (odom.delta_left * METERS_PER_TICK) / dt
                            vel_right = (odom.delta_right * METERS_PER_TICK) / dt
                            odom_state['vel_left'] = vel_left
                            odom_state['vel_right'] = vel_right

                            # Calculate ticks/second
                            ticks_per_sec_left = odom.delta_left / dt
                            ticks_per_sec_right = odom.delta_right / dt

                            # Log velocity vs commanded velocity
                            if abs(vel_left) > 0.05 or abs(vel_right) > 0.05:
                                log_debug(f'ODOM: Actual vel L={vel_left:.3f} R={vel_right:.3f} m/s | '
                                        f'L={ticks_per_sec_left:.1f} R={ticks_per_sec_right:.1f} ticks/s | '
                                        f'Commanded L={command_log["commanded_vel_left"]:.3f} R={command_log["commanded_vel_right"]:.3f} m/s | '
                                        f'dt={dt*1000:.1f}ms dL={odom.delta_left} dR={odom.delta_right}')

                    odom_state['last_timestamp'] = odom.timestamp

                    # Broadcast to all connected WebSocket clients
                    socketio.emit('odom_update', {
                        'encoder_left': odom_state['encoder_left_total'],
                        'encoder_right': odom_state['encoder_right_total'],
                        'vel_left': round(odom_state['vel_left'], 3),
                        'vel_right': round(odom_state['vel_right'], 3),
                        'pwm_left': 0,
                        'pwm_right': 0,
                        'timestamp': odom.timestamp,
                        'x_mm': odom.x_mm,
                        'y_mm': odom.y_mm,
                        'theta_mrad': odom.theta_mrad
                    })

                except Exception as e:
                    log_debug(f'Error parsing odometry: {e}')
                    import traceback
                    traceback.print_exc()

        except Exception as e:
            log_debug(f'Error in ESP32 thread: {e}')
            time.sleep(0.1)

    log_debug('ESP32 communication thread stopped')


# WebSocket event handlers

@socketio.on('connect')
def handle_connect():
    """Client connected to WebSocket"""
    log_debug(f'Client connected')

    # Send connection status
    emit('connection_status', {
        'status': 'connected',
        'esp32': robot_connected,
        'esp32_host': ESP32_HOST,
        'esp32_port': ESP32_PORT
    })


@socketio.on('disconnect')
def handle_disconnect():
    """Client disconnected from WebSocket"""
    log_debug(f'Client disconnected')


@socketio.on('motor_command')
def handle_motor_command(data):
    """Handle motor control commands from UI"""
    global robot, command_log

    if robot is None:
        emit('error', {'message': 'ESP32 not connected'})
        return

    try:
        command_type = data.get('type')
        timestamp = time.time()

        if command_type == 'velocity':
            vel_left = data.get('vel_left', 0.0)
            vel_right = data.get('vel_right', 0.0)

            # Update command log
            command_log['last_command'] = 'velocity'
            command_log['last_command_time'] = timestamp
            command_log['commanded_vel_left'] = vel_left
            command_log['commanded_vel_right'] = vel_right

            robot.set_velocity(vel_left, vel_right)
            log_debug(f'[MOTOR CMD] VELOCITY: L={vel_left:.3f} R={vel_right:.3f} m/s')

        elif command_type == 'cmd_vel':
            v = data.get('v', 0.0)
            w = data.get('w', 0.0)

            command_log['last_command'] = 'cmd_vel'
            command_log['last_command_time'] = timestamp

            robot.cmd_vel(v, w)
            log_debug(f'[MOTOR CMD] CMD_VEL: v={v:.3f} m/s, w={w:.3f} rad/s')

        elif command_type == 'stop':
            command_log['last_command'] = 'stop'
            command_log['last_command_time'] = timestamp
            command_log['commanded_vel_left'] = 0.0
            command_log['commanded_vel_right'] = 0.0

            robot.stop()
            log_debug(f'[MOTOR CMD] **STOP** sent to ESP32')

        else:
            log_debug(f'[MOTOR CMD] ERROR: Unknown command type: {command_type}')
            emit('error', {'message': f'Unknown command type: {command_type}'})

    except Exception as e:
        log_debug(f'[MOTOR CMD] ERROR: Exception: {e}')
        emit('error', {'message': str(e)})


@app.route('/')
def index():
    """Serve index.html"""
    return send_from_directory('../static', 'index.html')


def start_background_threads():
    """Start background threads"""
    global running, esp32_thread

    running = True

    # Initialize robot connection
    init_robot_connection()

    # Start ESP32 communication thread
    esp32_thread = threading.Thread(target=esp32_communication_thread, daemon=True)
    esp32_thread.start()


def shutdown():
    """Clean shutdown"""
    global running, robot

    log_debug('\nShutting down...')
    running = False

    if robot:
        robot.close()

    log_file.close()
    log_debug('Server stopped')


if __name__ == '__main__':
    log_debug('\n' + '=' * 70)
    log_debug('Robot Control UI Server - WITH COMPREHENSIVE LOGGING')
    log_debug('=' * 70)
    log_debug(f'ESP32: {ESP32_HOST}:{ESP32_PORT}')
    log_debug('Web UI: http://localhost:5001')
    log_debug('Log file: /Users/magnus/Documents/PlatformIO/Projects/robot-ui/server/motor_debug.log')
    log_debug('=' * 70 + '\n')

    # Start background threads
    start_background_threads()

    try:
        # Run Flask-SocketIO server
        socketio.run(app, host='0.0.0.0', port=5001, debug=False, allow_unsafe_werkzeug=True)

    except KeyboardInterrupt:
        shutdown()
    except Exception as e:
        log_debug(f'Server error: {e}')
        shutdown()
