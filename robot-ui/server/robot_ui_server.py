"""
Robot Control UI Server
Flask + SocketIO server for real-time robot control
"""

from flask import Flask, send_from_directory
from flask_socketio import SocketIO, emit
import threading
import time
import sys
import os

# Add server directory to Python path for robotlink import
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robotlink import RobotLink, MessageType, OdomPayload, LidarScanPayload

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

def init_robot_connection():
    """Initialize connection to ESP32"""
    global robot, robot_connected

    print(f'\n Connecting to ESP32 at {ESP32_HOST}:{ESP32_PORT}...')

    try:
        robot = RobotLink(host=ESP32_HOST, port=ESP32_PORT)
        time.sleep(0.5)

        # Enable odometry streaming
        robot.enable_stream(True, interval_ms=50)  # 20 Hz
        print('✓ ESP32 connection established')
        print('✓ Odometry streaming enabled (20 Hz)')

        robot_connected = True
        return True

    except Exception as e:
        print(f'✗ Failed to connect to ESP32: {e}')
        robot_connected = False
        return False


def esp32_communication_thread():
    """Background thread for ESP32 communication"""
    global running, robot, robot_connected, odom_state

    print('ESP32 communication thread started')

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
                    # At 20 Hz (50ms interval), delta is measured over ~0.05 seconds
                    if odom_state['last_timestamp'] > 0:
                        dt = (odom.timestamp - odom_state['last_timestamp']) / 1000.0  # Convert ms to seconds
                        if dt > 0:
                            # Velocity = (delta_ticks * meters_per_tick) / time
                            vel_left = (odom.delta_left * METERS_PER_TICK) / dt
                            vel_right = (odom.delta_right * METERS_PER_TICK) / dt
                            odom_state['vel_left'] = vel_left
                            odom_state['vel_right'] = vel_right

                    odom_state['last_timestamp'] = odom.timestamp

                    # Broadcast to all connected WebSocket clients
                    socketio.emit('odom_update', {
                        'encoder_left': odom_state['encoder_left_total'],
                        'encoder_right': odom_state['encoder_right_total'],
                        'vel_left': round(odom_state['vel_left'], 3),
                        'vel_right': round(odom_state['vel_right'], 3),
                        'pwm_left': 0,  # Not available in Arduino format
                        'pwm_right': 0,  # Not available in Arduino format
                        'timestamp': odom.timestamp,
                        'x_mm': odom.x_mm,
                        'y_mm': odom.y_mm,
                        'theta_mrad': odom.theta_mrad
                    })

                except Exception as e:
                    print(f'Error parsing odometry: {e}')
                    import traceback
                    traceback.print_exc()

            # Handle LIDAR scan messages
            elif msg_type == MessageType.MSG_LIDAR_SCAN:
                try:
                    lidar_scan = LidarScanPayload.unpack(payload)

                    # Convert to format suitable for polar plot
                    # Each reading needs angle and distance
                    readings = []
                    for i, reading in enumerate(lidar_scan.readings):
                        angle = (lidar_scan.start_angle + i) % 360
                        readings.append({
                            'angle': angle,
                            'distance': reading.distance_mm,
                            'strength': reading.signal_strength,
                            'valid': not reading.invalid
                        })

                    # Debug: Log LIDAR scan reception (throttled)
                    if not hasattr(esp32_communication_thread, 'lidar_scan_count'):
                        esp32_communication_thread.lidar_scan_count = 0
                    esp32_communication_thread.lidar_scan_count += 1
                    if esp32_communication_thread.lidar_scan_count % 50 == 0:
                        valid_count = sum(1 for r in readings if r['valid'])
                        print(f'LIDAR: Scan #{esp32_communication_thread.lidar_scan_count} '
                              f'({valid_count}/{len(readings)} valid, RPM={lidar_scan.rpm})')

                    # Broadcast to all connected WebSocket clients
                    socketio.emit('lidar_scan', {
                        'timestamp': lidar_scan.timestamp,
                        'rpm': lidar_scan.rpm,
                        'readings': readings
                    })

                except Exception as e:
                    print(f'Error parsing LIDAR scan: {e}')
                    import traceback
                    traceback.print_exc()

        except Exception as e:
            print(f'Error in ESP32 thread: {e}')
            time.sleep(0.1)

    print('ESP32 communication thread stopped')


# WebSocket event handlers

@socketio.on('connect')
def handle_connect():
    """Client connected to WebSocket"""
    client_id = request.sid if 'request' in dir() else 'unknown'
    print(f'Client connected: {client_id}')

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
    client_id = request.sid if 'request' in dir() else 'unknown'
    print(f'Client disconnected: {client_id}')


@socketio.on('motor_command')
def handle_motor_command(data):
    """Handle motor control commands from UI"""
    global robot

    if robot is None:
        emit('error', {'message': 'ESP32 not connected'})
        return

    try:
        command_type = data.get('type')

        if command_type == 'velocity':
            # Set wheel velocities (m/s)
            vel_left = data.get('vel_left', 0.0)
            vel_right = data.get('vel_right', 0.0)
            robot.set_velocity(vel_left, vel_right)
            print(f'Motor command: vel_left={vel_left}, vel_right={vel_right} m/s')

        elif command_type == 'cmd_vel':
            # Set linear + angular velocity
            v = data.get('v', 0.0)  # m/s
            w = data.get('w', 0.0)  # rad/s
            robot.cmd_vel(v, w)
            print(f'Cmd vel: v={v} m/s, w={w} rad/s')

        elif command_type == 'stop':
            robot.stop()
            print('Motor STOP command')

        else:
            emit('error', {'message': f'Unknown command type: {command_type}'})

    except Exception as e:
        print(f'Error handling motor command: {e}')
        emit('error', {'message': str(e)})


@socketio.on('enable_stream')
def handle_enable_stream(data):
    """Handle odometry streaming enable/disable"""
    global robot

    if robot is None:
        emit('error', {'message': 'ESP32 not connected'})
        return

    try:
        enable = data.get('enable', True)
        interval_ms = data.get('interval_ms', 200)  # Default 200ms = 5 Hz

        robot.enable_stream(enable, interval_ms=interval_ms)
        print(f'Odometry streaming: {"ENABLED" if enable else "DISABLED"} @ {interval_ms}ms')

    except Exception as e:
        print(f'Error handling enable_stream: {e}')
        emit('error', {'message': str(e)})


@socketio.on('lidar_enable')
def handle_lidar_enable(data):
    """Handle LIDAR motor enable/disable"""
    global robot

    if robot is None:
        emit('error', {'message': 'ESP32 not connected'})
        return

    try:
        enable = data.get('enable', True)
        robot.lidar_enable(enable)
        print(f'LIDAR motor: {"ENABLED" if enable else "DISABLED"}')

    except Exception as e:
        print(f'Error handling lidar_enable: {e}')
        emit('error', {'message': str(e)})


@socketio.on('lidar_set_rpm')
def handle_lidar_set_rpm(data):
    """Handle LIDAR RPM setting"""
    global robot

    if robot is None:
        emit('error', {'message': 'ESP32 not connected'})
        return

    try:
        rpm = data.get('rpm', 220)
        robot.lidar_set_rpm(rpm)
        print(f'LIDAR target RPM: {rpm}')

    except Exception as e:
        print(f'Error handling lidar_set_rpm: {e}')
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

    print('\nShutting down...')
    running = False

    if robot:
        robot.close()

    print('Server stopped')


if __name__ == '__main__':
    print('\n' + '=' * 70)
    print('Robot Control UI Server')
    print('=' * 70)
    print(f'ESP32: {ESP32_HOST}:{ESP32_PORT}')
    print('Web UI: http://localhost:5001')
    print('=' * 70 + '\n')

    # Start background threads
    start_background_threads()

    try:
        # Run Flask-SocketIO server
        socketio.run(app, host='0.0.0.0', port=5001, debug=False, allow_unsafe_werkzeug=True)

    except KeyboardInterrupt:
        shutdown()
    except Exception as e:
        print(f'Server error: {e}')
        shutdown()
