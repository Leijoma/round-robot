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
import numpy as np

# Add server directory to Python path for robotlink import
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robotlink import RobotLink, MessageType, OdomPayload, LidarScanPayload

# SLAM imports (Phase 2, 3, & 4 Integration)
from slam.data_sync import DataSynchronizer
from slam.sensor_data import OdomReading, LidarScan, LidarReading
from slam.motion_model import RobotParameters
from slam.localization import IntegratedLocalizer

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

# SLAM state (Phase 2, 3, & 4 Integration)
data_synchronizer = None
localizer = None  # Integrated localizer (dead reckoning + ICP)

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
    global robot, robot_connected, data_synchronizer, localizer

    print(f'\nConnecting to ESP32 at {ESP32_HOST}:{ESP32_PORT}...')

    try:
        robot = RobotLink(host=ESP32_HOST, port=ESP32_PORT)
        time.sleep(0.5)

        # Enable odometry streaming at 10 Hz (100ms)
        robot.enable_stream(True, interval_ms=100)  # 10 Hz
        print('✓ ESP32 connection established')
        print('✓ Odometry streaming enabled (10 Hz @ 100ms interval)')

        # Initialize SLAM components (Phase 2)
        data_synchronizer = DataSynchronizer(
            odom_buffer_size=50,  # 5 seconds @ 10Hz
            lidar_buffer_size=20,  # 5 seconds @ 4Hz
            timeout_sec=1.0
        )
        print('✓ Data synchronizer initialized')

        # Initialize integrated localizer (Phase 3 & 4)
        robot_params = RobotParameters(
            wheel_diameter=0.082,  # 82mm wheels
            wheelbase=0.24,  # 240mm wheelbase
            ticks_per_revolution=360
        )
        localizer = IntegratedLocalizer(robot_params=robot_params)
        print('✓ Integrated localizer initialized (dead reckoning + ICP)')

        robot_connected = True
        return True

    except Exception as e:
        print(f'✗ Failed to connect to ESP32: {e}')
        robot_connected = False
        return False


def esp32_communication_thread():
    """Background thread for ESP32 communication"""
    global running, robot, robot_connected, odom_state, data_synchronizer, localizer

    print('ESP32 communication thread started')

    # Stats logging
    last_stats_time = time.time()

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
                    # At 10 Hz (100ms interval), delta is measured over ~0.1 seconds
                    if odom_state['last_timestamp'] > 0:
                        dt = (odom.timestamp - odom_state['last_timestamp']) / 1000.0  # Convert ms to seconds
                        if dt > 0:
                            # Velocity = (delta_ticks * meters_per_tick) / time
                            vel_left = (odom.delta_left * METERS_PER_TICK) / dt
                            vel_right = (odom.delta_right * METERS_PER_TICK) / dt
                            odom_state['vel_left'] = vel_left
                            odom_state['vel_right'] = vel_right

                    odom_state['last_timestamp'] = odom.timestamp

                    # Add to SLAM data synchronizer (Phase 2) and localizer (Phase 4)
                    if data_synchronizer and localizer:
                        odom_reading = OdomReading(
                            timestamp=odom.timestamp,
                            delta_left=odom.delta_left,
                            delta_right=odom.delta_right,
                            x_mm=odom.x_mm,
                            y_mm=odom.y_mm,
                            theta_mrad=odom.theta_mrad,
                            vel_left=odom_state['vel_left'],
                            vel_right=odom_state['vel_right']
                        )
                        data_synchronizer.add_odometry(odom_reading)
                        localizer.update_odometry(odom_reading)

                        # Emit pose update (Story 5.1)
                        dr_pose = localizer.get_dead_reckoning_pose()
                        icp_pose = localizer.get_corrected_pose()
                        drift = localizer.get_pose_drift()

                        socketio.emit('pose_update', {
                            'dead_reckoning': {
                                'x': dr_pose.x,
                                'y': dr_pose.y,
                                'theta_deg': np.rad2deg(dr_pose.theta)
                            },
                            'icp_corrected': {
                                'x': icp_pose.x,
                                'y': icp_pose.y,
                                'theta_deg': np.rad2deg(icp_pose.theta)
                            },
                            'drift': {
                                'position': drift['distance_drift'],
                                'heading': drift['heading_drift_deg']
                            }
                        })

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

                    # Add to SLAM data synchronizer (Phase 2)
                    if data_synchronizer:
                        lidar_readings = []
                        for i, r in enumerate(lidar_scan.readings):
                            angle = (lidar_scan.start_angle + i) % 360
                            lidar_readings.append(LidarReading(
                                angle_deg=angle,
                                distance_mm=r.distance_mm,
                                signal_strength=r.signal_strength,
                                valid=not r.invalid
                            ))

                        lidar_scan_obj = LidarScan(
                            timestamp=lidar_scan.timestamp,
                            rpm=lidar_scan.rpm,
                            readings=lidar_readings
                        )
                        data_synchronizer.add_lidar_scan(lidar_scan_obj)

                        # Update integrated localizer with scan (Phase 4 Integration)
                        if localizer:
                            corrected_pose, icp_success, match_info = localizer.update_scan(lidar_scan_obj)

                            # Log ICP match results (throttled)
                            if icp_success and esp32_communication_thread.lidar_scan_count % 10 == 0:
                                transform = match_info['transform']
                                quality = match_info['quality']
                                print(f'  ICP: dx={transform[0]:.3f}m, dy={transform[1]:.3f}m, '
                                      f'dθ={np.rad2deg(transform[2]):.1f}°, '
                                      f'err={match_info["error"]:.4f}m, '
                                      f'corresp={quality["correspondence_ratio"]:.1%}')

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

            # Handle STATUS messages (robot configuration response)
            elif msg_type == MessageType.MSG_STATUS:
                try:
                    from robotlink import StatusPayload
                    status = StatusPayload.unpack(payload)

                    print(f'Received STATUS: Kp={status.kp:.2f}, Ki={status.ki:.2f}, Kd={status.kd:.2f}, '
                          f'DB=[{status.deadband[0]:.1f}, {status.deadband[1]:.1f}, '
                          f'{status.deadband[2]:.1f}, {status.deadband[3]:.1f}]')

                    # Broadcast to all connected WebSocket clients
                    socketio.emit('robot_status', {
                        'pid': {
                            'kp': status.kp,
                            'ki': status.ki,
                            'kd': status.kd
                        },
                        'deadband': {
                            'left_forward': status.deadband[0],
                            'left_reverse': status.deadband[1],
                            'right_forward': status.deadband[2],
                            'right_reverse': status.deadband[3]
                        },
                        'pid_enabled': status.pid_enabled,
                        'stream_enabled': status.stream_enabled,
                        'stream_interval': status.stream_interval,
                        'uptime': status.uptime
                    })

                except Exception as e:
                    print(f'Error parsing STATUS: {e}')
                    import traceback
                    traceback.print_exc()

            # Log synchronizer and localization stats every 10 seconds
            if data_synchronizer and (time.time() - last_stats_time) >= 10.0:
                stats = data_synchronizer.get_stats()
                print(f'\nSLAM Sync Stats: '
                      f'odom_buf={stats["odom_buffer_size"]}/{stats["odom_buffer_capacity"]}, '
                      f'lidar_buf={stats["lidar_buffer_size"]}/{stats["lidar_buffer_capacity"]}, '
                      f'synced={stats["synced_generated"]}, '
                      f'failed={stats["interpolation_failed"]}')

                # Log integrated localization stats (Phase 4)
                if localizer:
                    loc_stats = localizer.get_statistics()
                    dr_pose = localizer.get_dead_reckoning_pose()
                    corr_pose = localizer.get_corrected_pose()

                    print(f'Dead Reckoning Pose: '
                          f'pos=({dr_pose.x:.3f}, {dr_pose.y:.3f})m, '
                          f'θ={np.rad2deg(dr_pose.theta):.1f}°, '
                          f'dist={loc_stats["total_distance_traveled"]:.2f}m')

                    print(f'ICP-Corrected Pose: '
                          f'pos=({corr_pose.x:.3f}, {corr_pose.y:.3f})m, '
                          f'θ={np.rad2deg(corr_pose.theta):.1f}°')

                    print(f'ICP Stats: '
                          f'scans={loc_stats["scans_processed"]}, '
                          f'success_rate={loc_stats["icp_success_rate"]:.1%}, '
                          f'drift={loc_stats["current_drift"]["distance_drift"]:.3f}m')

                last_stats_time = time.time()

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


@socketio.on('request_status')
def handle_request_status():
    """Request current robot status (PID, deadband, etc.)"""
    global robot

    if robot is None:
        emit('error', {'message': 'ESP32 not connected'})
        return

    try:
        robot.request_status()
        print('Requested robot status')

    except Exception as e:
        print(f'Error requesting status: {e}')
        emit('error', {'message': str(e)})


@socketio.on('set_pid')
def handle_set_pid(data):
    """Set PID parameters"""
    global robot

    if robot is None:
        emit('error', {'message': 'ESP32 not connected'})
        return

    try:
        kp = float(data.get('kp'))
        ki = float(data.get('ki'))
        kd = float(data.get('kd'))

        robot.set_pid(kp, ki, kd)
        print(f'Set PID: Kp={kp}, Ki={ki}, Kd={kd}')

        # Request updated status to confirm
        time.sleep(0.1)
        robot.request_status()

    except Exception as e:
        print(f'Error setting PID: {e}')
        emit('error', {'message': str(e)})


@socketio.on('set_deadband')
def handle_set_deadband(data):
    """Set deadband parameters"""
    global robot

    if robot is None:
        emit('error', {'message': 'ESP32 not connected'})
        return

    try:
        left_fwd = float(data.get('left_forward'))
        left_rev = float(data.get('left_reverse'))
        right_fwd = float(data.get('right_forward'))
        right_rev = float(data.get('right_reverse'))

        robot.set_deadband(left_fwd, left_rev, right_fwd, right_rev)
        print(f'Set Deadband: L_fwd={left_fwd}, L_rev={left_rev}, R_fwd={right_fwd}, R_rev={right_rev}')

        # Request updated status to confirm
        time.sleep(0.1)
        robot.request_status()

    except Exception as e:
        print(f'Error setting deadband: {e}')
        emit('error', {'message': str(e)})


@socketio.on('save_config')
def handle_save_config():
    """Save configuration to EEPROM"""
    global robot

    if robot is None:
        emit('error', {'message': 'ESP32 not connected'})
        return

    try:
        robot.save_config()
        print('Saved configuration to EEPROM')
        emit('config_saved', {'success': True})

    except Exception as e:
        print(f'Error saving config: {e}')
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
