"""
Robot Control UI Server
Flask + SocketIO server for real-time robot control
"""

from flask import Flask, send_from_directory, request
from flask_socketio import SocketIO, emit
import threading
import time
import sys
import os
import numpy as np

# Add server directory to Python path for robotlink import
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robotlink import RobotLink, MessageType, OdomPayload, LidarScanPayload, AckPayload, NackPayload, StatusExtendedPayload

# SLAM imports (Phase 2, 3, 4, & 5 Integration)
from slam.data_sync import DataSynchronizer
from slam.sensor_data import OdomReading, LidarScan, LidarReading
from slam.motion_model import RobotParameters
from slam.localization import IntegratedLocalizer
from slam.occupancy_grid import OccupancyGrid

# Configuration
ESP32_HOST = '192.168.68.73'
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

# SLAM state (Phase 2, 3, 4, & 5 Integration)
data_synchronizer = None
localizer = None  # Integrated localizer (dead reckoning + ICP)
occupancy_grid = None  # Occupancy grid mapper (Phase 5)
last_map_broadcast_time = 0.0  # Throttle map updates to 2 Hz

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

# Robot configuration (legacy - not used, real values loaded from robot_config.json)
# These are kept for backwards compatibility but should match config file:
WHEEL_DIAMETER = 0.079  # meters (79mm, measured with robot weight)
TICKS_PER_REV = 714  # 2X quadrature decoding, calibrated
METERS_PER_TICK = (WHEEL_DIAMETER * 3.14159) / TICKS_PER_REV

def init_robot_connection():
    """Initialize connection to ESP32"""
    global robot, robot_connected, data_synchronizer, localizer, occupancy_grid

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
        # Load robot parameters from file or use defaults
        config_file = os.path.join(os.path.dirname(__file__), 'robot_config.json')
        robot_params = RobotParameters.load(config_file)
        print(f'✓ Robot parameters loaded: {robot_params}')

        localizer = IntegratedLocalizer(robot_params=robot_params)
        print('✓ Integrated localizer initialized (dead reckoning + ICP)')

        # Initialize occupancy grid (Phase 5)
        occupancy_grid = OccupancyGrid(width=100, height=100, resolution=0.05)
        print('✓ Occupancy grid initialized (100×100 @ 5cm resolution, 5m×5m coverage)')

        robot_connected = True

        # Broadcast connection status to all connected clients
        socketio.emit('connection_status', {
            'status': 'connected',
            'esp32': True,
            'esp32_host': ESP32_HOST,
            'esp32_port': ESP32_PORT
        })

        return True

    except Exception as e:
        print(f'✗ Failed to connect to ESP32: {e}')
        robot_connected = False

        # Broadcast connection failure to all connected clients
        socketio.emit('connection_status', {
            'status': 'disconnected',
            'esp32': False,
            'esp32_host': ESP32_HOST,
            'esp32_port': ESP32_PORT
        })

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

                        # Emit pose update (Story 5.1) - with three odometry sources
                        dr_pose = localizer.get_dead_reckoning_pose()
                        icp_pose = localizer.get_corrected_pose()
                        drift = localizer.get_pose_drift()

                        socketio.emit('pose_update', {
                            'dead_reckoning': {
                                'x': dr_pose.x,
                                'y': -dr_pose.y,  # Negate Y to fix mirroring
                                'theta_deg': -np.rad2deg(dr_pose.theta)  # Negate heading to fix mirroring
                            },
                            'arduino_odo': {
                                'x': odom.x_mm / 1000.0,  # Convert mm to meters
                                'y': odom.y_mm / 1000.0,
                                'theta_deg': odom.theta_mrad / 1000.0 * 180.0 / np.pi
                            },
                            'icp_corrected': {
                                'x': icp_pose.x,
                                'y': -icp_pose.y,  # Negate Y to fix mirroring
                                'theta_deg': -np.rad2deg(icp_pose.theta)  # Negate heading to fix mirroring
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

                            # Update occupancy grid with scan (Phase 5)
                            if occupancy_grid:
                                # Use ICP-corrected pose for mapping (more accurate)
                                occupancy_grid.update_from_scan(corrected_pose, lidar_scan_obj)

                                # Broadcast map update (throttled to 2 Hz to reduce bandwidth)
                                global last_map_broadcast_time
                                current_time = time.time()
                                if current_time - last_map_broadcast_time >= 0.5:  # 2 Hz
                                    map_data = occupancy_grid.serialize_for_ui(corrected_pose)
                                    socketio.emit('map_update', map_data)
                                    last_map_broadcast_time = current_time

                                    # Log mapping stats (every 10 map updates)
                                    if esp32_communication_thread.lidar_scan_count % 50 == 0:
                                        stats = occupancy_grid.get_stats()
                                        print(f'  Map: {stats["occupied_cells"]} occupied, '
                                              f'{stats["free_cells"]} free, '
                                              f'{stats["scans_processed"]} scans')

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

                    # Note: Arduino currently only sends pidLeft values in STATUS message
                    # Both motors may have different PID values but we only receive pidLeft
                    # TODO: Add MSG_STATUS_EXTENDED to get per-motor PID values

                    # Broadcast to all connected WebSocket clients
                    socketio.emit('robot_status', {
                        'pid': {
                            'kp': status.kp,
                            'ki': status.ki,
                            'kd': status.kd
                        },
                        'pid_left': {
                            'kp': status.kp,
                            'ki': status.ki,
                            'kd': status.kd
                        },
                        'pid_right': {
                            'kp': status.kp,  # Same as left - not actually from right motor
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
                        'uptime': status.uptime,
                        'heading_hold_kp': 15.0  # Default value - not yet sent by Arduino
                    })

                except Exception as e:
                    print(f'Error parsing STATUS: {e}')
                    import traceback
                    traceback.print_exc()

            # Handle STATUS_EXTENDED messages (per-motor PID status)
            elif msg_type == MessageType.MSG_STATUS_EXTENDED:
                try:
                    status = StatusExtendedPayload.unpack(payload)

                    print(f'Received STATUS_EXTENDED: '
                          f'L({status.left_kp:.2f}, {status.left_ki:.2f}, {status.left_kd:.2f}) '
                          f'R({status.right_kp:.2f}, {status.right_ki:.2f}, {status.right_kd:.2f}) '
                          f'HeadingHoldKp={status.heading_hold_kp:.2f}')

                    # Broadcast to all connected WebSocket clients
                    socketio.emit('robot_status', {
                        'pid_left': {
                            'kp': status.left_kp,
                            'ki': status.left_ki,
                            'kd': status.left_kd
                        },
                        'pid_right': {
                            'kp': status.right_kp,
                            'ki': status.right_ki,
                            'kd': status.right_kd
                        },
                        'deadband': {
                            'left_forward': status.deadband[0],
                            'left_reverse': status.deadband[1],
                            'right_forward': status.deadband[2],
                            'right_reverse': status.deadband[3]
                        },
                        'heading_hold_kp': status.heading_hold_kp,
                        'pid_enabled': status.pid_enabled,
                        'stream_enabled': status.stream_enabled,
                        'stream_interval': status.stream_interval,
                        'uptime': status.uptime
                    })

                except Exception as e:
                    print(f'Error parsing STATUS_EXTENDED: {e}')
                    import traceback
                    traceback.print_exc()

            # Handle ACK messages
            elif msg_type == MessageType.MSG_ACK:
                try:
                    ack = AckPayload.unpack(payload)
                    msg_name = MessageType(ack.original_msg_type).name if ack.original_msg_type in MessageType._value2member_map_ else f'0x{ack.original_msg_type:02X}'
                    status_str = 'SUCCESS' if ack.status == 0 else f'ERROR {ack.status}'
                    print(f'✓ ACK received for {msg_name}: {status_str}')

                    # Broadcast to UI
                    socketio.emit('command_ack', {
                        'message_type': msg_name,
                        'success': ack.status == 0,
                        'error_code': ack.status
                    })

                except Exception as e:
                    print(f'Error parsing ACK: {e}')

            # Handle NACK messages
            elif msg_type == MessageType.MSG_NACK:
                try:
                    nack = NackPayload.unpack(payload)
                    msg_name = MessageType(nack.original_msg_type).name if nack.original_msg_type in MessageType._value2member_map_ else f'0x{nack.original_msg_type:02X}'
                    print(f'✗ NACK received for {msg_name}: Error code {nack.error_code}')

                    # Error code descriptions
                    error_descriptions = {
                        0x01: 'Invalid payload',
                        0x02: 'Out of range',
                        0x03: 'EEPROM write failed',
                        0x04: 'EEPROM read failed',
                        0x05: 'Not implemented',
                        0x06: 'Timeout',
                        0xFF: 'Unknown error'
                    }
                    error_desc = error_descriptions.get(nack.error_code, f'Error {nack.error_code}')

                    # Broadcast to UI
                    socketio.emit('command_nack', {
                        'message_type': msg_name,
                        'error_code': nack.error_code,
                        'error_description': error_desc
                    })

                except Exception as e:
                    print(f'Error parsing NACK: {e}')

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
    client_id = request.sid
    print(f'✓ Client connected: {client_id}')
    print(f'  ESP32 status: {"Connected" if robot_connected else "Disconnected"}')

    # Send connection status
    emit('connection_status', {
        'status': 'connected',
        'esp32': robot_connected,
        'esp32_host': ESP32_HOST,
        'esp32_port': ESP32_PORT
    })
    print(f'  → Sent connection_status: esp32={robot_connected}')


@socketio.on('disconnect')
def handle_disconnect():
    """Client disconnected from WebSocket"""
    client_id = request.sid
    print(f'✗ Client disconnected: {client_id}')


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
    """Set PID parameters (both motors)"""
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


@socketio.on('set_pid_per_motor')
def handle_set_pid_per_motor(data):
    """Set per-motor PID parameters"""
    global robot

    if robot is None:
        emit('error', {'message': 'ESP32 not connected'})
        return

    try:
        left_kp = float(data.get('leftKp'))
        left_ki = float(data.get('leftKi'))
        left_kd = float(data.get('leftKd'))
        right_kp = float(data.get('rightKp'))
        right_ki = float(data.get('rightKi'))
        right_kd = float(data.get('rightKd'))

        robot.set_pid_per_motor(left_kp, left_ki, left_kd, right_kp, right_ki, right_kd)
        print(f'Set per-motor PID: Left(Kp={left_kp}, Ki={left_ki}, Kd={left_kd}) Right(Kp={right_kp}, Ki={right_ki}, Kd={right_kd})')

        # Request updated status to confirm
        time.sleep(0.1)
        robot.request_status()

    except Exception as e:
        print(f'Error setting per-motor PID: {e}')
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


@socketio.on('set_heading_hold_kp')
def handle_set_heading_hold_kp(data):
    """Set heading hold / angular velocity feedback gain"""
    global robot

    if robot is None:
        emit('error', {'message': 'ESP32 not connected'})
        return

    try:
        kp = float(data.get('kp'))
        robot.set_heading_hold_kp(kp)
        print(f'Set Heading Hold Kp: {kp}')

    except Exception as e:
        print(f'Error setting heading hold Kp: {e}')
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


@socketio.on('reset_pose')
def handle_reset_pose():
    """Reset robot pose to origin"""
    global localizer, robot

    if localizer is None:
        emit('error', {'message': 'Localizer not initialized'})
        return

    try:
        # Reset host localizer (dead reckoning + ICP)
        localizer.reset()

        # Reset Arduino odometry pose
        if robot is not None:
            robot.reset_pose()
            print('Reset pose to origin (0, 0, 0°) - Host & Arduino')
        else:
            print('Reset pose to origin (0, 0, 0°) - Host only (Arduino not connected)')

        # Immediately send updated pose to confirm
        dr_pose = localizer.get_dead_reckoning_pose()
        icp_pose = localizer.get_corrected_pose()
        drift = localizer.get_pose_drift()

        socketio.emit('pose_update', {
            'dead_reckoning': {
                'x': dr_pose.x,
                'y': dr_pose.y,
                'theta_deg': np.rad2deg(dr_pose.theta)
            },
            'arduino_odo': {
                'x': 0.0,
                'y': 0.0,
                'theta_deg': 0.0
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

    except Exception as e:
        print(f'Error resetting pose: {e}')
        emit('error', {'message': str(e)})


@socketio.on('zero_encoders')
def handle_zero_encoders():
    """Zero Arduino encoders and reset all odometry"""
    global localizer, robot, odom_state

    if localizer is None:
        emit('error', {'message': 'Localizer not initialized'})
        return

    try:
        # Reset host localizer (dead reckoning + ICP)
        localizer.reset()

        # Reset encoder totals
        odom_state['encoder_left_total'] = 0
        odom_state['encoder_right_total'] = 0

        # Zero Arduino encoders and pose
        if robot is not None:
            robot.zero_encoders()
            print('Zeroed encoders and reset pose to origin (0, 0, 0°) - Host & Arduino')
        else:
            print('Zeroed encoders and reset pose to origin (0, 0, 0°) - Host only (Arduino not connected)')

        # Immediately send updated pose to confirm
        dr_pose = localizer.get_dead_reckoning_pose()
        icp_pose = localizer.get_corrected_pose()
        drift = localizer.get_pose_drift()

        socketio.emit('pose_update', {
            'dead_reckoning': {
                'x': dr_pose.x,
                'y': dr_pose.y,
                'theta_deg': np.rad2deg(dr_pose.theta)
            },
            'arduino_odo': {
                'x': 0.0,
                'y': 0.0,
                'theta_deg': 0.0
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

    except Exception as e:
        print(f'Error zeroing encoders: {e}')
        emit('error', {'message': str(e)})


@socketio.on('get_robot_params')
def handle_get_robot_params():
    """Get current robot parameters"""
    global localizer

    if localizer is None:
        emit('error', {'message': 'Localizer not initialized'})
        return

    try:
        params = localizer.dead_reckoning.motion_model.params
        emit('robot_params', params.to_dict())
        print(f'Sent robot parameters: {params}')
    except Exception as e:
        print(f'Error getting robot parameters: {e}')
        emit('error', {'message': str(e)})


@socketio.on('set_robot_params')
def handle_set_robot_params(data):
    """Update robot parameters on both host and Arduino"""
    global localizer, robot

    if localizer is None:
        emit('error', {'message': 'Localizer not initialized'})
        return

    try:
        # Extract parameters (convert mm to meters)
        wheel_diameter = float(data.get('wheel_diameter_mm', 80)) / 1000.0
        wheelbase = float(data.get('wheelbase_mm', 244)) / 1000.0
        ticks_per_rev = float(data.get('ticks_per_revolution', 360))
        lidar_offset_x = float(data.get('lidar_offset_x_mm', 10)) / 1000.0
        lidar_offset_y = float(data.get('lidar_offset_y_mm', 0)) / 1000.0

        # Update HOST parameters (for host odometry)
        params = localizer.dead_reckoning.motion_model.params
        params.wheel_diameter = wheel_diameter
        params.wheelbase = wheelbase
        params.ticks_per_revolution = int(ticks_per_rev)
        params.lidar_offset_x = lidar_offset_x
        params.lidar_offset_y = lidar_offset_y

        print(f'Updated HOST robot parameters: {params}')

        # Send to ARDUINO (for Arduino odometry)
        if robot is not None:
            success = robot.set_robot_params(wheel_diameter, wheelbase, ticks_per_rev)
            if success:
                print(f'Sent robot params to Arduino: wheel_diam={wheel_diameter*1000:.1f}mm, '
                      f'wheelbase={wheelbase*1000:.1f}mm, ticks/rev={ticks_per_rev:.0f}')
            else:
                print('Warning: Failed to send robot params to Arduino')
        else:
            print('Warning: Robot not connected, params only updated on host')

        # Send back updated parameters
        emit('robot_params', params.to_dict())

    except Exception as e:
        print(f'Error setting robot parameters: {e}')
        emit('error', {'message': str(e)})


@socketio.on('save_robot_params')
def handle_save_robot_params():
    """Save robot parameters to file"""
    global localizer

    if localizer is None:
        emit('error', {'message': 'Localizer not initialized'})
        return

    try:
        config_file = os.path.join(os.path.dirname(__file__), 'robot_config.json')
        params = localizer.dead_reckoning.motion_model.params
        params.save(config_file)
        print(f'Saved robot parameters to {config_file}')
        emit('status', {'message': 'Robot parameters saved successfully'})
    except Exception as e:
        print(f'Error saving robot parameters: {e}')
        emit('error', {'message': str(e)})


@socketio.on('clear_map')
def handle_clear_map():
    """Clear occupancy grid map"""
    global occupancy_grid, last_map_broadcast_time

    if occupancy_grid is None:
        emit('error', {'message': 'Occupancy grid not initialized'})
        return

    try:
        # Clear the map
        occupancy_grid.clear()
        print('Occupancy grid cleared')

        # Get current pose from localizer
        corrected_pose = None
        if localizer:
            corrected_pose = localizer.get_corrected_pose()

        # Broadcast cleared map to all clients
        map_data = occupancy_grid.serialize_for_ui(corrected_pose)
        socketio.emit('map_update', map_data, broadcast=True)
        last_map_broadcast_time = time.time()

        emit('status', {'message': 'Map cleared successfully'})
    except Exception as e:
        print(f'Error clearing map: {e}')
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
