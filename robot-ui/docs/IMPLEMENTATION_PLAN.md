# Robot Control UI - Implementation Plan

## Overview

This document provides a detailed implementation plan for building a real-time web-based robot control dashboard with motor control, LIDAR visualization, and occupancy grid mapping.

## UI Layout Design

### 3-Panel Responsive Layout

```
┌─────────────────────────────────────────────────────────────────────┐
│  Robot Control Dashboard                    Status: Connected  ●    │
│  ESP32: 192.168.68.52:5000                  Latency: 12ms           │
├───────────────┬─────────────────────┬───────────────────────────────┤
│               │                     │                               │
│  CONTROL      │   LIDAR SCAN        │   OCCUPANCY MAP               │
│  PANEL        │   (360° Polar)      │   (Grid SLAM)                 │
│  (300px)      │   (400x400px)       │   (500x500px)                 │
│               │                     │                               │
│ ┌───────────┐ │        N            │         ▲                     │
│ │  FORWARD  │ │        │            │         │  Y                  │
│ │     ▲     │ │    W ──┼── E        │         │                     │
│ └───────────┘ │        │            │         │                     │
│               │        S            │   ◄─────┼─────► X             │
│ ┌──┬────┬──┐ │                     │         │                     │
│ │◄─┤STOP├─►│ │   ● LIDAR           │         │                     │
│ └──┴────┴──┘ │   • • •             │    [Robot Pose]               │
│               │  • • • •            │         ▼                     │
│ ┌───────────┐ │ • • • • •           │                               │
│ │ BACKWARD  │ │  • • • •            │   ⬛ Obstacles                │
│ │     ▼     │ │   • • •             │   ⬜ Free space               │
│ └───────────┘ │    • •              │   ⬜ Unknown                  │
│               │                     │                               │
│  Velocity:    │   RPM: 220 ✓        │   Scale: 5m x 5m              │
│  ┌─────────┐  │   Readings: 38/40   │   Resolution: 5cm/cell        │
│  │ ▓▓▓░░░  │  │   Rate: 5 Hz        │                               │
│  └─────────┘  │                     │   ┌─────────┐  ┌─────────┐    │
│  0.3 m/s      │   [Enable Motor]    │   │  Clear  │  │  Save   │    │
│               │   [Set RPM: 220]    │   └─────────┘  └─────────┘    │
│  Odometry:    │                     │                               │
│  x:  1.234 m  │   LIDAR Control:    │   Map Stats:                  │
│  y:  0.567 m  │   ☑ Motor ON        │   Cells: 10000                │
│  θ:  45.2°    │   RPM: [220    ]    │   Occupied: 234               │
│               │   [Start] [Stop]    │   Free: 1890                  │
│  Velocity:    │                     │   Unknown: 7876               │
│  L: 0.28 m/s  │   Status:           │                               │
│  R: 0.32 m/s  │   ✓ Scan active     │   ┌───────────────────────┐  │
│               │   ✓ Data streaming  │   │  Path Planning (TBD)  │  │
│  Encoders:    │   Packets: 1234     │   └───────────────────────┘  │
│  L: 5432      │   Errors: 0         │                               │
│  R: 5489      │                     │                               │
│               │                     │                               │
│  PWM:         │                     │                               │
│  L: 156       │                     │                               │
│  R: 162       │                     │                               │
│               │                     │                               │
└───────────────┴─────────────────────┴───────────────────────────────┘
```

## Implementation Steps

### Phase 1: Backend Foundation (Server Setup)

#### Step 1.1: Create Flask Server with SocketIO

**File**: `server/robot_ui_server.py`

```python
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import threading
import time

app = Flask(__name__,
            static_folder='../static',
            template_folder='../static')
app.config['SECRET_KEY'] = 'robot-ui-secret-key'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global state
robot_connected = False
esp32_client = None

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect')
def handle_connect():
    print(f'Client connected: {request.sid}')
    emit('connection_status', {'status': 'connected', 'esp32': robot_connected})

@socketio.on('disconnect')
def handle_disconnect():
    print(f'Client disconnected: {request.sid}')

if __name__ == '__main__':
    print('Starting Robot Control UI Server...')
    print('Access dashboard at: http://localhost:5000')
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
```

**Dependencies**: Create `server/requirements.txt`
```
Flask==3.0.0
Flask-SocketIO==5.3.5
python-socketio==5.10.0
numpy==1.26.2
```

#### Step 1.2: Integrate RobotLink UDP Communication

**Action**: Copy `robotlink.py` from existing project
```bash
cp "/Users/magnus/Documents/PlatformIO/Projects/round robot motor firmware/python/robotlink.py" \
   "/Users/magnus/Documents/PlatformIO/Projects/robot-ui/server/robotlink.py"
```

**Add to `robot_ui_server.py`**:
```python
from robotlink import RobotLink, MessageType, OdomPayload, LidarScanPayload, LidarStatusPayload
import struct

# Initialize RobotLink
ESP32_HOST = '192.168.68.52'
ESP32_PORT = 5000
robot = None
running = False

def init_robot_connection():
    global robot, robot_connected
    try:
        robot = RobotLink(host=ESP32_HOST, port=ESP32_PORT)
        robot_connected = True
        print(f'✓ Connected to ESP32 at {ESP32_HOST}:{ESP32_PORT}')

        # Enable odometry streaming
        robot.enable_stream(True, interval_ms=50)  # 20 Hz

        # Enable LIDAR
        robot.lidar_enable(True)
        robot.lidar_set_rpm(220)

        return True
    except Exception as e:
        print(f'✗ Failed to connect to ESP32: {e}')
        robot_connected = False
        return False
```

#### Step 1.3: Background Thread for ESP32 Communication

```python
def esp32_communication_thread():
    """Background thread that polls ESP32 for messages and forwards to WebSocket clients"""
    global running, robot

    print('ESP32 communication thread started')

    while running:
        if robot is None:
            time.sleep(1)
            continue

        try:
            # Poll for incoming message from ESP32
            result = robot.receive_frame()
            if result is None:
                time.sleep(0.01)  # Small delay to prevent CPU spinning
                continue

            msg_type, payload = result

            # Handle different message types
            if msg_type == MessageType.MSG_ODOM:
                # Parse odometry
                odom = OdomPayload.unpack(payload)
                socketio.emit('odom_update', {
                    'encoder_left': odom.encoder_left,
                    'encoder_right': odom.encoder_right,
                    'vel_left': odom.vel_left,
                    'vel_right': odom.vel_right,
                    'pwm_left': odom.pwm_left,
                    'pwm_right': odom.pwm_right,
                    'timestamp': odom.timestamp
                })

            elif msg_type == MessageType.MSG_LIDAR_SCAN:
                # Parse LIDAR scan
                scan = LidarScanPayload.unpack(payload)
                socketio.emit('lidar_scan', {
                    'timestamp': scan.timestamp,
                    'rpm': scan.rpm,
                    'start_angle': scan.start_angle,
                    'readings': [{
                        'distance_mm': r.distance_mm,
                        'signal_strength': r.signal_strength,
                        'invalid': r.invalid,
                        'warning': r.warning
                    } for r in scan.readings]
                })

                # Update occupancy grid (will implement in Step 2)
                # occupancy_grid.update(scan, current_robot_pose)

            elif msg_type == MessageType.MSG_LIDAR_STATUS:
                # Parse LIDAR status
                status = LidarStatusPayload.unpack(payload)
                socketio.emit('lidar_status', {
                    'current_rpm': status.current_rpm,
                    'target_rpm': status.target_rpm,
                    'motor_running': status.motor_running,
                    'packets_received': status.packets_received,
                    'packets_invalid': status.packets_invalid,
                    'scans_complete': status.scans_complete,
                    'timestamp': status.timestamp
                })

        except Exception as e:
            print(f'Error in ESP32 thread: {e}')
            time.sleep(0.1)

    print('ESP32 communication thread stopped')

# Start background thread
def start_background_threads():
    global running
    running = True

    # Initialize robot connection
    init_robot_connection()

    # Start ESP32 communication thread
    esp32_thread = threading.Thread(target=esp32_communication_thread, daemon=True)
    esp32_thread.start()

# Call this when server starts
start_background_threads()
```

#### Step 1.4: WebSocket Event Handlers for UI Commands

```python
@socketio.on('motor_command')
def handle_motor_command(data):
    """Handle motor control commands from UI"""
    global robot
    if robot is None:
        emit('error', {'message': 'Robot not connected'})
        return

    command_type = data.get('type')

    if command_type == 'velocity':
        # Set wheel velocities
        vel_left = data.get('vel_left', 0.0)
        vel_right = data.get('vel_right', 0.0)
        robot.set_velocity(vel_left, vel_right)
        print(f'Motor command: L={vel_left} R={vel_right} m/s')

    elif command_type == 'cmd_vel':
        # Set linear + angular velocity
        v = data.get('v', 0.0)  # m/s
        w = data.get('w', 0.0)  # rad/s
        robot.cmd_vel(v, w)
        print(f'Cmd vel: v={v} m/s, w={w} rad/s')

    elif command_type == 'stop':
        robot.stop()
        print('Motor STOP command')

@socketio.on('lidar_command')
def handle_lidar_command(data):
    """Handle LIDAR control commands from UI"""
    global robot
    if robot is None:
        emit('error', {'message': 'Robot not connected'})
        return

    command_type = data.get('type')

    if command_type == 'enable':
        enable = data.get('enable', False)
        robot.lidar_enable(enable)
        print(f'LIDAR motor: {\"ON\" if enable else \"OFF\"}')

    elif command_type == 'set_rpm':
        rpm = data.get('rpm', 220)
        robot.lidar_set_rpm(rpm)
        print(f'LIDAR target RPM: {rpm}')
```

### Phase 2: Occupancy Grid Backend

#### Step 2.1: Create Occupancy Grid Class

**File**: `server/occupancy_grid.py`

```python
import numpy as np
import math

class OccupancyGrid:
    """
    2D occupancy grid map for robot navigation
    Uses probabilistic occupancy with Bresenham ray tracing
    """

    def __init__(self, width=100, height=100, resolution=0.05):
        """
        Args:
            width: Grid width in cells
            height: Grid height in cells
            resolution: Cell size in meters (default 5cm)
        """
        self.width = width
        self.height = height
        self.resolution = resolution  # meters per cell

        # Grid: 0.5 = unknown, 0.0 = free, 1.0 = occupied
        self.grid = np.full((height, width), 0.5, dtype=np.float32)

        # Robot pose in world frame (meters, meters, radians)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

    def update_robot_pose(self, x, y, theta):
        """Update robot pose from odometry"""
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta

    def world_to_grid(self, x, y):
        """Convert world coordinates (meters) to grid indices"""
        # Origin at center of grid
        center_x = self.width // 2
        center_y = self.height // 2

        grid_x = int(center_x + x / self.resolution)
        grid_y = int(center_y - y / self.resolution)  # Y-axis inverted

        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        """Convert grid indices to world coordinates (meters)"""
        center_x = self.width // 2
        center_y = self.height // 2

        x = (grid_x - center_x) * self.resolution
        y = (center_y - grid_y) * self.resolution

        return x, y

    def is_valid_cell(self, grid_x, grid_y):
        """Check if grid coordinates are within bounds"""
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height

    def bresenham_line(self, x0, y0, x1, y1):
        """
        Bresenham's line algorithm - returns all cells along line from (x0,y0) to (x1,y1)
        """
        cells = []

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0

        while True:
            cells.append((x, y))

            if x == x1 and y == y1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return cells

    def update_from_lidar_scan(self, lidar_scan):
        """
        Update grid from LIDAR scan using ray tracing

        Args:
            lidar_scan: dict with 'start_angle', 'readings' [{distance_mm, invalid, ...}]
        """
        start_angle = lidar_scan['start_angle']  # degrees
        readings = lidar_scan['readings']

        # Robot position in grid coordinates
        robot_gx, robot_gy = self.world_to_grid(self.robot_x, self.robot_y)

        if not self.is_valid_cell(robot_gx, robot_gy):
            return  # Robot outside map

        for i, reading in enumerate(readings):
            if reading['invalid']:
                continue

            # Calculate beam angle in world frame
            beam_angle_deg = (start_angle + i) % 360
            beam_angle_rad = math.radians(beam_angle_deg) + self.robot_theta

            # Convert LIDAR reading to world coordinates
            distance_m = reading['distance_mm'] / 1000.0

            # Hit point in world frame
            hit_x = self.robot_x + distance_m * math.cos(beam_angle_rad)
            hit_y = self.robot_y + distance_m * math.sin(beam_angle_rad)

            # Convert to grid coordinates
            hit_gx, hit_gy = self.world_to_grid(hit_x, hit_y)

            if not self.is_valid_cell(hit_gx, hit_gy):
                continue

            # Ray trace from robot to hit point
            ray_cells = self.bresenham_line(robot_gx, robot_gy, hit_gx, hit_gy)

            # Update cells along ray
            for j, (gx, gy) in enumerate(ray_cells):
                if not self.is_valid_cell(gx, gy):
                    continue

                if j < len(ray_cells) - 1:
                    # Free space along ray
                    self.grid[gy, gx] = max(0.0, self.grid[gy, gx] - 0.05)
                else:
                    # Occupied endpoint
                    self.grid[gy, gx] = min(1.0, self.grid[gy, gx] + 0.3)

    def get_grid_data(self):
        """Return grid as JSON-serializable format for WebSocket"""
        # Convert to 3-level representation: 0=free, 1=unknown, 2=occupied
        display_grid = np.zeros_like(self.grid, dtype=np.uint8)
        display_grid[self.grid < 0.3] = 0  # Free
        display_grid[(self.grid >= 0.3) & (self.grid <= 0.7)] = 1  # Unknown
        display_grid[self.grid > 0.7] = 2  # Occupied

        return {
            'width': self.width,
            'height': self.height,
            'resolution': self.resolution,
            'robot_pose': {
                'x': self.robot_x,
                'y': self.robot_y,
                'theta': self.robot_theta
            },
            'grid': display_grid.tolist()  # Convert numpy array to list
        }

    def clear(self):
        """Reset grid to unknown"""
        self.grid.fill(0.5)
```

#### Step 2.2: Integrate Occupancy Grid into Server

**Add to `robot_ui_server.py`**:
```python
from occupancy_grid import OccupancyGrid

# Create global occupancy grid
occupancy_grid = OccupancyGrid(width=100, height=100, resolution=0.05)

# In esp32_communication_thread(), when receiving MSG_LIDAR_SCAN:
elif msg_type == MessageType.MSG_LIDAR_SCAN:
    scan = LidarScanPayload.unpack(payload)
    scan_dict = {
        'timestamp': scan.timestamp,
        'rpm': scan.rpm,
        'start_angle': scan.start_angle,
        'readings': [{
            'distance_mm': r.distance_mm,
            'signal_strength': r.signal_strength,
            'invalid': r.invalid,
            'warning': r.warning
        } for r in scan.readings]
    }

    # Broadcast LIDAR scan to UI
    socketio.emit('lidar_scan', scan_dict)

    # Update occupancy grid
    occupancy_grid.update_from_lidar_scan(scan_dict)

    # Broadcast map update (throttle to ~2 Hz)
    if scan.timestamp % 500 < 50:  # Every 500ms
        socketio.emit('map_update', occupancy_grid.get_grid_data())

# When receiving MSG_ODOM, update robot pose:
elif msg_type == MessageType.MSG_ODOM:
    odom = OdomPayload.unpack(payload)

    # Update occupancy grid with robot pose
    # (Assuming odometry provides x, y, theta - adjust based on your ODOM format)
    # For now, we'll track pose from encoder data
    # occupancy_grid.update_robot_pose(x, y, theta)

    socketio.emit('odom_update', {...})

# WebSocket handler for clearing map
@socketio.on('clear_map')
def handle_clear_map():
    global occupancy_grid
    occupancy_grid.clear()
    emit('map_update', occupancy_grid.get_grid_data(), broadcast=True)
    print('Map cleared')
```

### Phase 3: Frontend Development

#### Step 3.1: HTML Structure

**File**: `static/index.html`

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Control Dashboard</title>
    <link rel="stylesheet" href="/static/css/style.css">
    <script src="https://cdn.socket.io/4.5.4/socket.io.min.js"></script>
</head>
<body>
    <!-- Top Status Bar -->
    <header class="status-bar">
        <h1>Robot Control Dashboard</h1>
        <div class="status-info">
            <span id="esp32-status">ESP32: <span class="status-indicator offline"></span> Connecting...</span>
            <span id="latency">Latency: --ms</span>
        </div>
    </header>

    <!-- Main 3-Panel Layout -->
    <main class="dashboard">
        <!-- Left Panel: Motor Control -->
        <section class="panel control-panel">
            <h2>Motor Control</h2>

            <!-- Directional Controls -->
            <div class="direction-controls">
                <button class="btn-direction btn-forward" data-direction="forward">
                    ▲<br>FORWARD
                </button>
                <div class="btn-row">
                    <button class="btn-direction btn-left" data-direction="left">◄<br>LEFT</button>
                    <button class="btn-stop" id="btn-stop">STOP</button>
                    <button class="btn-direction btn-right" data-direction="right">►<br>RIGHT</button>
                </div>
                <button class="btn-direction btn-backward" data-direction="backward">
                    ▼<br>BACKWARD
                </button>
            </div>

            <!-- Velocity Slider -->
            <div class="control-group">
                <label for="velocity-slider">Velocity:</label>
                <input type="range" id="velocity-slider" min="0.1" max="0.5" step="0.05" value="0.3">
                <span id="velocity-value">0.3 m/s</span>
            </div>

            <!-- Odometry Display -->
            <div class="odometry-display">
                <h3>Odometry</h3>
                <div class="odom-row">
                    <span>x:</span> <span id="odom-x">0.000 m</span>
                </div>
                <div class="odom-row">
                    <span>y:</span> <span id="odom-y">0.000 m</span>
                </div>
                <div class="odom-row">
                    <span>θ:</span> <span id="odom-theta">0.0°</span>
                </div>

                <h4>Velocity</h4>
                <div class="odom-row">
                    <span>L:</span> <span id="vel-left">0.00 m/s</span>
                </div>
                <div class="odom-row">
                    <span>R:</span> <span id="vel-right">0.00 m/s</span>
                </div>

                <h4>Encoders</h4>
                <div class="odom-row">
                    <span>L:</span> <span id="enc-left">0</span>
                </div>
                <div class="odom-row">
                    <span>R:</span> <span id="enc-right">0</span>
                </div>

                <h4>PWM</h4>
                <div class="odom-row">
                    <span>L:</span> <span id="pwm-left">0</span>
                </div>
                <div class="odom-row">
                    <span>R:</span> <span id="pwm-right">0</span>
                </div>
            </div>

            <!-- Keyboard Hint -->
            <div class="keyboard-hint">
                <small>Keyboard: WASD or Arrow Keys, SPACE to stop</small>
            </div>
        </section>

        <!-- Center Panel: LIDAR Visualization -->
        <section class="panel lidar-panel">
            <h2>LIDAR Scan (360°)</h2>
            <canvas id="lidar-canvas" width="400" height="400"></canvas>

            <!-- LIDAR Stats -->
            <div class="lidar-stats">
                <div class="stat-row">
                    <span>RPM:</span> <span id="lidar-rpm">--</span>
                </div>
                <div class="stat-row">
                    <span>Readings:</span> <span id="lidar-readings">--/--</span>
                </div>
                <div class="stat-row">
                    <span>Rate:</span> <span id="lidar-rate">-- Hz</span>
                </div>
            </div>

            <!-- LIDAR Controls -->
            <div class="lidar-controls">
                <h3>LIDAR Control</h3>
                <div class="control-group">
                    <label>
                        <input type="checkbox" id="lidar-enable" checked>
                        Motor Enabled
                    </label>
                </div>
                <div class="control-group">
                    <label for="lidar-rpm-slider">Target RPM:</label>
                    <input type="range" id="lidar-rpm-slider" min="200" max="300" step="10" value="220">
                    <span id="lidar-rpm-value">220</span>
                </div>
            </div>

            <!-- LIDAR Status -->
            <div class="lidar-status">
                <div class="status-item">
                    <span id="lidar-motor-status" class="indicator">●</span> Motor
                </div>
                <div class="status-item">
                    <span>Packets:</span> <span id="lidar-packets">0</span>
                </div>
                <div class="status-item">
                    <span>Errors:</span> <span id="lidar-errors">0</span>
                </div>
            </div>
        </section>

        <!-- Right Panel: Occupancy Map -->
        <section class="panel map-panel">
            <h2>Occupancy Map</h2>
            <canvas id="map-canvas" width="500" height="500"></canvas>

            <!-- Map Legend -->
            <div class="map-legend">
                <div class="legend-item">
                    <span class="legend-color occupied"></span> Occupied
                </div>
                <div class="legend-item">
                    <span class="legend-color free"></span> Free
                </div>
                <div class="legend-item">
                    <span class="legend-color unknown"></span> Unknown
                </div>
            </div>

            <!-- Map Stats -->
            <div class="map-stats">
                <div class="stat-row">
                    <span>Scale:</span> <span id="map-scale">5m x 5m</span>
                </div>
                <div class="stat-row">
                    <span>Resolution:</span> <span id="map-resolution">5cm/cell</span>
                </div>
                <div class="stat-row">
                    <span>Cells:</span> <span id="map-cells">10000</span>
                </div>
            </div>

            <!-- Map Controls -->
            <div class="map-controls">
                <button class="btn-secondary" id="btn-clear-map">Clear Map</button>
                <button class="btn-secondary" id="btn-save-map">Save Map</button>
            </div>
        </section>
    </main>

    <!-- Load JavaScript -->
    <script src="/static/js/app.js"></script>
    <script src="/static/js/lidar_viz.js"></script>
    <script src="/static/js/occupancy_map.js"></script>
    <script src="/static/js/controls.js"></script>
</body>
</html>
```

*Continuing in next message due to length...*
