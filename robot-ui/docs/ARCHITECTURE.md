# Robot Control UI - System Architecture

## Overview

The Robot Control UI is a real-time web-based dashboard for controlling and monitoring a differential drive robot with LIDAR mapping capabilities. The system provides motor control, live LIDAR visualization, and occupancy grid mapping through a responsive 3-panel interface.

## System Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Web Browser (Client)                         │
│  ┌──────────────┬─────────────────────┬────────────────────────┐   │
│  │   MOTOR      │    LIDAR POLAR      │   OCCUPANCY GRID       │   │
│  │   CONTROL    │    PLOT (Canvas)    │   MAP (Canvas)         │   │
│  │              │                     │                        │   │
│  │  ▲  Forward  │   360° live scan    │  Robot pose + grid     │   │
│  │ ◄ ► L/R      │   Distance rings    │  Obstacles & free      │   │
│  │  ▼  Back     │   Color-coded       │  Path history          │   │
│  │              │                     │                        │   │
│  │  [STOP]      │   RPM: 220          │  Scale: 5m x 5m        │   │
│  │              │                     │                        │   │
│  │  Velocity:   │   Valid: 38/40      │  Clear/Save buttons    │   │
│  │  [==] 0.3m/s │                     │                        │   │
│  │              │                     │                        │   │
│  │  Odometry:   │                     │                        │   │
│  │  x:  1.2m    │                     │                        │   │
│  │  y:  0.5m    │                     │                        │   │
│  │  θ:  45°     │                     │                        │   │
│  └──────────────┴─────────────────────┴────────────────────────┘   │
│                                                                       │
│  ┌───────────────────────────────────────────────────────────────┐  │
│  │                   WebSocket Client (JavaScript)                │  │
│  │  - Real-time bidirectional communication                       │  │
│  │  - Event handlers for odom, lidar_scan, lidar_status, map     │  │
│  │  - Command senders for motor control, LIDAR control           │  │
│  └───────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    │ WebSocket (Socket.IO)
                                    │
┌─────────────────────────────────────────────────────────────────────┐
│                    Flask Server + SocketIO (Python)                  │
│  ┌───────────────────────────────────────────────────────────────┐  │
│  │                      Main Application Loop                     │  │
│  │  1. Receive RobotLink messages from ESP32                      │  │
│  │  2. Process and forward to WebSocket clients                   │  │
│  │  3. Update occupancy grid with LIDAR scans                     │  │
│  │  4. Receive commands from UI → forward to ESP32                │  │
│  └───────────────────────────────────────────────────────────────┘  │
│                                                                       │
│  ┌──────────────────┐  ┌─────────────────────┐  ┌───────────────┐  │
│  │  RobotLink       │  │  Occupancy Grid     │  │  SocketIO     │  │
│  │  UDP Client      │  │  Backend            │  │  Server       │  │
│  │                  │  │                     │  │               │  │
│  │  - CRC16 frames  │  │  - 100x100 cells    │  │  - Room-based │  │
│  │  - Binary proto  │  │  - Ray tracing      │  │  - JSON msgs  │  │
│  │  - 192.168.68.52 │  │  - Prob. updates    │  │  - Broadcast  │  │
│  └──────────────────┘  └─────────────────────┘  └───────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    │ RobotLink Protocol (UDP)
                                    │ Binary framed messages with CRC16
                                    │
┌─────────────────────────────────────────────────────────────────────┐
│                      ESP32 WiFi/UDP Bridge                           │
│              IP: 192.168.68.52:5000                                  │
│  ┌───────────────────────────────────────────────────────────────┐  │
│  │  - Receives UDP commands from host                             │  │
│  │  - Forwards commands to Arduino (Serial1 @ 9600)               │  │
│  │  - Receives LIDAR data (Serial2 @ 115200)                      │  │
│  │  - Streams odometry from Arduino to host                       │  │
│  │  - Streams LIDAR scans to host (batched 40 readings/packet)    │  │
│  └───────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
           │                                              │
           │ Serial1 @ 9600 baud                          │ Serial2 @ 115200 baud
           │                                              │ GPIO21 PWM (motor)
           │                                              │
    ┌──────▼────────┐                            ┌───────▼───────┐
    │    Arduino    │                            │  Neato XV-11  │
    │    Motor      │                            │     LIDAR     │
    │   Controller  │                            │               │
    │               │                            │  360° scans   │
    │  - PID ctrl   │                            │  220 RPM      │
    │  - Encoders   │                            │               │
    │  - Odometry   │                            │               │
    └───────────────┘                            └───────────────┘
```

## Technology Stack

### Frontend
- **HTML5**: Structure and semantics
- **CSS3**: Dark theme styling with Flexbox/Grid layout
- **JavaScript (ES6+)**: Client-side logic
- **Socket.IO Client**: Real-time WebSocket communication
- **HTML5 Canvas API**: 2D rendering for LIDAR polar plot and occupancy grid

### Backend
- **Python 3.8+**: Server runtime
- **Flask 3.0**: Web framework
- **Flask-SocketIO 5.3**: WebSocket support
- **NumPy 1.26**: Numerical operations for grid mapping
- **RobotLink**: Binary protocol library (existing)

### Communication Protocols
- **WebSocket (Socket.IO)**: Browser ↔ Server (JSON messages)
- **RobotLink over UDP**: Server ↔ ESP32 (binary framed messages)
- **Serial UART**: ESP32 ↔ Arduino/LIDAR

## Component Descriptions

### 1. Web Browser Client

**Responsibilities**:
- Render 3-panel UI with responsive layout
- Handle user input (buttons, keyboard, mouse)
- Maintain WebSocket connection to server
- Update visualizations in real-time

**Key Files**:
- `static/index.html`: Main UI structure
- `static/js/app.js`: Application logic & WebSocket handler
- `static/js/lidar_viz.js`: LIDAR polar plot renderer
- `static/js/occupancy_map.js`: Grid map renderer
- `static/js/controls.js`: Motor control handlers

### 2. Flask Server + SocketIO

**Responsibilities**:
- Serve static web assets
- Manage WebSocket connections (rooms & broadcasting)
- Bridge between WebSocket (JSON) and RobotLink (binary)
- Run background thread for ESP32 communication
- Update occupancy grid from LIDAR scans

**Key Files**:
- `server/robot_ui_server.py`: Main server application
- `server/occupancy_grid.py`: Grid mapping backend

**Background Thread**:
```python
def esp32_communication_thread():
    while running:
        # Poll for messages from ESP32
        result = robot.receive_frame()
        if result:
            msg_type, payload = result

            # Forward to WebSocket clients
            if msg_type == MSG_ODOM:
                socketio.emit('odom_update', parse_odom(payload))

            elif msg_type == MSG_LIDAR_SCAN:
                scan = parse_lidar_scan(payload)
                socketio.emit('lidar_scan', scan)
                occupancy_grid.update(scan, robot_pose)
```

### 3. RobotLink UDP Client

**Responsibilities**:
- Establish UDP connection to ESP32
- Encode/decode RobotLink binary frames
- Calculate/verify CRC16 checksums
- Maintain statistics (frames sent/received, errors)

**Protocol Format**:
```
[SOF0] [SOF1] [TYPE] [LEN] [PAYLOAD...] [CRC_LO] [CRC_HI]
 0xAA   0x55   uint8  uint8   0-255 bytes   uint16 LE
```

### 4. Occupancy Grid Backend

**Responsibilities**:
- Maintain 100x100 cell grid (5m x 5m @ 5cm resolution)
- Convert LIDAR polar readings to Cartesian coordinates
- Perform Bresenham ray tracing from robot to hit point
- Update cell probabilities (occupied/free)
- Serialize grid for WebSocket transmission

**Algorithm**:
```
For each LIDAR reading (angle, distance):
  1. Convert to Cartesian: (x, y) = polar_to_cart(angle, distance)
  2. Transform to robot frame: (x_robot, y_robot) = transform(x, y, robot_pose)
  3. Ray trace from robot to (x_robot, y_robot):
     - Mark cells along ray as FREE (prob -= 0.1)
     - Mark endpoint cell as OCCUPIED (prob += 0.3)
  4. Clamp probabilities to [0.0, 1.0]
```

## Data Flow

### 1. Odometry Stream (20 Hz)
```
Arduino → ESP32 (Serial1) → Host (UDP) → Flask → WebSocket → Browser
```

### 2. LIDAR Stream (variable rate)
```
LIDAR → ESP32 (Serial2) → Host (UDP) → Flask → {WebSocket, OccupancyGrid} → Browser
```

### 3. Motor Commands
```
Browser → WebSocket → Flask → UDP → ESP32 → Serial1 → Arduino
```

### 4. LIDAR Commands
```
Browser → WebSocket → Flask → UDP → ESP32 (PWM control)
```

## Deployment Considerations

### Local Development
```bash
cd /Users/magnus/Documents/PlatformIO/Projects/robot-ui
python3 server/robot_ui_server.py
# Open browser to http://localhost:5000
```

### Network Access
- ESP32 must be on same WiFi network as host computer
- Default ESP32 IP: 192.168.68.52:5000
- Flask server binds to 0.0.0.0:5000 (accessible from LAN)

### Performance
- Target: 20 Hz odometry updates, 5-20 Hz LIDAR visualization
- WebSocket latency: <50ms (local network)
- Grid update rate: Real-time with 40 readings/packet
- Canvas rendering: 60 FPS (requestAnimationFrame)

## Security Notes

- **No authentication**: This is a local development tool
- **Firewall**: UDP port 5000 must be open for ESP32 communication
- **Network**: Intended for trusted local network only
- **Input validation**: Server validates message types and payload lengths

## Future Enhancements

1. **SLAM Integration**: Replace simple occupancy grid with particle filter/EKF SLAM
2. **Path Planning**: Add A* or RRT path planning visualization
3. **Multi-Robot**: Support multiple robots with session management
4. **Data Logging**: Record/replay odometry and LIDAR scans
5. **Map Export**: Save maps in standard formats (PGM, PNG)
6. **Authentication**: Add user authentication for multi-user access
