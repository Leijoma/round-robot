# Robot Control UI

Real-time web-based dashboard for controlling a differential drive robot with LIDAR mapping capabilities.

![Dashboard Preview](docs/dashboard-preview.png)
*3-panel layout: Motor Control | LIDAR Visualization | Occupancy Map*

## Features

- **Motor Control**: Directional buttons, velocity slider, keyboard controls (WASD/Arrows)
- **LIDAR Visualization**: Real-time 360° polar plot with distance rings
- **Occupancy Mapping**: Live occupancy grid with ray tracing and robot pose overlay
- **Odometry Display**: Position, velocity, encoders, and PWM values
- **WebSocket Communication**: Real-time bidirectional data streaming
- **ESP32 Integration**: Connects to ESP32 WiFi/UDP bridge (RobotLink protocol)

## Quick Start

### Prerequisites

- Python 3.8+
- ESP32 robot controller running WiFi/UDP bridge firmware
- ESP32 IP address (default: 192.168.68.52:5000)

### Installation

```bash
cd /Users/magnus/Documents/PlatformIO/Projects/robot-ui

# Install Python dependencies
pip3 install -r server/requirements.txt

# Start the server
python3 server/robot_ui_server.py
```

### Access Dashboard

Open your web browser to:
```
http://localhost:5000
```

Or from another device on the same network:
```
http://<your-computer-ip>:5000
```

## System Architecture

```
┌──────────────┐          WebSocket          ┌──────────────┐
│   Browser    │◄──────────────────────────►│ Flask Server │
│     (UI)     │      (Socket.IO/JSON)       │   (Python)   │
└──────────────┘                             └──────┬───────┘
                                                    │
                                                    │ RobotLink
                                                    │ UDP/Binary
                                             ┌──────▼───────┐
                                             │    ESP32     │
                                             │ WiFi Bridge  │
                                             └──────┬───────┘
                                                    │
                         ┌──────────────────────────┼──────────────┐
                         │ Serial1 @ 9600           │              │
                         │                          │ Serial2 @ 115200
                  ┌──────▼───────┐          ┌───────▼────────┐
                  │   Arduino    │          │  Neato LIDAR   │
                  │    Motor     │          │    (XV-11)     │
                  │  Controller  │          │  220 RPM       │
                  └──────────────┘          └────────────────┘
```

## Configuration

### ESP32 Connection

Edit `server/robot_ui_server.py`:

```python
ESP32_HOST = '192.168.68.52'  # Your ESP32 IP address
ESP32_PORT = 5000              # UDP port
```

### Server Port

The Flask server runs on port 5000 by default. Change it in `robot_ui_server.py`:

```python
socketio.run(app, host='0.0.0.0', port=5000, debug=True)
```

## Project Structure

```
robot-ui/
├── docs/
│   ├── ARCHITECTURE.md          # System architecture documentation
│   ├── IMPLEMENTATION_PLAN.md   # Detailed implementation plan
│   └── API.md                   # WebSocket API documentation
├── server/
│   ├── robot_ui_server.py       # Flask + SocketIO server
│   ├── robotlink.py             # RobotLink protocol library
│   ├── occupancy_grid.py        # Occupancy grid mapping backend
│   └── requirements.txt         # Python dependencies
├── static/
│   ├── index.html               # Main UI
│   ├── css/
│   │   └── style.css            # Dark theme styling
│   └── js/
│       ├── app.js               # Main logic + WebSocket
│       ├── lidar_viz.js         # LIDAR polar plot visualization
│       ├── occupancy_map.js     # Occupancy grid rendering
│       └── controls.js          # Motor control handlers
└── README.md                    # This file
```

## Usage

### Motor Control

**Keyboard Controls:**
- `W` or `↑`: Move forward
- `S` or `↓`: Move backward
- `A` or `←`: Rotate left
- `D` or `→`: Rotate right
- `SPACE`: Emergency stop

**UI Controls:**
- Directional buttons (▲▼◄►)
- Velocity slider (0.1 - 0.5 m/s)
- STOP button (emergency stop)

### LIDAR Control

- **Enable Motor**: Toggle LIDAR motor on/off
- **Target RPM**: Adjust LIDAR rotation speed (200-300 RPM)
- **Status**: Monitor current RPM, packets received, and errors

### Occupancy Map

- **Live Update**: Map updates in real-time as LIDAR scans
- **Robot Pose**: Triangle shows robot position and heading
- **Clear Map**: Reset grid to unknown state
- **Save Map**: Export map data (future feature)

**Color Coding:**
- ⬜ White = Free space
- ⬜ Gray = Unknown
- ⬛ Black = Occupied

## WebSocket API

The UI communicates with the server via Socket.IO WebSocket events:

### Server → Client

- `connection_status`: Connection state and ESP32 availability
- `odom_update`: Odometry data @ 20 Hz
- `lidar_scan`: LIDAR scan batches (40 readings/packet)
- `lidar_status`: LIDAR motor status @ 1 Hz
- `map_update`: Occupancy grid updates @ 2 Hz
- `error`: Error messages

### Client → Server

- `motor_command`: Motor control (velocity, cmd_vel, stop)
- `lidar_command`: LIDAR control (enable, set_rpm)
- `clear_map`: Reset occupancy grid

See [docs/API.md](docs/API.md) for detailed API documentation.

## Dependencies

### Python (Server)

```
Flask==3.0.0
Flask-SocketIO==5.3.5
python-socketio==5.10.0
numpy==1.26.2
```

Install with:
```bash
pip3 install -r server/requirements.txt
```

### JavaScript (Client)

- Socket.IO Client (v4.5.4) - loaded from CDN
- HTML5 Canvas API - built-in

## Development

### Running in Debug Mode

```bash
python3 server/robot_ui_server.py
```

Flask's debug mode enables:
- Auto-reload on file changes
- Detailed error messages
- WebSocket debugging

### Testing Without Robot

The server will start even if the ESP32 is not reachable. The UI will show "ESP32: Offline" status. Connect the ESP32 and the system will automatically establish communication.

### Browser Compatibility

Tested on:
- Chrome 120+
- Firefox 121+
- Safari 17+
- Edge 120+

Requires WebSocket and HTML5 Canvas support.

## Troubleshooting

### Server Won't Start

**Issue**: `Address already in use`

**Solution**: Another process is using port 5000
```bash
# Find process using port 5000
lsof -i :5000

# Kill the process
kill -9 <PID>
```

### ESP32 Not Connected

**Issue**: UI shows "ESP32: Offline"

**Checks**:
1. Verify ESP32 is powered and running WiFi bridge firmware
2. Check ESP32 IP address: `ping 192.168.68.52`
3. Ensure computer and ESP32 are on same WiFi network
4. Check firewall allows UDP port 5000
5. Verify ESP32 firmware is using correct UDP port

### No Odometry Data

**Issue**: LIDAR works but no odometry updates

**Checks**:
1. Arduino is connected to ESP32 Serial1 (GPIO26/27)
2. Baud rate matches (9600)
3. Ground connection between ESP32 and Arduino
4. Arduino firmware is streaming odometry

### LIDAR Not Spinning

**Issue**: LIDAR motor doesn't start

**Checks**:
1. LIDAR motor enable checkbox is checked
2. GPIO21 PWM connection to LIDAR motor controller
3. LIDAR power supply (5V)
4. Check ESP32 serial monitor for LIDAR status messages

### Map Not Updating

**Issue**: LIDAR scans received but map doesn't change

**Checks**:
1. Robot pose is being updated from odometry
2. Check browser console for JavaScript errors
3. LIDAR readings are valid (not all flagged as invalid)
4. Canvas rendering is working (check browser compatibility)

## Performance

### Expected Rates

- **Odometry**: 20 Hz (50ms interval)
- **LIDAR Scans**: 5-20 Hz (variable, depends on RPM)
- **Map Updates**: 2 Hz (throttled to reduce bandwidth)
- **UI Rendering**: 60 FPS (requestAnimationFrame)

### Bandwidth Usage

Per client:
- Odometry: ~1.2 KB/s
- LIDAR: ~5 KB/s
- Map: ~24 KB/s (uncompressed)
- **Total**: ~30 KB/s = 240 Kbps

## Future Enhancements

- [ ] SLAM algorithm integration (particle filter / EKF)
- [ ] Path planning (A*, RRT)
- [ ] Waypoint navigation
- [ ] Map export/import (PGM, PNG formats)
- [ ] Session recording/replay
- [ ] Multi-robot support
- [ ] 3D visualization
- [ ] Mobile-responsive design
- [ ] Touch controls for tablets
- [ ] User authentication

## Documentation

- [ARCHITECTURE.md](docs/ARCHITECTURE.md) - System architecture and components
- [IMPLEMENTATION_PLAN.md](docs/IMPLEMENTATION_PLAN.md) - Detailed implementation steps
- [API.md](docs/API.md) - WebSocket API specification

## License

This project is part of the Round Robot platform.

## Author

**Magnus** - 2026-01-06

## Related Projects

- **ESP32 WiFi/UDP Bridge**: `/Users/magnus/Documents/PlatformIO/Projects/round robot motor firmware`
- **Arduino Motor Controller**: Differential drive with encoders and PID control
- **Neato LIDAR Integration**: XV-11 LIDAR with RPM control

## Contributing

This is a personal robotics project. Feel free to use and modify for your own robots!

## Support

For issues or questions:
- Check [Troubleshooting](#troubleshooting) section
- Review documentation in `docs/` folder
- Check ESP32 serial monitor for debug output
- Verify WebSocket connection in browser console

## Acknowledgments

- Flask and Flask-SocketIO for web framework
- Socket.IO for real-time communication
- RobotLink protocol for reliable binary messaging
- Neato Robotics for the XV-11 LIDAR hardware
