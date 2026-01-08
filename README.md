# Round Robot Project

Complete wireless differential drive robot system with LIDAR mapping capabilities.

## Overview

This repository contains three integrated components that work together to create a complete robot control system:

1. **[Arduino Motor Control](Arduino%20Motor%20control/)** - Motor controller firmware with PID velocity control
2. **[ESP32 WiFi Bridge](round%20robot%20esp32%20firmware/)** - Wireless communication bridge with LIDAR integration
3. **[Robot UI](robot-ui/)** - Web-based control dashboard with real-time visualization

## System Architecture

```
┌─────────────────┐
│   Web Browser   │  User Interface
│   (Dashboard)   │  - Motor control
└────────┬────────┘  - LIDAR visualization
         │ WebSocket  - Occupancy mapping
         │ (Socket.IO)
┌────────▼────────┐
│  Flask Server   │  Python Backend
│   (Robot UI)    │  - WebSocket relay
└────────┬────────┘  - Map generation
         │ UDP
         │ RobotLink Protocol
┌────────▼────────┐
│      ESP32      │  WiFi Bridge
│  WiFi Bridge    │  - UDP ↔ Serial
└────┬───────┬────┘  - LIDAR batching
     │       │
     │Serial │Serial - Message forwarding
     │9600   │115200
     │       │
┌────▼──┐  ┌─▼──────────┐
│Arduino│  │Neato LIDAR │  Hardware
│Motor  │  │  (XV-11)   │  - Sensors
│Control│  │  220 RPM   │  - Actuators
└───────┘  └────────────┘
```

## Features

### Complete Robot Control System

- **Wireless Operation**: WiFi/UDP communication with web-based control
- **Real-time Telemetry**: 5-20 Hz odometry streaming
- **LIDAR Mapping**: 360° scanning with occupancy grid generation
- **PID Velocity Control**: Smooth motor control with deadband compensation
- **Web Dashboard**: Responsive 3-panel interface for complete robot control

### Protocol Stack

- **RobotLink Protocol**: Binary framing with CRC16 for reliable communication
- **Transport Layers**:
  - WebSocket (JSON) for Browser ↔ Flask
  - UDP (binary) for Flask ↔ ESP32
  - Serial (binary) for ESP32 ↔ Arduino/LIDAR

### Hardware Support

- Arduino Uno (ATmega328P) with Monster Moto Shield
- ESP32 (WiFi/dual UART)
- Neato XV-11 LIDAR
- Quadrature encoders (360 ticks/rev)
- Differential drive (2-wheel robot)

## Quick Start

### 1. Arduino Motor Controller Setup

```bash
cd "Arduino Motor control"
pio run --target upload
pio device monitor --baud 115200  # Debug output
```

See [Arduino Motor control/README.md](Arduino%20Motor%20control/README.md) for details.

**Key Configuration**:
- Firmware: v1.1 (SoftwareSerial)
- RobotLink: A0/A1 @ 9600 baud
- Debug: USB @ 115200 baud
- Control loop: 20 Hz

### 2. ESP32 WiFi Bridge Setup

Edit `round robot esp32 firmware/src/main.cpp`:

```cpp
const char* WIFI_SSID = "YourNetwork";
const char* WIFI_PASSWORD = "YourPassword";  // CHANGE THIS!
```

Upload firmware:

```bash
cd "round robot esp32 firmware"
pio run --target upload
pio device monitor --baud 115200  # Monitor connection
```

See [round robot esp32 firmware/README.md](round%20robot%20esp32%20firmware/README.md) for details.

**Key Configuration**:
- WiFi: Configure in main.cpp
- Arduino Serial: GPIO26/27 @ 9600 baud
- LIDAR Serial: GPIO16 @ 115200 baud
- UDP Port: 5000

### 3. Web UI Setup

Install dependencies:

```bash
cd robot-ui
pip3 install -r server/requirements.txt
```

Edit `robot-ui/server/robot_ui_server.py`:

```python
ESP32_HOST = '192.168.68.52'  # Your ESP32 IP address
ESP32_PORT = 5000
```

Start server:

```bash
python3 server/robot_ui_server.py
```

Access dashboard at: **http://localhost:5001**

See [robot-ui/README.md](robot-ui/README.md) for details.

**Controls**:
- Keyboard: W/A/S/D or arrow keys
- Mouse: Directional buttons + velocity slider
- Space: Emergency stop

## Wiring Connections

### ESP32 ↔ Arduino (SoftwareSerial)

| ESP32 | Arduino Uno | Signal |
|-------|-------------|--------|
| GPIO26 (RX) | A1 (TX) | Serial data to ESP32 |
| GPIO27 (TX) | A0 (RX) | Serial data to Arduino |
| GND | GND | Common ground (REQUIRED) |

**Baud rate**: 9600

### ESP32 ↔ Neato LIDAR

| ESP32 | Neato LIDAR | Signal |
|-------|-------------|--------|
| GPIO16 (RX) | Orange wire (TX) | LIDAR data |
| GPIO21 (PWM) | Motor control | Speed control |
| GND | GND | Common ground |
| - | Red wire | 5V power supply |

**Baud rate**: 115200

### Arduino ↔ Motors & Encoders

See [Arduino Motor control/docs/hardware-pinout.md](Arduino%20Motor%20control/docs/hardware-pinout.md) for complete pinout.

**Quick Reference**:
- Left Motor: INA=7, INB=8, PWM=5
- Right Motor: INA=4, INB=9, PWM=6
- Left Encoder: A=2 (INT0), B=10
- Right Encoder: A=3 (INT1), B=11

## Communication Protocol

All components use the **RobotLink binary protocol** for reliable, efficient communication.

### Key Message Types

| Type | Value | Direction | Description |
|------|-------|-----------|-------------|
| MSG_ODOM | 0x01 | Arduino → Host | Odometry (encoders, pose, velocity) |
| MSG_CMD_VEL | 0x02 | Host → Arduino | Velocity command (v, w) |
| MSG_SET_VEL | 0x10 | Host → Arduino | Direct wheel velocities |
| MSG_SET_PID | 0x11 | Host → Arduino | PID gain tuning |
| MSG_ENABLE_STREAM | 0x14 | Host → Arduino | Enable/disable odometry |
| MSG_STOP | 0x1A | Host → Arduino | Emergency stop |
| MSG_LIDAR_ENABLE | 0x30 | Host → ESP32 | Start/stop LIDAR motor |
| MSG_LIDAR_SET_RPM | 0x31 | Host → ESP32 | Set LIDAR rotation speed |
| MSG_LIDAR_SCAN | 0x32 | ESP32 → Host | LIDAR scan data (batched) |
| MSG_PING/PONG | 0x7E/0x7F | Both | Connection testing |

See protocol documentation:
- [Arduino Motor control/docs/PROTOCOL.md](Arduino%20Motor%20control/docs/PROTOCOL.md)
- [round robot esp32 firmware/docs/PROTOCOL.md](round%20robot%20esp32%20firmware/docs/PROTOCOL.md)

## Robot Configuration

### Hardware Parameters (Example Configuration)

**Arduino Motor Control Firmware**:
- Wheel diameter: 82mm
- Wheelbase: 240mm
- Encoder resolution: 360 ticks/rev
- PID defaults: Kp=10.0, Ki=5.0, Kd=0.1
- Deadband: 30 PWM units (forward/reverse)

**Note**: The robot-ui server contains its own configuration for odometry calculation. Adjust `WHEEL_DIAMETER` and `TICKS_PER_REV` in `robot-ui/server/robot_ui_server.py` to match your robot's actual hardware.

## Performance Specifications

| Metric | Value |
|--------|-------|
| **Control Loop** | 20 Hz (Arduino) |
| **Odometry Rate** | 5-20 Hz (configurable) |
| **LIDAR Scan Rate** | 5-6 Hz (360° scans) |
| **LIDAR RPM** | 200-250 RPM (PD-controlled) |
| **WiFi Latency** | 10-30ms typical |
| **Network Bandwidth** | ~30 KB/s (odom + LIDAR) |
| **Max Velocity** | ~0.3 m/s (recommended) |
| **Min Velocity** | ~0.05 m/s (with tuning) |

## Development

### Project Structure

```
Round_robot/
├── Arduino Motor control/          # Motor controller firmware
│   ├── src/main.cpp               # PID control, RobotLink handler
│   ├── lib/RobotLink/             # Binary protocol library
│   ├── docs/                      # Protocol & hardware docs
│   └── README.md
│
├── round robot esp32 firmware/    # WiFi bridge firmware
│   ├── src/main.cpp               # UDP↔Serial bridge, LIDAR
│   ├── lib/
│   │   ├── RobotLink/             # Binary protocol library
│   │   └── NeatoLidar/            # LIDAR driver library
│   ├── python/robotlink.py        # Python client library
│   ├── docs/                      # Protocol & hardware docs
│   └── README.md
│
└── robot-ui/                       # Web dashboard
    ├── server/
    │   ├── robot_ui_server.py     # Flask + SocketIO server
    │   └── robotlink.py           # RobotLink UDP client
    ├── static/
    │   ├── index.html             # Dashboard UI
    │   ├── css/style.css          # Dark theme
    │   └── js/                    # Frontend logic
    ├── docs/                      # Architecture & API docs
    └── README.md
```

### Building and Testing

Each subproject has its own build system:

**Arduino & ESP32** (PlatformIO):
```bash
cd "Arduino Motor control"
pio run                 # Build
pio run --target upload # Upload
pio device monitor      # Monitor serial
```

**Robot UI** (Python):
```bash
cd robot-ui
pip3 install -r server/requirements.txt
python3 server/robot_ui_server.py
```

### Python Testing Scripts

Each firmware includes Python test scripts:

**Arduino**:
- `test_robot.py` - Basic motor control
- `test_protocol.py` - Protocol verification
- `check_odom.py` - Odometry monitoring

**ESP32**:
- `python/test_robotlink.py` - Full system test
- `python/test_motors.py` - Motor control via WiFi
- `python/test_lidar.py` - LIDAR functionality

## Troubleshooting

### System Not Working

1. **Check Arduino firmware** - USB serial should show debug output
2. **Check ESP32 WiFi** - Should connect and display IP address
3. **Check wiring** - Verify all connections, especially common ground
4. **Check baud rates** - Arduino: 9600 (A0/A1), LIDAR: 115200, Debug: 115200

### Motors Not Moving

- Verify power supply to motor shield
- Check that Arduino firmware uploaded successfully
- Monitor Arduino debug output (USB @ 115200)
- Verify commands are being received (watch debug output)
- Check deadband settings if motors vibrate but don't move

### No Odometry Data in UI

- Check ESP32 boot messages (odometry streaming should auto-enable)
- Verify ESP32 WiFi connection (should show IP address)
- Check robot-ui server connection to ESP32 (connection status in UI)
- Monitor Arduino debug for outgoing ODOM frames

### LIDAR Not Scanning

- Verify LIDAR power supply (5V, >500mA)
- Check GPIO16 (RX) connection to LIDAR TX (orange wire)
- Send LIDAR enable command from UI
- Monitor ESP32 debug for LIDAR packet reception
- Verify LIDAR motor spinning (should be audible)

### WiFi Connection Issues

- Verify SSID and password in ESP32 firmware
- Check WiFi signal strength
- Ensure ESP32 gets IP address on same subnet as host
- Try static IP if DHCP fails

## Documentation

### Component Documentation

- **[Arduino Motor Control](Arduino%20Motor%20control/README.md)** - Firmware, PID tuning, protocol
- **[ESP32 WiFi Bridge](round%20robot%20esp32%20firmware/README.md)** - Setup, configuration, LIDAR
- **[Robot UI](robot-ui/README.md)** - Installation, usage, API

### Protocol Documentation

- **[RobotLink Protocol](Arduino%20Motor%20control/docs/PROTOCOL.md)** - Binary framing, message types
- **[Protocol Quick Reference](Arduino%20Motor%20control/docs/PROTOCOL_QUICK_REF.md)** - Fast lookup
- **[WebSocket API](robot-ui/docs/API.md)** - Browser ↔ Server events

### Hardware Documentation

- **[Arduino Pinout](Arduino%20Motor%20control/docs/hardware-pinout.md)** - Complete pin assignments
- **[ESP32 Pinout](round%20robot%20esp32%20firmware/docs/hardware-pinout.md)** - ESP32 connections
- **[SoftwareSerial Wiring](Arduino%20Motor%20control/docs/SOFTWARESERIAL_WIRING.md)** - Arduino ↔ ESP32

### Status & Integration

- **[ESP32 Integration](round%20robot%20esp32%20firmware/INTEGRATION_COMPLETE.md)** - LIDAR integration report
- **[System Status](round%20robot%20esp32%20firmware/SYSTEM_STATUS.md)** - Current status & issues
- **[SoftwareSerial Update](Arduino%20Motor%20control/SOFTWARESERIAL_UPDATE.md)** - Firmware v1.1 changes

## Security Notice

⚠️ **WiFi Credentials**: The ESP32 firmware contains hardcoded WiFi credentials in `src/main.cpp`. Before deploying:

1. Change the default password
2. Never commit actual passwords to version control
3. Consider using WiFi provisioning or configuration files for production

See [round robot esp32 firmware/README.md](round%20robot%20esp32%20firmware/README.md#wifi-configuration) for security best practices.

## License

See LICENSE file for details.

## Contributing

This is a personal robot project, but contributions and suggestions are welcome! Please ensure:

- Code follows existing style and conventions
- Documentation is updated for any changes
- Test scripts are provided for new features
- Hardware changes are documented with pinout updates

## Author

Magnus
January 2026

## Version History

- **v1.1** (2026-01-07) - SoftwareSerial update, unified documentation
- **v1.0** (2026-01-06) - Initial integration complete
  - Arduino Motor Control firmware
  - ESP32 WiFi Bridge with LIDAR
  - Robot UI web dashboard
