# ESP32 WiFi Bridge Firmware

ESP32-based WiFi/UDP bridge for wireless robot control with integrated Neato XV-11 LIDAR support.

## Overview

This firmware transforms an ESP32 into a wireless bridge that enables:
- **WiFi/UDP communication** between host computer and robot
- **Arduino Motor Controller** integration (Serial1 @ 9600 baud)
- **Neato XV-11 LIDAR** integration (Serial2 @ 115200 baud)
- **RobotLink protocol** message forwarding and LIDAR data batching

## Features

- ✅ Bidirectional WiFi/UDP ↔ Serial bridge
- ✅ RobotLink binary protocol support
- ✅ Neato XV-11 LIDAR integration with RPM control
- ✅ LIDAR scan batching (40 readings per UDP packet)
- ✅ Auto-enable odometry streaming on boot
- ✅ Message forwarding between Host ↔ Arduino
- ✅ Real-time debug output via USB Serial

## Quick Start

### 1. Configure WiFi

Edit `src/main.cpp` lines 10-12:

```cpp
const char* WIFI_SSID = "YourNetworkName";
const char* WIFI_PASSWORD = "YourPassword";  // CHANGE THIS!
const uint16_t UDP_PORT = 5000;
```

**⚠️ SECURITY**: Never commit actual passwords to version control. See [ESP32_WiFi_Bridge_README.md](ESP32_WiFi_Bridge_README.md#wifi-configuration) for best practices.

### 2. Build and Upload

```bash
cd "round robot esp32 firmware"
pio run --target upload
pio device monitor --baud 115200
```

### 3. Verify Connection

Expected boot output:
```
========================================
ESP32 Robot Bridge - INCREMENTAL BUILD
========================================

WiFi Setup
  IP Address: 192.168.68.52
  UDP Port: 5000

Arduino Serial Setup
  Serial1: GPIO26(RX) GPIO27(TX) @ 9600 baud
  Odometry streaming enabled @ 200ms (5 Hz)

System Ready!
Waiting for Host to connect...
```

## Hardware Connections

### Pin Assignments

| Component | ESP32 Pin | Baud Rate | Notes |
|-----------|-----------|-----------|-------|
| **Arduino Motor Controller** | | | |
| RX (from Arduino TX) | GPIO26 | 9600 | SoftwareSerial A1 on Arduino |
| TX (to Arduino RX) | GPIO27 | 9600 | SoftwareSerial A0 on Arduino |
| **Neato XV-11 LIDAR** | | | |
| RX (LIDAR data) | GPIO16 | 115200 | Orange wire from LIDAR |
| PWM (Motor control) | GPIO21 | 25kHz PWM | Motor speed control |
| **Debug** | | | |
| USB Serial | Built-in | 115200 | Debug output only |

**Common ground required** between all devices!

### Wiring Diagram

```
┌──────────────┐         ┌──────────────┐         ┌──────────────┐
│   Host PC    │         │    ESP32     │         │   Arduino    │
│              │  WiFi   │  WiFi Bridge │ Serial1 │    Motor     │
│  (Python)    │◄───────►│              │◄───────►│  Controller  │
│              │  UDP    │  GPIO26/27   │ 9600    │  A0/A1       │
└──────────────┘  5000   └───────┬──────┘         └──────────────┘
                                 │
                                 │ Serial2
                                 │ 115200
                          ┌──────▼──────┐
                          │ Neato LIDAR │
                          │   (XV-11)   │
                          │  GPIO16/21  │
                          └─────────────┘
```

See [ESP32_WiFi_Bridge_README.md](ESP32_WiFi_Bridge_README.md#wiring-diagram) for detailed wiring.

## System Architecture

The ESP32 acts as a transparent bridge forwarding messages bidirectionally:

1. **Host → ESP32 → Arduino**: Motor commands, configuration
2. **Arduino → ESP32 → Host**: Odometry data
3. **LIDAR → ESP32 → Host**: Scan data (batched)
4. **Host → ESP32 → LIDAR**: Motor control (enable/disable, RPM)

All communication uses the RobotLink binary protocol for reliability and efficiency.

## Configuration

### Odometry Streaming

The firmware auto-enables odometry streaming on boot with these defaults:

```cpp
enableCmd.enable = 1;           // Enable streaming
enableCmd.intervalMs = 200;     // 200ms = 5 Hz
```

To change the rate, edit `src/main.cpp` line 104. Valid range: 50-500ms.

**Note**: Higher rates (50ms = 20Hz) work well but increase network traffic.

### LIDAR Settings

Default LIDAR configuration:

```cpp
#define LIDAR_TARGET_RPM 220    // Target rotation speed
#define LIDAR_BATCH_SIZE 40     // Readings per UDP packet (10 Neato packets)
```

LIDAR motor is **stopped by default** on boot. Use `MSG_LIDAR_ENABLE` to start.

## Usage

### Python Client Example

```python
from robotlink import RobotLink

# Connect to ESP32
robot = RobotLink(host='192.168.68.52', port=5000)

# Motor control
robot.set_velocity(left=0.2, right=0.2)  # Move forward
robot.cmd_vel(v=0.15, w=0.5)             # Curve
robot.stop()                             # Emergency stop

# LIDAR control
robot.lidar_enable(True)                 # Start LIDAR motor
robot.lidar_set_rpm(240)                 # Set target RPM

# Enable odometry streaming
robot.enable_stream(True, interval_ms=50)  # 20 Hz

# Register callbacks
robot.on_odom(lambda data: print(f"Odom: {data}"))
robot.on_lidar_scan(lambda data: print(f"LIDAR: {len(data.readings)} readings"))

# Process messages
while True:
    robot.process_messages(timeout=0.1)
```

See [python/README.md](python/README.md) for the complete RobotLink Python library documentation.

## Documentation

- **[ESP32_WiFi_Bridge_README.md](ESP32_WiFi_Bridge_README.md)** - Detailed firmware documentation
- **[INTEGRATION_COMPLETE.md](INTEGRATION_COMPLETE.md)** - LIDAR integration report
- **[SYSTEM_STATUS.md](SYSTEM_STATUS.md)** - Current system status and known issues
- **[docs/PROTOCOL.md](docs/PROTOCOL.md)** - RobotLink protocol specification
- **[docs/hardware-pinout.md](docs/hardware-pinout.md)** - Complete pin assignments
- **[lib/NeatoLidar/README.md](lib/NeatoLidar/README.md)** - LIDAR library documentation
- **[python/README.md](python/README.md)** - Python library documentation

## Troubleshooting

### WiFi Connection Fails

- Verify SSID and password in `src/main.cpp`
- Check that ESP32 is within WiFi range
- Monitor debug output for connection status
- Try resetting ESP32

### Arduino Communication Issues

- Verify Serial1 connections: GPIO26 (RX) ↔ A1 (TX), GPIO27 (TX) ↔ A0 (RX)
- Check common ground between ESP32 and Arduino
- Ensure Arduino firmware uses 9600 baud on SoftwareSerial
- Monitor debug output for RX frame errors

### LIDAR Not Working

- Verify Serial2 connection: GPIO16 (RX) ↔ LIDAR TX (orange wire)
- Check PWM connection: GPIO21 ↔ LIDAR motor control
- Ensure LIDAR power supply is adequate (5V, >500mA)
- Send `MSG_LIDAR_ENABLE` to start motor
- Monitor debug output for packet reception

### ESP32 Keeps Rebooting

- **Most common cause**: RX buffer allocation failure
- Solution: Verify `setRxBufferSize(512)` in code (NOT 2048)
- See [ESP32_WiFi_Bridge_README.md](ESP32_WiFi_Bridge_README.md#critical-bug-fix-esp32-boot-crash) for details

### No Odometry Data

- Ensure Arduino firmware is running and configured correctly
- Check if odometry streaming is enabled (auto-enabled on ESP32 boot)
- Verify Serial1 baud rate matches Arduino (9600)
- Monitor debug output for incoming ODOM frames

## Performance

- **WiFi Latency**: ~10-30ms typical
- **Odometry Rate**: Configurable 5-20 Hz (default 5 Hz)
- **LIDAR Rate**: 5-6 Hz (full 360° scans)
- **LIDAR RPM Control**: PD-controller maintains ±5 RPM
- **Network Bandwidth**: ~30 KB/s (odometry + LIDAR)
- **Memory Usage**: 512-byte Serial RX buffer

## Known Issues

See [SYSTEM_STATUS.md](SYSTEM_STATUS.md) for current operational status and known limitations.

## License

See LICENSE file for details.

## References

- [Arduino Motor Control Firmware](../Arduino%20Motor%20control/README.md)
- [Robot UI Server](../robot-ui/README.md)
- [RobotLink Protocol Specification](docs/PROTOCOL.md)
- [Neato XV-11 LIDAR Documentation](lib/NeatoLidar/README.md)
