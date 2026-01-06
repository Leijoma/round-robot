# RobotLink Python Library

Python implementation of the RobotLink binary protocol for communicating with the ESP32 robot controller via UDP.

## Overview

The RobotLink protocol provides reliable binary communication between a host computer and the ESP32-based robot controller. The protocol includes:

- **CRC16 error checking** for data integrity
- **Type-safe message structures** for motor control, odometry, and lidar data
- **UDP transport** for wireless communication
- **Callback-based message handling** for real-time data processing

## Architecture

```
Host (Python)  <--UDP-->  ESP32 (WiFi Bridge)  <--Serial-->  Arduino (Motor Controller)
                          + Lidar (via Serial)
```

The ESP32 acts as a bridge:
- Routes motor control commands from Host to Arduino
- Forwards odometry data from Arduino to Host
- Handles lidar control and streams scan data to Host

## Installation

No external dependencies required - uses only Python standard library.

```bash
# Make the example script executable
chmod +x robot_control_example.py
```

## Quick Start

```python
from robotlink import RobotLink, MessageType, OdomPayload

# Connect to robot (ESP32 in AP mode)
robot = RobotLink(host='192.168.4.1', port=5000)

# Register callback for odometry data
def on_odom(payload):
    odom = OdomPayload.unpack(payload)
    print(f"Velocity: L={odom.vel_left:.3f}, R={odom.vel_right:.3f} m/s")

robot.register_callback(MessageType.MSG_ODOM, on_odom)

# Enable odometry streaming at 100ms intervals
robot.enable_stream(True, 100)

# Set motor velocities (m/s)
robot.set_velocity(0.1, 0.1)

# Process incoming messages
while True:
    robot.process_messages(timeout=0.5)
```

## API Reference

### RobotLink Class

#### Constructor
```python
RobotLink(host='192.168.4.1', port=5000)
```

#### Motor Control
```python
set_velocity(vel_left: float, vel_right: float) -> bool
    """Set motor velocities in m/s"""

stop() -> bool
    """Emergency stop - immediately halt motors"""

set_pid(kp: float, ki: float, kd: float) -> bool
    """Set PID controller parameters"""

set_deadband(left_fwd, left_rev, right_fwd, right_rev) -> bool
    """Set motor deadband compensation"""
```

#### Odometry and Status
```python
enable_stream(enable: bool, interval_ms: int = 100) -> bool
    """Enable/disable continuous odometry streaming"""

get_config() -> bool
    """Request robot configuration"""

set_config(wheel_diameter, wheelbase, ticks_per_rev, ...) -> bool
    """Set robot configuration"""
```

#### Lidar Control
```python
lidar_enable(enable: bool) -> bool
    """Enable/disable lidar motor"""

lidar_set_rpm(target_rpm: int) -> bool
    """Set lidar target RPM (typically 200-300)"""
```

#### Message Processing
```python
register_callback(msg_type: MessageType, callback: Callable)
    """Register a callback for a specific message type"""

process_messages(timeout: float = 0.1) -> int
    """Process incoming messages and call registered callbacks
    Returns: Number of messages processed"""

get_stats() -> Dict[str, int]
    """Get protocol statistics (frames sent/received, CRC errors)"""
```

### Message Types

#### Motor Control (Host → Robot)
- `MSG_SET_VEL` (0x10) - Set motor velocities
- `MSG_SET_PID` (0x11) - Set PID parameters
- `MSG_SET_DEADBAND` (0x12) - Set deadband compensation
- `MSG_SET_CONFIG` (0x13) - Set robot configuration
- `MSG_ENABLE_STREAM` (0x14) - Enable odometry streaming
- `MSG_STOP` (0x15) - Emergency stop

#### Odometry & Status (Robot → Host)
- `MSG_ODOM` (0x01) - Odometry data (encoders, velocities, PWM)
- `MSG_STATUS` (0x03) - Robot status (PID, deadband, uptime)
- `MSG_CONFIG_RESP` (0x16) - Configuration response
- `MSG_ACK` (0x02) - Command acknowledgment
- `MSG_ERROR` (0x04) - Error message

#### Lidar (ESP32 ↔ Host)
- `MSG_LIDAR_STATUS` (0x21) - Lidar status (RPM, packets, etc.)
- `MSG_LIDAR_SCAN` (0x20) - Scan data (distance + strength)
- `MSG_LIDAR_ENABLE` (0x22) - Enable/disable motor
- `MSG_LIDAR_SET_RPM` (0x23) - Set target RPM

### Payload Structures

#### OdomPayload
```python
@dataclass
class OdomPayload:
    encoder_left: int      # Total encoder ticks
    encoder_right: int     # Total encoder ticks
    vel_left: float        # m/s
    vel_right: float       # m/s
    pwm_left: int          # PWM value (-255 to 255)
    pwm_right: int         # PWM value (-255 to 255)
    timestamp: int         # milliseconds since startup
```

#### LidarStatusPayload
```python
@dataclass
class LidarStatusPayload:
    current_rpm: int
    target_rpm: int
    motor_running: bool
    rpm_control_enabled: bool  # PD-controller on/off
    motor_speed: int           # Current PWM (0-255)
    packets_received: int
    packets_invalid: int
    scans_complete: int
    timestamp: int
```

#### LidarScanPayload
```python
@dataclass
class LidarScanPayload:
    timestamp: int
    rpm: int
    start_angle: int       # 0-359 degrees
    readings: list         # List of LidarReading

@dataclass
class LidarReading:
    distance_mm: int
    signal_strength: int
    invalid: bool
    warning: bool
```

## Example: Autonomous Navigation

```python
#!/usr/bin/env python3
from robotlink import RobotLink, MessageType, OdomPayload, LidarScanPayload
import time

robot = RobotLink(host='192.168.4.1', port=5000)

# Track obstacle data
closest_obstacle = float('inf')

def on_lidar_scan(payload):
    global closest_obstacle
    scan = LidarScanPayload.unpack(payload)

    # Find closest valid reading
    valid_distances = [r.distance_mm for r in scan.readings if not r.invalid]
    if valid_distances:
        closest_obstacle = min(valid_distances)

def on_odom(payload):
    odom = OdomPayload.unpack(payload)
    print(f"Position: L={odom.encoder_left}, R={odom.encoder_right} ticks")

robot.register_callback(MessageType.MSG_LIDAR_SCAN, on_lidar_scan)
robot.register_callback(MessageType.MSG_ODOM, on_odom)

# Start lidar and odometry
robot.lidar_enable(True)
robot.lidar_set_rpm(220)
robot.enable_stream(True, 100)

# Simple obstacle avoidance
try:
    while True:
        robot.process_messages(timeout=0.1)

        # Drive forward if clear, rotate if obstacle detected
        if closest_obstacle > 500:  # 500mm clearance
            robot.set_velocity(0.15, 0.15)
            print(f"Forward - clearance: {closest_obstacle}mm")
        else:
            robot.set_velocity(-0.05, 0.05)  # Rotate right
            print(f"Rotating - obstacle at {closest_obstacle}mm")

        time.sleep(0.1)

except KeyboardInterrupt:
    robot.stop()
    robot.lidar_enable(False)
    robot.close()
```

## Protocol Frame Format

```
[SOF0] [SOF1] [TYPE] [LEN] [PAYLOAD...] [CRC_LO] [CRC_HI]

SOF0, SOF1: 0xAA, 0x55 (start-of-frame markers)
TYPE:       Message type code (1 byte)
LEN:        Payload length in bytes (0-255)
PAYLOAD:    Message-specific data (little-endian)
CRC:        CRC16 checksum (little-endian)
```

## Error Handling

The library tracks these statistics:
- `frames_sent` - Total frames transmitted
- `frames_received` - Total valid frames received
- `crc_errors` - Frames discarded due to checksum errors

Check connection health:
```python
stats = robot.get_stats()
error_rate = stats['crc_errors'] / max(stats['frames_received'] + stats['crc_errors'], 1)
if error_rate > 0.05:
    print("WARNING: High error rate - check WiFi connection")
```

## Network Configuration

### ESP32 Access Point Mode (Default)
- **SSID**: `RobotAP` (configurable in ESP32 firmware)
- **Password**: See `config.h` in ESP32 firmware
- **IP**: `192.168.4.1`
- **Port**: `5000`

### Connecting
```python
# AP mode
robot = RobotLink(host='192.168.4.1', port=5000)

# Station mode (if ESP32 connected to your WiFi)
robot = RobotLink(host='192.168.1.100', port=5000)
```

## Troubleshooting

### No response from robot
1. Check WiFi connection to ESP32
2. Verify ESP32 is powered and firmware running
3. Check IP address and port
4. Increase socket timeout: `robot.sock.settimeout(1.0)`

### High CRC error rate
- WiFi interference - move closer to ESP32
- Check ESP32 antenna connection
- Reduce data rate (increase stream interval)

### Lidar not working
1. Check motor is enabled: `robot.lidar_enable(True)`
2. Verify motor power supply (5V, 150mA)
3. Check serial connections (ESP32 GPIO16/17)
4. Monitor lidar status messages

## Files

- `robotlink.py` - Main library implementation
- `robot_control_example.py` - Complete usage example
- `README.md` - This documentation

## License

Same as main project.
