# Robot Control UI - WebSocket API Documentation

## Overview

The Robot Control UI uses Socket.IO (WebSocket) for real-time bidirectional communication between the web browser and Flask server. The server acts as a bridge between the WebSocket (JSON messages) and the ESP32 (RobotLink binary protocol over UDP).

## Connection

### Client Connection

```javascript
const socket = io('http://localhost:5000');

socket.on('connect', () => {
    console.log('Connected to server');
});

socket.on('disconnect', () => {
    console.log('Disconnected from server');
});
```

### Server Events

#### `connect`
Emitted when client successfully connects to server.

**Response**: Server sends `connection_status` event

#### `disconnect`
Emitted when client disconnects from server.

---

## Events: Server → Client (ESP32 → UI)

### `connection_status`

**Description**: Initial connection status and ESP32 availability

**Payload**:
```json
{
    "status": "connected",
    "esp32": true
}
```

**Fields**:
- `status` (string): Connection status ("connected" or "disconnected")
- `esp32` (boolean): Whether ESP32 is reachable

**Example**:
```javascript
socket.on('connection_status', (data) => {
    console.log(`Status: ${data.status}, ESP32: ${data.esp32}`);
    updateStatusIndicator(data.esp32);
});
```

---

### `odom_update`

**Description**: Odometry data from robot (20 Hz rate)

**Payload**:
```json
{
    "encoder_left": 12345,
    "encoder_right": 12389,
    "vel_left": 0.28,
    "vel_right": 0.32,
    "pwm_left": 156,
    "pwm_right": 162,
    "timestamp": 45678
}
```

**Fields**:
- `encoder_left` (int32): Left wheel encoder ticks (total since startup)
- `encoder_right` (int32): Right wheel encoder ticks (total since startup)
- `vel_left` (float): Left wheel velocity in m/s
- `vel_right` (float): Right wheel velocity in m/s
- `pwm_left` (int16): Left motor PWM value (-255 to +255)
- `pwm_right` (int16): Right motor PWM value (-255 to +255)
- `timestamp` (uint32): Milliseconds since robot startup

**Rate**: ~20 Hz (configurable via `enable_stream` command)

**Example**:
```javascript
socket.on('odom_update', (data) => {
    document.getElementById('vel-left').textContent = `${data.vel_left.toFixed(2)} m/s`;
    document.getElementById('vel-right').textContent = `${data.vel_right.toFixed(2)} m/s`;
    document.getElementById('enc-left').textContent = data.encoder_left;
    document.getElementById('enc-right').textContent = data.encoder_right;
});
```

---

### `lidar_scan`

**Description**: LIDAR scan readings batch (40 readings per message)

**Payload**:
```json
{
    "timestamp": 45678,
    "rpm": 220,
    "start_angle": 0,
    "readings": [
        {
            "distance_mm": 1234,
            "signal_strength": 180,
            "invalid": false,
            "warning": false
        },
        ...
    ]
}
```

**Fields**:
- `timestamp` (uint32): Milliseconds since ESP32 startup
- `rpm` (uint16): LIDAR motor RPM at time of scan
- `start_angle` (uint16): Starting angle in degrees (0-359)
- `readings` (array): Array of LIDAR readings

**Reading Object**:
- `distance_mm` (uint16): Distance in millimeters (0-4000mm typical)
- `signal_strength` (uint16): Signal strength (0-65535)
- `invalid` (boolean): True if reading is invalid/unreliable
- `warning` (boolean): True if signal strength is weak

**Rate**: Variable (~5-20 Hz depending on LIDAR RPM and batching)

**Example**:
```javascript
socket.on('lidar_scan', (data) => {
    // Update polar plot visualization
    lidarViz.addScanBatch(data.start_angle, data.readings);

    // Update stats
    const validCount = data.readings.filter(r => !r.invalid).length;
    document.getElementById('lidar-readings').textContent =
        `${validCount}/${data.readings.length}`;
});
```

---

### `lidar_status`

**Description**: LIDAR motor status and statistics

**Payload**:
```json
{
    "current_rpm": 224,
    "target_rpm": 220,
    "motor_running": true,
    "packets_received": 1234,
    "packets_invalid": 5,
    "scans_complete": 42,
    "timestamp": 45678
}
```

**Fields**:
- `current_rpm` (uint16): Current LIDAR motor RPM
- `target_rpm` (uint16): Target RPM setpoint
- `motor_running` (boolean): True if motor is enabled
- `packets_received` (uint32): Total valid LIDAR packets received
- `packets_invalid` (uint32): Total invalid packets (CRC errors)
- `scans_complete` (uint32): Total complete 360° scans
- `timestamp` (uint32): Milliseconds since ESP32 startup

**Rate**: ~1 Hz

**Example**:
```javascript
socket.on('lidar_status', (data) => {
    document.getElementById('lidar-rpm').textContent =
        `${data.current_rpm} (target: ${data.target_rpm})`;
    document.getElementById('lidar-packets').textContent = data.packets_received;
    document.getElementById('lidar-errors').textContent = data.packets_invalid;

    const motorIndicator = document.getElementById('lidar-motor-status');
    motorIndicator.className = data.motor_running ? 'indicator active' : 'indicator';
});
```

---

### `map_update`

**Description**: Occupancy grid map data

**Payload**:
```json
{
    "width": 100,
    "height": 100,
    "resolution": 0.05,
    "robot_pose": {
        "x": 1.234,
        "y": 0.567,
        "theta": 0.785
    },
    "grid": [[0, 0, 1, 2, ...], ...]
}
```

**Fields**:
- `width` (int): Grid width in cells
- `height` (int): Grid height in cells
- `resolution` (float): Cell size in meters (e.g., 0.05 = 5cm)
- `robot_pose` (object): Robot position and heading
  - `x` (float): X position in meters
  - `y` (float): Y position in meters
  - `theta` (float): Heading in radians
- `grid` (2D array): Occupancy grid data
  - 0 = Free space
  - 1 = Unknown
  - 2 = Occupied

**Rate**: ~2 Hz (throttled to reduce bandwidth)

**Example**:
```javascript
socket.on('map_update', (data) => {
    occupancyMap.updateGrid(data.grid);
    occupancyMap.updateRobotPose(data.robot_pose);

    // Update stats
    const occupied = data.grid.flat().filter(v => v === 2).length;
    const free = data.grid.flat().filter(v => v === 0).length;
    document.getElementById('map-occupied').textContent = occupied;
    document.getElementById('map-free').textContent = free;
});
```

---

### `error`

**Description**: Error message from server

**Payload**:
```json
{
    "message": "Robot not connected"
}
```

**Fields**:
- `message` (string): Human-readable error message

**Example**:
```javascript
socket.on('error', (data) => {
    console.error('Server error:', data.message);
    alert(`Error: ${data.message}`);
});
```

---

## Events: Client → Server (UI → ESP32)

### `motor_command`

**Description**: Send motor control command to robot

**Payload (Type: velocity)**:
```json
{
    "type": "velocity",
    "vel_left": 0.3,
    "vel_right": 0.3
}
```

**Payload (Type: cmd_vel)**:
```json
{
    "type": "cmd_vel",
    "v": 0.3,
    "w": 0.0
}
```

**Payload (Type: stop)**:
```json
{
    "type": "stop"
}
```

**Fields**:
- `type` (string): Command type ("velocity", "cmd_vel", or "stop")
- `vel_left` (float): Left wheel velocity in m/s (for "velocity" type)
- `vel_right` (float): Right wheel velocity in m/s (for "velocity" type)
- `v` (float): Linear velocity in m/s (for "cmd_vel" type)
- `w` (float): Angular velocity in rad/s (for "cmd_vel" type)

**Example**:
```javascript
// Direct wheel velocities
socket.emit('motor_command', {
    type: 'velocity',
    vel_left: 0.3,
    vel_right: 0.3
});

// Linear + angular (differential drive kinematics)
socket.emit('motor_command', {
    type: 'cmd_vel',
    v: 0.3,    // forward at 0.3 m/s
    w: 0.5     // turn at 0.5 rad/s
});

// Emergency stop
socket.emit('motor_command', {
    type: 'stop'
});
```

---

### `lidar_command`

**Description**: Control LIDAR motor and settings

**Payload (Type: enable)**:
```json
{
    "type": "enable",
    "enable": true
}
```

**Payload (Type: set_rpm)**:
```json
{
    "type": "set_rpm",
    "rpm": 220
}
```

**Fields**:
- `type` (string): Command type ("enable" or "set_rpm")
- `enable` (boolean): Enable (true) or disable (false) LIDAR motor
- `rpm` (uint16): Target RPM (200-300 typical for Neato XV-11)

**Example**:
```javascript
// Enable LIDAR motor
socket.emit('lidar_command', {
    type: 'enable',
    enable: true
});

// Set target RPM
socket.emit('lidar_command', {
    type: 'set_rpm',
    rpm: 250
});
```

---

### `clear_map`

**Description**: Clear occupancy grid map (reset to unknown)

**Payload**: None (empty object)

**Example**:
```javascript
socket.emit('clear_map');
```

**Response**: Server broadcasts `map_update` with cleared grid to all clients

---

## Message Flow Examples

### Example 1: Robot Startup and Telemetry

```
1. Client connects to server
   Client → Server: [WebSocket connect]
   Server → Client: connection_status {status: "connected", esp32: true}

2. Server establishes ESP32 connection
   Server → ESP32: ENABLE_STREAM (RobotLink MSG_ENABLE_STREAM)
   Server → ESP32: LIDAR_ENABLE (RobotLink MSG_LIDAR_ENABLE)
   Server → ESP32: LIDAR_SET_RPM (RobotLink MSG_LIDAR_SET_RPM)

3. Continuous telemetry stream begins
   ESP32 → Server: ODOM messages @ 20 Hz
   Server → Client: odom_update events @ 20 Hz

   ESP32 → Server: LIDAR_SCAN messages @ variable rate
   Server → Client: lidar_scan events @ variable rate
   Server → Client: map_update events @ 2 Hz (after grid processing)

   ESP32 → Server: LIDAR_STATUS messages @ 1 Hz
   Server → Client: lidar_status events @ 1 Hz
```

### Example 2: Motor Control

```
1. User presses "Forward" button
   Client → Server: motor_command {type: "velocity", vel_left: 0.3, vel_right: 0.3}

2. Server forwards to ESP32
   Server → ESP32: SET_VEL (RobotLink MSG_SET_VEL with payload)

3. Robot executes and reports motion
   ESP32 → Server: ODOM messages (now with non-zero velocities and encoder deltas)
   Server → Client: odom_update events (UI shows robot moving)

4. User presses "Stop" button
   Client → Server: motor_command {type: "stop"}
   Server → ESP32: STOP (RobotLink MSG_STOP)
   ESP32 → Server: ODOM messages (velocities return to zero)
```

### Example 3: LIDAR Control

```
1. User adjusts RPM slider to 250
   Client → Server: lidar_command {type: "set_rpm", rpm: 250}

2. Server forwards to ESP32
   Server → ESP32: LIDAR_SET_RPM (RobotLink MSG_LIDAR_SET_RPM)

3. ESP32 adjusts motor speed
   ESP32 → Server: LIDAR_STATUS messages (current_rpm increases: 220→230→240→248→250)
   Server → Client: lidar_status events (UI shows RPM increasing)
```

### Example 4: Map Management

```
1. User clicks "Clear Map" button
   Client → Server: clear_map {}

2. Server clears occupancy grid
   Server internal: occupancyGrid.clear()

3. Server broadcasts updated map
   Server → All Clients: map_update {grid: [[1,1,1,...], ...]} (all cells = unknown)

4. Map rebuilds from new LIDAR scans
   ESP32 → Server: LIDAR_SCAN messages (continuous)
   Server internal: occupancyGrid.update(scan)
   Server → All Clients: map_update events (grid gradually fills with occupied/free cells)
```

---

## Error Handling

### Server Errors

If the server encounters an error (e.g., ESP32 not connected), it sends an `error` event:

```javascript
socket.on('error', (data) => {
    console.error('Error:', data.message);
    showErrorNotification(data.message);
});
```

### Connection Loss

Handle disconnection gracefully:

```javascript
socket.on('disconnect', () => {
    console.warn('Disconnected from server');
    updateStatusIndicator(false);
    disableControls();
});

socket.on('reconnect', () => {
    console.log('Reconnected to server');
    updateStatusIndicator(true);
    enableControls();
});
```

### Timeout Handling

For commands that expect a response, implement timeout logic:

```javascript
function sendMotorCommand(vel_left, vel_right, timeout = 1000) {
    return new Promise((resolve, reject) => {
        const timer = setTimeout(() => {
            reject(new Error('Command timeout'));
        }, timeout);

        socket.emit('motor_command', {
            type: 'velocity',
            vel_left,
            vel_right
        });

        // Assume success if no error received within timeout
        socket.once('error', (data) => {
            clearTimeout(timer);
            reject(new Error(data.message));
        });

        // For simplicity, resolve after a short delay (or wait for confirmation)
        setTimeout(() => {
            clearTimeout(timer);
            resolve();
        }, 100);
    });
}
```

---

## Performance Considerations

### Message Rates

- **Odometry**: 20 Hz (50ms interval) - high frequency for real-time control
- **LIDAR Scans**: 5-20 Hz (variable) - depends on LIDAR RPM and batching
- **LIDAR Status**: 1 Hz - low frequency status updates
- **Map Updates**: 2 Hz (throttled) - balance between responsiveness and bandwidth

### Bandwidth Estimation

Typical bandwidth usage per client:

- Odometry: 20 Hz × ~60 bytes JSON = 1.2 KB/s
- LIDAR: 10 Hz × ~500 bytes JSON = 5 KB/s (40 readings × ~12 bytes each)
- Map: 2 Hz × ~12 KB JSON = 24 KB/s (100×100 grid, uncompressed)
- **Total**: ~30 KB/s = 240 Kbps per client

### Optimization Tips

1. **Throttle map updates**: Already implemented at 2 Hz
2. **Compress grid data**: Use RLE or delta encoding for sparse grids
3. **Binary WebSocket**: Switch from JSON to binary for LIDAR data
4. **Client-side prediction**: Interpolate odometry between updates

---

## Security Notes

**WARNING**: This API is designed for local network development only.

- No authentication or authorization
- No encryption (use WSS for production)
- No input validation on client commands (server validates)
- Intended for trusted local network only

For production deployment, add:
- User authentication (JWT tokens)
- TLS/SSL encryption (WSS://)
- Rate limiting on commands
- Input validation and sanitization
