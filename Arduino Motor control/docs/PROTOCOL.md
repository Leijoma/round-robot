# RobotLink Serial Protocol v1

## Transport
- Physical link: UART/Serial between Arduino <-> ESP32/Host
- Recommended baud: 115200 or higher (9600 works for odom-only but is limiting)
- Byte order: little-endian for multi-byte integer fields

## Frame Format
Each message is a framed packet:

| Field | Size | Value |
|------|------|-------|
| SOF0 | 1 | 0xAA |
| SOF1 | 1 | 0x55 |
| TYPE | 1 | Message type |
| LEN  | 1 | Payload length in bytes (0..255) |
| PAYLOAD | LEN | Binary payload |
| CRC16_LO | 1 | CRC16 (low byte) |
| CRC16_HI | 1 | CRC16 (high byte) |

### CRC
CRC16/CCITT-FALSE over the bytes: `TYPE, LEN, PAYLOAD`.
- Polynomial: 0x1021
- Init: 0xFFFF
- XOROUT: 0x0000
- refin/refout: false/false

SOF bytes are NOT included in CRC.

## Message Types

### Core Protocol Messages

#### 0x01 ODOM (Arduino -> Host)
Purpose: Provide incremental wheel ticks and Arduino-integrated pose for debugging/UI.

**Payload (18 bytes):**
- `uint32 t_ms`    : milliseconds since boot (Arduino millis)
- `int16 dL_ticks` : delta ticks since last message (left wheel)
- `int16 dR_ticks` : delta ticks since last message (right wheel)
- `int32 x_mm`     : pose x in millimeters (odometry)
- `int32 y_mm`     : pose y in millimeters (odometry)
- `int16 th_mrad`  : heading theta in milliradians (odometry)

**Units:**
- `x_mm, y_mm`: mm
- `th_mrad`: radians * 1000

**Notes:**
- Host can use `dL_ticks,dR_ticks` to integrate pose itself.
- `x_mm,y_mm,th_mrad` are for convenience and debugging.
- Sent automatically when streaming is enabled (see MSG_ENABLE_STREAM)

**Implementation:** [main.cpp:473-502](../src/main.cpp#L473-L502)

---

#### 0x02 CMD_VEL (Host -> Arduino)
Purpose: Command the robot using velocity setpoints (differential drive kinematics).

**Payload (4 bytes):**
- `int16 v_mm_s`    : linear velocity in mm/s
- `int16 w_mrad_s`  : angular velocity in mrad/s

**Behavior:**
Arduino converts (v, w) to individual motor velocities using differential drive kinematics:
```
wheelbase_half = wheelbase / 2
vel_left  = v - w * wheelbase_half
vel_right = v + w * wheelbase_half
```

**Example:**
- Forward at 150 mm/s: `v=150, w=0`
- Turn in place: `v=0, w=500` (rotate at 500 mrad/s)
- Arc motion: `v=150, w=200`

**Implementation:** [main.cpp:363-379](../src/main.cpp#L363-L379)

---

#### 0x7E PING (Bidirectional)
Purpose: Connection test / keepalive / latency measurement.

**Payload (variable, typically 4 bytes):**
- `uint32 timestamp` : arbitrary timestamp value (optional)

**Behavior:**
- Sender sends PING with optional timestamp
- Receiver echoes back as PONG (0x7F) with same payload

**Implementation:** [main.cpp:465-469](../src/main.cpp#L465-L469)

---

#### 0x7F PONG (Bidirectional)
Purpose: Response to PING message.

**Payload (variable):**
- Same as received PING payload

**Implementation:** [main.cpp:465-469](../src/main.cpp#L465-L469)

---

### Extended Protocol Messages

#### 0x10 MSG_SET_VEL (Host -> Arduino)
Purpose: Direct motor velocity control (bypasses differential drive kinematics).

**Payload (8 bytes):**
- `float velLeft`  : left motor velocity in m/s
- `float velRight` : right motor velocity in m/s

**Use case:**
When you want precise control of individual motors without differential drive conversion.

**Implementation:** [main.cpp:381-389](../src/main.cpp#L381-L389)

---

#### 0x11 MSG_SET_PID (Host -> Arduino)
Purpose: Update PID controller parameters at runtime.

**Payload (12 bytes):**
- `float Kp` : Proportional gain
- `float Ki` : Integral gain
- `float Kd` : Derivative gain

**Behavior:**
- Updates both left and right motor PID controllers
- Takes effect immediately
- Not persisted unless MSG_SAVE_CONFIG is sent

**Example values:**
- Default: Kp=10.0, Ki=5.0, Kd=0.1
- Aggressive: Kp=15.0, Ki=8.0, Kd=0.2
- Gentle: Kp=8.0, Ki=3.0, Kd=0.05

**Implementation:** [main.cpp:391-399](../src/main.cpp#L391-L399)

---

#### 0x12 MSG_SET_DEADBAND (Host -> Arduino)
Purpose: Update motor deadband compensation values.

**Payload (16 bytes):**
- `float leftForward`  : PWM offset for left motor forward
- `float leftReverse`  : PWM offset for left motor reverse
- `float rightForward` : PWM offset for right motor forward
- `float rightReverse` : PWM offset for right motor reverse

**Behavior:**
- Compensates for motor friction at low speeds
- Values are in PWM units (typically 20-50 out of 255)
- Applied on top of PID output
- Not persisted unless MSG_SAVE_CONFIG is sent

**Typical values:**
- Small motors: 20-30
- Medium motors: 30-40
- Large/geared motors: 40-50

**Implementation:** [main.cpp:401-410](../src/main.cpp#L401-L410)

---

#### 0x14 MSG_ENABLE_STREAM (Host -> Arduino)
Purpose: Enable/disable automatic odometry streaming.

**Payload (3 bytes):**
- `uint8 enable`     : 0=disable, 1=enable
- `uint16 intervalMs` : update interval in milliseconds

**Behavior:**
- When enabled, Arduino automatically sends MSG_ODOM at specified interval
- Default interval: 50ms (20 Hz)
- Minimum recommended: 20ms (50 Hz)
- Maximum: 65535ms (~65 seconds)

**Example:**
- Enable at 50ms: `enable=1, intervalMs=50`
- Disable: `enable=0, intervalMs=0`

**Implementation:** [main.cpp:412-419](../src/main.cpp#L412-L419)

---

#### 0x15 MSG_GET_CONFIG (Host -> Arduino)
Purpose: Request current robot configuration.

**Payload:** Empty (0 bytes)

**Response:** MSG_CONFIG_RESP (0x16) with ConfigPayload

**Implementation:** [main.cpp:450-463](../src/main.cpp#L450-L463)

---

#### 0x16 MSG_CONFIG_RESP (Arduino -> Host)
Purpose: Response to MSG_GET_CONFIG with robot parameters.

**Payload (20 bytes):**
- `float wheelDiameter` : wheel diameter in meters (e.g., 0.082)
- `float wheelbase`     : distance between wheels in meters (e.g., 0.24)
- `float ticksPerRev`   : encoder ticks per revolution (e.g., 360)
- `uint8 invertLeft`    : 0=normal, 1=inverted
- `uint8 invertRight`   : 0=normal, 1=inverted
- `uint8 balanceEnable` : 0=disabled, 1=enabled (reserved)
- `uint8 reserved`      : padding (set to 0)
- `float balanceGain`   : encoder balance gain (reserved)

**Implementation:** [main.cpp:450-463](../src/main.cpp#L450-L463)

---

#### 0x17 MSG_SAVE_CONFIG (Host -> Arduino)
Purpose: Save current PID and deadband values to EEPROM.

**Payload:** Empty (0 bytes)

**Behavior:**
- Saves current Kp, Ki, Kd values for both motors
- Saves deadband values (forward/reverse for each motor)
- Saves robot configuration (wheel diameter, wheelbase, encoder resolution)
- Uses magic number and checksum for validation
- Values persist across power cycles

**EEPROM Structure:**
- Magic: 0xAB12
- Version: 1
- PID parameters (12 bytes)
- Deadband values (16 bytes)
- Robot config (12 bytes)
- Checksum (1 byte)

**Implementation:** [main.cpp:313-330](../src/main.cpp#L313-L330)

---

#### 0x18 MSG_LOAD_CONFIG (Host -> Arduino)
Purpose: Reload configuration from EEPROM.

**Payload:** Empty (0 bytes)

**Behavior:**
- Reads configuration from EEPROM
- Validates magic number and checksum
- Applies values if valid
- Silently fails if EEPROM data is invalid
- Automatically called on Arduino boot

**Implementation:** [main.cpp:332-356](../src/main.cpp#L332-L356)

---

#### 0x19 MSG_ZERO_ENCODERS (Host -> Arduino)
Purpose: Reset encoder counts and odometry pose to zero.

**Payload:** Empty (0 bytes)

**Behavior:**
- Sets encoder counts to 0
- Resets pose: x=0, y=0, theta=0
- Does not affect velocity tracking
- Useful for resetting odometry after repositioning robot

**Implementation:** [main.cpp:421-433](../src/main.cpp#L421-L433)

---

#### 0x1A MSG_STOP (Host -> Arduino)
Purpose: Emergency stop - immediately halt all motors.

**Payload:** Empty (0 bytes)

**Behavior:**
- Sets target velocities to 0
- Outputs 0 PWM to both motors
- Resets PID integral terms
- Does not clear encoder counts

**Implementation:** [main.cpp:435-438](../src/main.cpp#L435-L438)

---

### Reserved/Optional Messages

#### 0x03 STATUS (Either direction)
**Status:** Not implemented
**Purpose:** Reserved for system state, battery level, error codes, etc.
**Note:** Marked as "Not required for MVP" - implement if needed for your application.

#### 0x13 MSG_SET_CONFIG (Host -> Arduino)
**Status:** Not implemented
**Purpose:** Would allow runtime modification of wheel diameter, wheelbase, encoder resolution.
**Note:** Currently these are compile-time constants from hardware-pinout.md. Could be implemented if runtime reconfiguration is needed.

---

## Resynchronization
Receiver scans for SOF sequence 0xAA 0x55.
If CRC fails, receiver discards the frame and resumes SOF scanning.

## Error Handling
- Invalid message length: Ignored
- CRC mismatch: Frame discarded, resync initiated
- Unknown message type: Silently ignored
- Oversized payload: Rejected if configured (see RobotLink::Config)

## Versioning
This is v1. If versioning is needed later, add a `MSG_HELLO` with a version byte.

## Implementation Notes

### Arduino Motor Control Firmware
- **Baud rate:** 115200
- **Control loop:** 20 Hz (50ms)
- **Odometry streaming:** Configurable (default 20 Hz)
- **PWM range:** 0-255
- **Velocity units:** m/s
- **Platform:** Arduino Uno (ATmega328P)
- **RAM usage:** ~327 bytes
- **Flash usage:** ~9.6 KB

### Message Ordering
- No guaranteed ordering (single-threaded, so effectively ordered)
- No acknowledgments (fire-and-forget)
- For critical commands, verify result via odometry or status messages

### Typical Communication Flow

**Initialization:**
```
Host -> Arduino: PING
Arduino -> Host: PONG
Host -> Arduino: GET_CONFIG
Arduino -> Host: CONFIG_RESP
Host -> Arduino: ENABLE_STREAM (enable=1, interval=50)
```

**Normal Operation:**
```
Arduino -> Host: ODOM (every 50ms)
Host -> Arduino: CMD_VEL (as needed)
```

**Tuning:**
```
Host -> Arduino: SET_PID (Kp=15, Ki=8, Kd=0.2)
Host -> Arduino: SET_DEADBAND (35, 35, 35, 35)
Host -> Arduino: SET_VEL (0.1, 0.1)  # test
# Observe ODOM messages...
Host -> Arduino: SAVE_CONFIG  # if satisfied
```

**Emergency:**
```
Host -> Arduino: STOP
```

## References

- [RobotLink.h](../lib/RobotLink/src/RobotLink.h) - Protocol implementation
- [RobotLinkMessages.h](../lib/RobotLink/src/RobotLinkMessages.h) - Message payload structures
- [main.cpp](../src/main.cpp) - Arduino firmware implementation
- [hardware-pinout.md](hardware-pinout.md) - Hardware configuration
- [test_robot.py](../test_robot.py) - Python reference implementation
