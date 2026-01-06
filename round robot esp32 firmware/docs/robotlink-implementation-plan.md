# RobotLink Binary Protocol Implementation Plan

## Overview
Replace the current text-based serial commands with the RobotLink binary framed protocol for robust communication between Arduino and ESP32/host computer.

**Date Created:** 2026-01-05
**Status:** Planning

## Design Decisions

### Text Commands
- Keep for initial testing only
- Remove entirely for production (binary only)
- Cleaner, smaller code, faster parsing

### Odometry Calculations
- Arduino sends raw data (encoders, velocities)
- Python/ESP32 calculates position/heading
- Saves Arduino RAM/CPU
- ESP32 will be between Arduino and Python in final solution

### Update Rate
- Configurable interval via `MSG_ENABLE_STREAM` payload
- Default: 50ms (20 Hz) matching PID rate
- Can be adjusted for different use cases

### Configuration Storage
- Include EEPROM from the start
- Persistent storage for robot parameters
- Auto-load on startup with validation
- Save via command

### Serial Baud Rate
- Upgrade from 9600 to 115200 baud
- Higher throughput for binary protocol
- Better for continuous streaming

## Message Types

### Command Messages (Host → Arduino)

| Type | Code | Payload | Description |
|------|------|---------|-------------|
| `MSG_SET_VEL` | 0x10 | 2x float32 (left, right m/s) | Set motor velocities |
| `MSG_SET_PID` | 0x11 | 3x float32 (Kp, Ki, Kd) | Set PID parameters (both motors) |
| `MSG_SET_DEADBAND` | 0x12 | 4x float32 (L_fwd, L_rev, R_fwd, R_rev) | Set deadband PWM offsets |
| `MSG_SET_CONFIG` | 0x13 | See config struct | Set robot physical parameters |
| `MSG_ENABLE_STREAM` | 0x14 | uint8 enable + uint16 interval_ms | Enable/disable continuous odometry |
| `MSG_GET_CONFIG` | 0x15 | Empty | Request current configuration |
| `MSG_SAVE_CONFIG` | 0x17 | Empty | Save current config to EEPROM |
| `MSG_LOAD_CONFIG` | 0x18 | Empty | Load config from EEPROM |
| `MSG_PING` | 0x7E | Empty or uint32 timestamp | Connection test |

### Response Messages (Arduino → Host)

| Type | Code | Payload | Description |
|------|------|---------|-------------|
| `MSG_ODOM` | 0x01 | 2x int32 encoders + 2x float32 velocities | Odometry data |
| `MSG_STATUS` | 0x03 | Status struct | Current state (PID gains, flags, etc) |
| `MSG_CONFIG_RESP` | 0x16 | Config struct | Configuration response |
| `MSG_PONG` | 0x7F | Echo of PING payload | Connection test response |

## Payload Structures

All structures use little-endian byte order and packed alignment.

### MSG_SET_VEL (0x10)
```c
struct SetVelPayload {
  float velLeft;    // m/s
  float velRight;   // m/s
} __attribute__((packed));
// Size: 8 bytes
```

### MSG_SET_PID (0x11)
```c
struct SetPidPayload {
  float Kp;
  float Ki;
  float Kd;
} __attribute__((packed));
// Size: 12 bytes
```

### MSG_SET_DEADBAND (0x12)
```c
struct SetDeadbandPayload {
  float leftForward;   // PWM offset
  float leftReverse;   // PWM offset
  float rightForward;  // PWM offset
  float rightReverse;  // PWM offset
} __attribute__((packed));
// Size: 16 bytes
```

### MSG_SET_CONFIG (0x13) & MSG_CONFIG_RESP (0x16)
```c
struct ConfigPayload {
  float wheelDiameter;   // meters
  float wheelbase;       // meters (distance between wheels)
  float ticksPerRev;     // encoder ticks per wheel revolution
  uint8_t invertLeft;    // 0 or 1
  uint8_t invertRight;   // 0 or 1
  uint8_t balanceEnable; // 0 or 1
  uint8_t reserved;      // padding
  float balanceGain;     // encoder balance gain
} __attribute__((packed));
// Size: 20 bytes
```

### MSG_ODOM (0x01)
```c
struct OdomPayload {
  int32_t encoderLeft;   // total ticks since startup/zero
  int32_t encoderRight;  // total ticks since startup/zero
  float velLeft;         // m/s
  float velRight;        // m/s
  int16_t pwmLeft;       // current PWM value
  int16_t pwmRight;      // current PWM value
} __attribute__((packed));
// Size: 20 bytes
```

### MSG_STATUS (0x03)
```c
struct StatusPayload {
  float Kp, Ki, Kd;           // PID gains (12 bytes)
  float deadband[4];          // L_fwd, L_rev, R_fwd, R_rev (16 bytes)
  uint8_t pidEnabled;         // 1 byte
  uint8_t streamEnabled;      // 1 byte
  uint16_t streamInterval;    // milliseconds (2 bytes)
  uint32_t framesReceived;    // stats (4 bytes)
  uint32_t framesSent;        // stats (4 bytes)
} __attribute__((packed));
// Size: 40 bytes
```

### MSG_ENABLE_STREAM (0x14)
```c
struct EnableStreamPayload {
  uint8_t enable;          // 0=disable, 1=enable
  uint16_t intervalMs;     // update interval (default 50ms)
} __attribute__((packed));
// Size: 3 bytes
```

## EEPROM Layout

**Total Size:** 64 bytes
**Address:** 0x00 - 0x3F

| Offset | Size | Field | Notes |
|--------|------|-------|-------|
| 0-1    | 2    | Magic number | 0xAA55 for validation |
| 2      | 1    | Version | Current: 1 |
| 3      | 1    | Flags | Reserved for future use |
| 4-7    | 4    | Wheel diameter (float) | meters |
| 8-11   | 4    | Wheelbase (float) | meters |
| 12-15  | 4    | Ticks per rev (float) | encoder resolution |
| 16-19  | 4    | Kp (float) | PID proportional gain |
| 20-23  | 4    | Ki (float) | PID integral gain |
| 24-27  | 4    | Kd (float) | PID derivative gain |
| 28-31  | 4    | Deadband L_fwd (float) | PWM |
| 32-35  | 4    | Deadband L_rev (float) | PWM |
| 36-39  | 4    | Deadband R_fwd (float) | PWM |
| 40-43  | 4    | Deadband R_rev (float) | PWM |
| 44     | 1    | Balance enable (uint8) | 0 or 1 |
| 45-48  | 4    | Balance gain (float) | default 0.02 |
| 49-61  | 13   | Reserved | Future expansion |
| 62-63  | 2    | CRC16 | Of bytes 0-61 |

**Default Values (if EEPROM invalid):**
- Wheel diameter: 0.082m (82mm)
- Wheelbase: 0.15m (150mm, needs measurement)
- Ticks per rev: 360
- Kp: 0.25, Ki: 0.05, Kd: 0.0
- Deadband: L=12/12, R=10/10
- Balance: enabled, gain 0.02

## Implementation Tasks

### Task 1: Define Protocol Extensions (30 min)
- [x] Document message types
- [x] Document payload structures
- [x] Document EEPROM layout
- [ ] Add message type enums to `RobotLink.h`
- [ ] Create payload structs in new header file

### Task 2: EEPROM Configuration System (45 min)
- [ ] Create `config.h` with EEPROM layout and defaults
- [ ] Implement `loadConfig()` function
- [ ] Implement `saveConfig()` function
- [ ] Implement `validateConfig()` with CRC check
- [ ] Add default config initialization

### Task 3: Implement Arduino Firmware (2 hours)
- [ ] Include RobotLink and EEPROM headers
- [ ] Create `RobotLink::Link` instance
- [ ] Change Serial.begin() to 115200 baud
- [ ] Define all payload structs
- [ ] Implement `frameHandler()` callback
  - [ ] Handle MSG_SET_VEL
  - [ ] Handle MSG_SET_PID
  - [ ] Handle MSG_SET_DEADBAND
  - [ ] Handle MSG_SET_CONFIG
  - [ ] Handle MSG_GET_CONFIG
  - [ ] Handle MSG_ENABLE_STREAM
  - [ ] Handle MSG_SAVE_CONFIG
  - [ ] Handle MSG_LOAD_CONFIG
  - [ ] Handle MSG_PING
- [ ] Replace text telemetry with MSG_ODOM binary frames
- [ ] Add streaming control logic
- [ ] Load config from EEPROM in setup()
- [ ] Remove text command parsing (keep for testing, remove later)
- [ ] Test and verify all commands

### Task 4: Python Client Library (1.5 hours)
- [ ] Create `robotlink.py` module
- [ ] Implement CRC16-CCITT-FALSE function
- [ ] Implement frame encoding/decoding
- [ ] Create message classes:
  - [ ] SetVelocity
  - [ ] SetPID
  - [ ] SetDeadband
  - [ ] SetConfig
  - [ ] GetConfig
  - [ ] EnableStream
  - [ ] SaveConfig
  - [ ] LoadConfig
  - [ ] Ping
- [ ] Create `RobotLink` class with:
  - [ ] Serial connection management
  - [ ] Non-blocking receive
  - [ ] Message queue for incoming frames
  - [ ] High-level API methods
  - [ ] Statistics tracking
  - [ ] Context manager support
- [ ] Add timeout and error handling
- [ ] Test against Arduino firmware

### Task 5: Create Test Scripts (1 hour)
- [ ] `test_protocol.py` - Basic connectivity, PING/PONG
- [ ] `test_velocity.py` - Set velocities, verify response
- [ ] `test_config.py` - Set/get/save/load configuration
- [ ] `test_streaming.py` - Enable streaming at different rates
- [ ] `test_full_system.py` - Complete integration test

### Task 6: Update Existing Scripts (30 min)
- [ ] Update `test_motor_balance.py` to use RobotLink
- [ ] Update `tune_pid.py` to use RobotLink
- [ ] Archive old scripts as `*_legacy.py`

## Testing Plan

### Phase 1: Basic Protocol
1. Upload firmware with RobotLink integration
2. Test PING/PONG for connectivity
3. Verify CRC validation (send corrupted frames)
4. Test frame parsing edge cases

### Phase 2: Command Testing
1. Test MSG_SET_VEL with various velocities
2. Test MSG_SET_PID and verify motor response
3. Test MSG_SET_DEADBAND
4. Test MSG_SET_CONFIG with different parameters
5. Test MSG_GET_CONFIG response

### Phase 3: EEPROM Testing
1. Set configuration via MSG_SET_CONFIG
2. Save with MSG_SAVE_CONFIG
3. Power cycle Arduino
4. Verify config loaded on startup
5. Test with corrupted EEPROM (should load defaults)

### Phase 4: Streaming
1. Enable streaming at 50ms interval
2. Verify update rate with timestamps
3. Test different intervals (20ms, 100ms, 200ms)
4. Disable streaming and verify it stops
5. Measure throughput and latency

### Phase 5: Integration
1. Run existing motor balance test via binary protocol
2. Run PID tuning session via binary protocol
3. Verify encoder balancing still works
4. Stress test: rapid velocity changes
5. Long-duration test: 10+ minutes continuous operation

## Success Criteria

- [ ] All messages send/receive correctly
- [ ] CRC validation prevents corrupted data
- [ ] EEPROM save/load preserves all parameters
- [ ] Streaming achieves target update rates (20Hz minimum)
- [ ] Binary protocol uses less bandwidth than text (verify with stats)
- [ ] No crashes or hangs during operation
- [ ] Motor control same or better quality than text protocol
- [ ] Python library easy to use with clean API

## Future Enhancements

- [ ] Add MSG_SET_BALANCE command for per-motor balance control
- [ ] Add MSG_ERROR for Arduino to report errors
- [ ] Add MSG_DEBUG for optional debug info
- [ ] Implement MSG_RESET command
- [ ] Add emergency stop (E-STOP) message with high priority
- [ ] Implement message sequencing/acknowledgment for critical commands
- [ ] Add support for individual motor PID tuning (currently both motors share gains)

## References

- RobotLink protocol: `lib/RobotLink/src/RobotLink.h`
- Current firmware: `src/main.cpp`
- Hardware pinout: `docs/hardware-pinout.md`
- CRC-16/CCITT-FALSE: poly=0x1021 init=0xFFFF xorout=0x0000

## Notes

- Frame overhead: 6 bytes (2 SOF + type + len + 2 CRC)
- Max payload: 64 bytes (configurable, saves RAM on Arduino Uno)
- Total max frame size: 70 bytes
- At 115200 baud: ~11520 bytes/sec = ~165 frames/sec theoretical max
- Practical streaming rate: 20-50 Hz is reasonable target

## Appendix: Example Frame

**MSG_SET_VEL with left=0.3 m/s, right=0.3 m/s:**

```
Byte offset | Value | Description
------------|-------|------------
0           | 0xAA  | SOF byte 0
1           | 0x55  | SOF byte 1
2           | 0x10  | Message type (MSG_SET_VEL)
3           | 0x08  | Payload length (8 bytes)
4-7         | ...   | velLeft as float32 LE (0.3)
8-11        | ...   | velRight as float32 LE (0.3)
12          | 0x??  | CRC16 low byte
13          | 0x??  | CRC16 high byte
```

**Total frame size: 14 bytes**
