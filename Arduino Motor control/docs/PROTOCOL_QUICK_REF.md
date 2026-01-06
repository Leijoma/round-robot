# RobotLink Protocol Quick Reference

## Frame Format
```
[0xAA][0x55][TYPE][LEN][PAYLOAD...][CRC_LO][CRC_HI]
```
- CRC16/CCITT-FALSE over TYPE+LEN+PAYLOAD
- Little-endian byte order

---

## Message Types Summary

| Code | Name | Dir | Payload | Purpose |
|------|------|-----|---------|---------|
| **Core Messages** |
| 0x01 | ODOM | A→H | 18 bytes | Encoder deltas + pose |
| 0x02 | CMD_VEL | H→A | 4 bytes | Differential drive (v,w) |
| 0x7E | PING | ↔ | Variable | Connection test |
| 0x7F | PONG | ↔ | Variable | Ping response |
| **Extended Messages** |
| 0x10 | SET_VEL | H→A | 8 bytes | Direct motor velocities |
| 0x11 | SET_PID | H→A | 12 bytes | PID parameters |
| 0x12 | SET_DEADBAND | H→A | 16 bytes | Deadband compensation |
| 0x14 | ENABLE_STREAM | H→A | 3 bytes | Enable/disable streaming |
| 0x15 | GET_CONFIG | H→A | 0 bytes | Request configuration |
| 0x16 | CONFIG_RESP | A→H | 20 bytes | Configuration response |
| 0x17 | SAVE_CONFIG | H→A | 0 bytes | Save to EEPROM |
| 0x18 | LOAD_CONFIG | H→A | 0 bytes | Load from EEPROM |
| 0x19 | ZERO_ENCODERS | H→A | 0 bytes | Reset encoders |
| 0x1A | STOP | H→A | 0 bytes | Emergency stop |

**Legend:** A=Arduino, H=Host

---

## Common Commands

### Basic Movement
```python
# Forward at 0.15 m/s
robot.cmd_vel(v=150, w=0)  # 150 mm/s, 0 mrad/s

# Turn in place
robot.cmd_vel(v=0, w=500)  # 500 mrad/s rotation

# Direct motor control
robot.set_velocity(left=0.1, right=0.1)  # both 0.1 m/s
```

### Configuration
```python
# Get robot parameters
config = robot.get_config()

# Update PID
robot.set_pid(Kp=15.0, Ki=8.0, Kd=0.2)

# Update deadband
robot.set_deadband(35, 35, 35, 35)

# Save to EEPROM
robot.save_config()
```

### Monitoring
```python
# Enable odometry at 50ms
robot.enable_stream(True, 50)

# Read one odometry message
odom = robot.read_odometry()
print(f"Left: {odom['dL_ticks']} ticks")
```

---

## Payload Structures

### ODOM (0x01) - 18 bytes
```
uint32 t_ms         [0:4]   Timestamp (ms)
int16  dL_ticks     [4:6]   Left encoder delta
int16  dR_ticks     [6:8]   Right encoder delta
int32  x_mm         [8:12]  X position (mm)
int32  y_mm         [12:16] Y position (mm)
int16  th_mrad      [16:18] Heading (mrad)
```

### CMD_VEL (0x02) - 4 bytes
```
int16  v_mm_s       [0:2]   Linear velocity (mm/s)
int16  w_mrad_s     [2:4]   Angular velocity (mrad/s)
```

### SET_VEL (0x10) - 8 bytes
```
float  velLeft      [0:4]   Left motor (m/s)
float  velRight     [4:8]   Right motor (m/s)
```

### SET_PID (0x11) - 12 bytes
```
float  Kp           [0:4]   Proportional gain
float  Ki           [4:8]   Integral gain
float  Kd           [8:12]  Derivative gain
```

### SET_DEADBAND (0x12) - 16 bytes
```
float  leftForward  [0:4]   Left forward PWM offset
float  leftReverse  [4:8]   Left reverse PWM offset
float  rightForward [8:12]  Right forward PWM offset
float  rightReverse [12:16] Right reverse PWM offset
```

### ENABLE_STREAM (0x14) - 3 bytes
```
uint8  enable       [0]     0=disable, 1=enable
uint16 intervalMs   [1:3]   Update interval (ms)
```

### CONFIG_RESP (0x16) - 20 bytes
```
float  wheelDiameter [0:4]   Meters (e.g., 0.082)
float  wheelbase     [4:8]   Meters (e.g., 0.24)
float  ticksPerRev   [8:12]  Encoder resolution (e.g., 360)
uint8  invertLeft    [12]    0 or 1
uint8  invertRight   [13]    0 or 1
uint8  balanceEnable [14]    0 or 1
uint8  reserved      [15]    Padding
float  balanceGain   [16:20] Reserved
```

---

## Default Values

| Parameter | Default | Unit | Notes |
|-----------|---------|------|-------|
| Baud rate | 115200 | bps | Serial communication |
| Control loop | 20 | Hz | PID update rate |
| Odom interval | 50 | ms | Default streaming |
| Kp | 10.0 | - | Proportional gain |
| Ki | 5.0 | - | Integral gain |
| Kd | 0.1 | - | Derivative gain |
| Deadband | 30 | PWM | Forward/reverse offset |
| Wheel diameter | 0.082 | m | 82mm |
| Wheelbase | 0.24 | m | 240mm |
| Encoder resolution | 360 | ticks/rev | - |

---

## Typical Usage Flow

```
1. Connect:       PING → PONG
2. Configure:     GET_CONFIG → CONFIG_RESP
3. Start stream:  ENABLE_STREAM(1, 50)
4. Control:       CMD_VEL or SET_VEL
5. Monitor:       Receive ODOM messages
6. Tune (if needed): SET_PID, SET_DEADBAND
7. Save:          SAVE_CONFIG
8. Stop:          STOP
```

---

## Error Codes

None - protocol is fire-and-forget. Verify commands by:
- Reading ODOM messages for motion verification
- Using GET_CONFIG to verify settings
- Observing encoder counts for motor activity

---

## Units Summary

| Field | Unit | Range | Notes |
|-------|------|-------|-------|
| Velocity | m/s | ±1.0 | Typical: 0.05-0.3 |
| Angular vel | mrad/s | ±3000 | Milliradians/sec |
| Position | mm | ±2³¹ | Millimeters |
| Angle | mrad | ±π×1000 | Wraps at ±π |
| PWM | 0-255 | 0-255 | Arduino analogWrite |
| Time | ms | 0-2³² | Milliseconds |
| Encoder | ticks | ±2³¹ | Cumulative |

---

## File References

- Full protocol: [PROTOCOL.md](PROTOCOL.md)
- Implementation: [../src/main.cpp](../src/main.cpp)
- Python client: [../test_robot.py](../test_robot.py)
- Hardware: [hardware-pinout.md](hardware-pinout.md)
