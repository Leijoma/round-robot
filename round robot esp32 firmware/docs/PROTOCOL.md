# RobotLink Serial Protocol v1

## Transport
- Physical link: UART/Serial between Arduino <-> ESP32
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

### 0x01 ODOM (Arduino -> ESP32)
Purpose: Provide incremental wheel ticks and (optionally) Arduino-integrated pose for debugging/UI.

**Payload (18 bytes):**
- `uint32 t_ms`    : milliseconds since boot (Arduino millis)
- `int16 dL_ticks` : delta ticks since last message (left wheel)
- `int16 dR_ticks` : delta ticks since last message (right wheel)
- `int32 x_mm`     : pose x in millimeters (odometry)
- `int32 y_mm`     : pose y in millimeters (odometry)
- `int16 th_mrad`  : heading theta in milliradians (odometry)

Units:
- `x_mm, y_mm`: mm
- `th_mrad`: radians * 1000

Notes:
- Host can use `dL_ticks,dR_ticks` to integrate pose itself.
- `x_mm,y_mm,th_mrad` are for convenience and debugging.

### 0x02 CMD_VEL (ESP32 -> Arduino)
Purpose: Command the robot using velocity setpoints.

**Payload (4 bytes):**
- `int16 v_mm_s`    : linear velocity in mm/s
- `int16 w_mrad_s`  : angular velocity in mrad/s

Arduino is responsible for converting (v,w) to motor outputs.

### 0x03 STATUS (Optional)
Not required for MVP; reserved for system state/battery/errors.

### 0x7E PING / 0x7F PONG (Optional)
Keepalive / latency measurement.

## Resynchronization
Receiver scans for SOF sequence 0xAA 0x55.
If CRC fails, receiver discards the frame and resumes SOF scanning.

## Versioning
This is v1. If you need versioning later, add a `MSG_HELLO` with a version byte.
