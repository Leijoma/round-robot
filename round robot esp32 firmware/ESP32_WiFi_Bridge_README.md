# ESP32 WiFi/UDP Bridge for Robot Control

## Overview

This ESP32 firmware creates a WiFi/UDP bridge that enables wireless robot control by forwarding messages bidirectionally between:
- **Host Computer** (Python via UDP) ↔ **ESP32** (WiFi) ↔ **Arduino** (Serial1 @ 9600 baud)

The bridge uses the RobotLink protocol for framed binary communication, providing reliable command and telemetry exchange over WiFi.

## System Architecture

```
┌──────────────────┐         WiFi/UDP          ┌──────────────────┐
│                  │    (192.168.68.52:5000)    │                  │
│  Host Computer   │◄──────────────────────────►│      ESP32       │
│   (Python)       │      RobotLink Protocol    │   WiFi Bridge    │
│                  │                            │                  │
└──────────────────┘                            └────────┬─────────┘
                                                         │ Serial1
                                                         │ 9600 baud
                                                         │ RobotLink
                                                         │
                                                ┌────────▼─────────┐
                                                │                  │
                                                │     Arduino      │
                                                │  Motor Controller│
                                                │                  │
                                                └──────────────────┘
```

## Hardware Configuration

### ESP32 Pin Assignments

| Function | ESP32 Pin | Connected To | Details |
|----------|-----------|--------------|---------|
| **Arduino Serial Communication** | | | |
| Serial1 RX | GPIO26 | Arduino TX (SoftwareSerial) | 9600 baud |
| Serial1 TX | GPIO27 | Arduino RX (SoftwareSerial) | 9600 baud |
| GND | GND | Arduino GND | Common ground required |
| **LIDAR (Neato XV-11)** | | | |
| Serial2 RX | GPIO16 | Neato LIDAR TX (orange wire) | 115200 baud data input |
| PWM Motor Control | GPIO21 | LIDAR Motor Control | 25kHz PWM @ 0-255 duty cycle |
| **Debug/Programming** | | | |
| USB Serial | GPIO1/GPIO3 | Debug monitor @ 115200 baud | Built-in USB-to-Serial |

### Wiring Diagram

```
ESP32                          Arduino                  Neato LIDAR
┌─────────────┐               ┌─────────────┐         ┌─────────────┐
│             │               │             │         │             │
│  GPIO26 (RX)│◄──────────────│ TX (Soft)   │         │             │
│  GPIO27 (TX)│──────────────►│ RX (Soft)   │         │             │
│  GND        │◄──────────────│ GND         │         │             │
│             │               │             │         │             │
│  GPIO16 (RX)│◄──────────────────────────────────────│ TX (orange) │
│  GPIO21(PWM)│──────────────────────────────────────►│ Motor Ctrl  │
│  GND        │◄──────────────────────────────────────│ GND         │
│             │               │             │         │             │
└─────────────┘               └─────────────┘         └─────────────┘

      WiFi: 192.168.68.52:5000
```

**Important Notes**:
- Serial1 (Arduino): 9600 baud to match Arduino's SoftwareSerial configuration
- Serial2 (LIDAR): 115200 baud for Neato LIDAR data packets
- PWM (LIDAR Motor): 25kHz, 0-255 duty cycle for motor speed control (typically 200-250 RPM)

## Software Setup

### Dependencies

The firmware requires the following library:
- **RobotLink**: Binary framing protocol library

Library is located in: `/Users/magnus/Documents/PlatformIO/Projects/round robot motor firmware/lib/RobotLink`

### WiFi Configuration

Edit [src/main.cpp](src/main.cpp) to configure WiFi credentials:

```cpp
const char* WIFI_SSID = "SurfsUp";        // Your WiFi network name
const char* WIFI_PASSWORD = "surf-1111";  // Your WiFi password
const uint16_t UDP_PORT = 5000;            // UDP port for host communication
```

### Build and Upload

```bash
cd "/Users/magnus/Documents/PlatformIO/Projects/round robot motor firmware"
pio run --target upload
```

### Monitor ESP32 Debug Output

```bash
pio device monitor --baud 115200
```

Expected boot output:
```
========================================
ESP32 Robot Bridge - INCREMENTAL BUILD
========================================

========================================
WiFi Setup
========================================
Connecting to SurfsUp. Connected!
IP Address: 192.168.68.52
UDP Port: 5000

========================================
Arduino Serial Setup
========================================
Serial1: GPIO26(RX) GPIO27(TX) @ 9600 baud
RX Buffer: 512 bytes (2x default)
RobotLink initialized
Waiting 2 seconds for Arduino boot...

========================================
UDP Setup
========================================
UDP listening on port 5000
RobotLinkUDP initialized

========================================
System Ready!
========================================
Waiting for Host to connect...
```

## Critical Bug Fix: ESP32 Boot Crash

### Problem

The ESP32 was continuously crashing during `setupArduino()` when attempting to allocate a 2048-byte RX buffer for Serial1:

```cpp
arduinoSerial.setRxBufferSize(2048);  // ← CAUSED CRASH
```

**Symptom**: ESP32 would print "Arduino Serial Setup" but never complete initialization, resulting in continuous reboots.

### Root Cause

ESP32 heap memory insufficient to allocate 2048 bytes for the Serial1 RX buffer. This memory allocation failure caused a system crash before `loop()` could execute.

### Solution

Reduced RX buffer size from 2048 to 512 bytes in [src/main.cpp:64](src/main.cpp#L64):

```cpp
// Using 512 bytes (2x default) - larger values may cause ESP32 to crash
arduinoSerial.setRxBufferSize(512);
```

**Why 512 bytes is sufficient**:
- Default Arduino Serial buffer: 256 bytes
- 2x increase provides adequate buffering for 20Hz ODOM data at 9600 baud
- Prevents WiFi-induced delays from causing data loss
- Stays within ESP32 safe memory allocation limits

## RobotLink Protocol

### Message Types

| Type | Value | Direction | Description |
|------|-------|-----------|-------------|
| ODOM | 0x01 | Arduino → Host | Odometry data (timestamp, encoders, pose) |
| ENABLE_STREAM | 0x14 | Host → Arduino | Enable/disable odometry streaming |
| SET_VEL | 0x15 | Host → Arduino | Set left/right wheel velocities |
| STOP | 0x16 | Host → Arduino | Emergency stop |

### ODOM Message Format

**Payload**: 16 bytes, struct format `<IhhIIh`

| Field | Type | Bytes | Description |
|-------|------|-------|-------------|
| timestamp | uint32 | 4 | System time in milliseconds |
| dL | int16 | 2 | Left encoder delta (ticks since last message) |
| dR | int16 | 2 | Right encoder delta (ticks since last message) |
| x | uint32 | 4 | X position (millimeters) |
| y | uint32 | 4 | Y position (millimeters) |
| theta | int16 | 2 | Heading angle (milliradians) |

**Example Python parsing**:
```python
import struct
from robotlink import RobotLink

robot = RobotLink(host='192.168.68.52', port=5000)
robot.enable_stream(enable=True, interval_ms=50)

result = robot.receive_frame()
if result:
    msg_type, payload = result
    if msg_type == 0x01:  # ODOM
        timestamp, dL, dR, x, y, theta = struct.unpack('<IhhIIh', payload)
        print(f'Odom: t={timestamp}ms dL={dL} dR={dR} x={x}mm y={y}mm θ={theta}mrad')
```

## Usage Examples

### Python Client

Located in: `/Users/magnus/Documents/PlatformIO/Projects/round robot motor firmware/python/`

#### Basic Odometry Streaming

```python
from robotlink import RobotLink
import struct
import time

robot = RobotLink(host='192.168.68.52', port=5000)

# Enable odometry at 50ms intervals (20Hz)
robot.enable_stream(enable=True, interval_ms=50)

# Receive and parse odometry
for _ in range(100):
    result = robot.receive_frame()
    if result:
        msg_type, payload = result
        if msg_type == 0x01:
            vals = struct.unpack('<IhhIIh', payload)
            print(f'ODOM: t={vals[0]}ms dL={vals[1]} dR={vals[2]}')
    time.sleep(0.05)

robot.close()
```

#### Motor Control

```python
from robotlink import RobotLink
import time

robot = RobotLink(host='192.168.68.52', port=5000)

# Enable odometry
robot.enable_stream(enable=True, interval_ms=50)
time.sleep(1)

# Drive forward at 0.5 m/s
robot.set_velocity(0.5, 0.5)
time.sleep(3)

# Stop motors
robot.stop()

robot.close()
```

### Verified Test Results

**Test Date**: 2026-01-06
**Configuration**: ESP32 @ 192.168.68.52:5000, Arduino @ 9600 baud

| Test | Result | Details |
|------|--------|---------|
| WiFi Connection | ✓ PASS | ESP32 connected to "SurfsUp" |
| UDP Communication | ✓ PASS | Host ↔ ESP32 bidirectional |
| Serial1 Communication | ✓ PASS | ESP32 ↔ Arduino @ 9600 baud |
| ENABLE_STREAM Command | ✓ PASS | Arduino responds with ODOM stream |
| ODOM Message Parsing | ✓ PASS | 160+ messages received and parsed |
| SET_VEL Command | ✓ PASS | Command sent and acknowledged |
| STOP Command | ✓ PASS | Motors stopped, dL=0 dR=0 |
| Full Communication Chain | ✓ PASS | End-to-end bidirectional verified |

**Odometry Message Rate**: 20 Hz (one message every 50ms)
**Message Success Rate**: 100% (160/160 messages received)

## Firmware Code Structure

### Main Components

**File**: [src/main.cpp](src/main.cpp)

```cpp
// WiFi and network
#include <WiFi.h>
#include <WiFiUdp.h>
#include <RobotLink.h>
#include <RobotLinkUDP.h>

// Serial configuration
#define ARDUINO_BAUD 9600
#define ARDUINO_RX_PIN 26  // ESP32 RX ← Arduino TX
#define ARDUINO_TX_PIN 27  // ESP32 TX → Arduino RX

// Global objects
HardwareSerial arduinoSerial(1);        // Serial1
RobotLink::Link* arduinoLink;           // Arduino protocol handler
RobotLink::UDPStream* udpStream;        // UDP stream handler
RobotLink::Link* hostLink;              // Host protocol handler
```

### Setup Functions

1. **setupWiFi()**: Connects to WiFi network, assigns static IP
2. **setupArduino()**: Configures Serial1 with 512-byte RX buffer, initializes RobotLink
3. **setupUDP()**: Starts UDP server on port 5000

### Main Loop

The `loop()` function polls for messages in both directions:

```cpp
void loop() {
  // Poll FROM Arduino → forward TO Host
  arduinoLink->poll([](uint8_t type, const uint8_t* payload, uint8_t len) {
    if (udpStream && udpStream->hasClient()) {
      hostLink->sendFrame(type, payload, len);
    }
  });

  // Poll FROM Host → forward TO Arduino
  if (hostLink) {
    hostLink->poll([](uint8_t type, const uint8_t* payload, uint8_t len) {
      // Track client connection
      if (udpStream->hasClient()) {
        // Log new clients
      }

      // Forward to Arduino
      arduinoLink->sendFrame(type, payload, len);

      // Debug logging for important commands
      if (type == RobotLink::MSG_ENABLE_STREAM) {
        Serial.println("  Host → Arduino: ENABLE_STREAM");
      }
    });
  }

  delay(1);  // Prevent watchdog issues
}
```

## Troubleshooting

### ESP32 Continuously Reboots

**Symptom**: ESP32 prints "Arduino Serial Setup" but crashes before completing boot

**Solution**: Verify RX buffer size in `setupArduino()` is set to 512 bytes (not 2048):
```cpp
arduinoSerial.setRxBufferSize(512);
```

### No Odometry Messages Received

**Checks**:
1. Verify Arduino is sending data: Check Arduino USB serial monitor for "waiting for odo enable"
2. Check wiring: ESP32 GPIO26(RX) ← Arduino TX, ESP32 GPIO27(TX) → Arduino RX
3. Verify baud rate: Both ESP32 Serial1 and Arduino SoftwareSerial must be 9600
4. Check GND connection: ESP32 and Arduino must share common ground

### Host Cannot Connect via UDP

**Checks**:
1. Verify ESP32 IP address from debug monitor
2. Ensure ESP32 and host are on same WiFi network
3. Check firewall settings on host computer (port 5000 UDP)
4. Test with: `echo "test" | nc -u 192.168.68.52 5000`

### Messages Being Dropped

**Solution**: The 512-byte RX buffer should prevent data loss during WiFi operations. If messages are still being dropped:
- Reduce odometry streaming rate: `robot.enable_stream(enable=True, interval_ms=100)`  # 10Hz instead of 20Hz
- Check WiFi signal strength
- Consider increasing RX buffer to 1024 bytes (max safe value)

## Future Enhancements

- [x] Add LIDAR integration on ESP32 Serial2 - **COMPLETED (2026-01-06)**
- [ ] Implement connection timeout and auto-reconnect
- [ ] Add statistics reporting (message counts, error rates)
- [ ] Support multiple simultaneous host connections
- [ ] Add WiFi AP fallback mode for field deployment
- [ ] Add LIDAR status streaming (similar to odometry)
- [ ] Implement LIDAR scan visualization tools

## License

This firmware is part of the Round Robot project.

## Author

**Magnus** - 2026-01-06

## Change Log

### 2026-01-06
- **CRITICAL FIX**: Reduced Serial1 RX buffer from 2048 to 512 bytes to prevent ESP32 boot crash
- Updated debug output to correctly report 512-byte buffer size
- Verified full communication chain: Host ↔ ESP32 ↔ Arduino
- Tested motor control and odometry parsing at 0.5 m/s
- Confirmed 160+ odometry messages received and parsed successfully

### Initial Release
- WiFi/UDP bridge implementation
- RobotLink protocol integration
- Bidirectional message forwarding
- Debug logging for important messages
