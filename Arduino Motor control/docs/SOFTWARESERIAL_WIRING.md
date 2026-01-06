# SoftwareSerial Wiring Guide

## Overview

The firmware now uses **SoftwareSerial** on pins A0 and A1 for the RobotLink protocol, freeing up the hardware serial (USB) for debugging.

## Pin Configuration

### Communication

| Function | Pin | Baud | Purpose |
|----------|-----|------|---------|
| **USB Serial (Debug)** | TX (Pin 1), RX (Pin 0) | 115200 | Debug messages, configuration output |
| **SoftwareSerial (RobotLink)** | A0 (RX), A1 (TX) | 9600 | Robot control protocol |

###  Wiring RobotLink Protocol

```
Arduino Uno          ESP32 / FTDI / Host
-----------          --------------------
  A0 (RX)  --------->  TX
  A1 (TX)  <---------  RX
  GND      -----------  GND
```

**Important Notes:**
- **A0 (Pin 14)**: SoftwareSerial RX (receives commands from host)
- **A1 (Pin 15)**: SoftwareSerial TX (sends odometry to host)
- Baud rate: **9600** (SoftwareSerial limitation for reliable communication)
- Connect GND between Arduino and host for common reference

### Connection Options

#### Option 1: ESP32 Bridge
```
Arduino A0  -->  ESP32 TX (GPIO 17)
Arduino A1  -->  ESP32 RX (GPIO 16)
Arduino GND -->  ESP32 GND
```

#### Option 2: USB-to-Serial Adapter (FTDI, CP2102, etc.)
```
Arduino A0  -->  FTDI TX
Arduino A1  -->  FTDI RX
Arduino GND -->  FTDI GND
```

#### Option 3: Another Arduino
```
Arduino A0  -->  Other Arduino TX (Pin 1)
Arduino A1  -->  Other Arduino RX (Pin 0)
Arduino GND -->  Other Arduino GND
```

## Full Pin Assignment

### Motors & Encoders (unchanged)

| Component | Pin | Description |
|-----------|-----|-------------|
| **Left Motor (M1)** |
| INA | 7 | H-Bridge direction A |
| INB | 8 | H-Bridge direction B |
| PWM | 5 | PWM speed control |
| **Right Motor (M2)** |
| INA | 4 | H-Bridge direction A |
| INB | 9 | H-Bridge direction B |
| PWM | 6 | PWM speed control |
| **Left Encoder** |
| Phase A | 2 | INT0 (hardware interrupt) |
| Phase B | 10 | Quadrature channel B |
| **Right Encoder** |
| Phase A | 3 | INT1 (hardware interrupt) |
| Phase B | 11 | Quadrature channel B |
| **Communication** |
| USB RX | 0 | Hardware serial (debug) - 115200 |
| USB TX | 1 | Hardware serial (debug) - 115200 |
| Soft RX | A0 (14) | SoftwareSerial (RobotLink) - 9600 |
| Soft TX | A1 (15) | SoftwareSerial (RobotLink) - 9600 |

## Debug Output

Connect via USB to see real-time debug messages:

```bash
# Using Python
python3 debug_monitor.py

# Or using screen
screen /dev/cu.usbmodem212201 115200

# Or using minicom
minicom -D /dev/cu.usbmodem212201 -b 115200
```

### Debug Output Format

```
=== Arduino Motor Control Firmware ===
Version: 1.0
Debug enabled on USB Serial (115200)
RobotLink on SoftwareSerial A0/A1 (9600)

SoftwareSerial initialized at 9600 baud
Motor pins configured
Encoders configured (INT0, INT1)
Motors initialized (stopped)
Loading config from EEPROM... OK

Configuration:
  PID: Kp=10.00 Ki=5.00 Kd=0.10
  Deadband: Fwd=30.00 Rev=30.00
  Wheel diameter: 82.00 mm
  Wheelbase: 240.00 mm
  Encoder res: 360 ticks/rev

RobotLink protocol initialized

=== Setup Complete ===
Ready for commands on A0/A1 (9600 baud)
Debug messages on USB Serial
```

When commands are received on SoftwareSerial:

```
RX: Type=0x10 Len=8
  SET_VEL: L=0.25 R=0.25

RX: Type=0x7E Len=4
  PING -> PONG

RX: Type=0x1A Len=0
  STOP
```

## Using the Updated System

### 1. Monitor Debug Output
```bash
# Terminal 1 - USB Serial Debug
python3 debug_monitor.py
```

### 2. Send Commands
```bash
# Terminal 2 - RobotLink on A0/A1
# Update test script to use correct port (see below)
python3 test_robot_softserial.py
```

## Advantages of This Setup

1. **Real-time Debugging**: See what's happening inside the firmware
2. **Command Logging**: Every command received is logged to USB
3. **Non-intrusive**: Debug messages don't interfere with RobotLink protocol
4. **Easy Troubleshooting**: Immediately see if commands are being received
5. **Configuration Visibility**: See PID parameters, deadband values on startup

## Limitations

### SoftwareSerial Constraints

- **Baud rate**: Maximum reliable rate is 9600 (vs 115200 on hardware serial)
- **No interrupts**: SoftwareSerial is polled, not interrupt-driven
- **Data loss possible**: If `loop()` is blocked too long, bytes can be missed

### Recommended Practices

1. **Keep loop() fast**: Avoid blocking operations
2. **Reduce odometry rate**: 50ms (20 Hz) is good, don't go faster
3. **Use hardware serial for time-critical apps**: If you need >9600 baud
4. **Test thoroughly**: Verify no message loss at your odometry rate

## Testing

### Step 1: Verify Debug Output
```bash
python3 debug_monitor.py
```
You should see the startup message and configuration.

### Step 2: Test RobotLink Communication
```bash
# Update and run test (see next section)
python3 test_robot_softserial.py
```

### Step 3: Monitor Both
```bash
# Terminal 1
python3 debug_monitor.py

# Terminal 2
python3 test_motor_speed_softserial.py
```

Watch debug output in Terminal 1 while sending commands in Terminal 2.

## Reverting to Hardware Serial

If you need to revert to hardware serial (115200 baud):

1. Edit [main.cpp](../src/main.cpp)
2. Change line 580: `robotLink = new RobotLink::Link(Serial, cfg);`
3. Remove SoftwareSerial initialization
4. Rebuild and upload

## Troubleshooting

### No debug output
- Check USB cable (must be data cable)
- Verify port: `ls /dev/cu.*`
- Try different baud rates
- Reset Arduino (press reset button)

### Commands not received on A0/A1
- Check wiring (TX-RX crossover)
- Verify GND connection
- Reduce baud to 4800 if issues at 9600
- Check for loose connections on A0/A1

### Garbled debug messages
- Ensure 115200 baud on debug monitor
- Check for ground loops
- Verify stable 5V power supply

### Message loss
- Reduce odometry streaming rate
- Simplify debug output (remove verbose logging)
- Check for long blocking operations in code

## References

- [main.cpp](../src/main.cpp) - Firmware implementation
- [test_robot.py](../test_robot.py) - Original test script (needs port update)
- [debug_monitor.py](../debug_monitor.py) - USB serial debug monitor
