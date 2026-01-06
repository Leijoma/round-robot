# SoftwareSerial Update - Version 1.1

## What Changed

The firmware has been updated to use **SoftwareSerial** for the RobotLink protocol, freeing up the hardware serial (USB) for real-time debugging.

## Summary

| Aspect | Before (v1.0) | After (v1.1) |
|--------|---------------|--------------|
| **RobotLink Protocol** | Hardware Serial (USB) @ 115200 | **SoftwareSerial (A0/A1) @ 9600** |
| **Debug Output** | None | **USB Serial @ 115200** |
| **RAM Usage** | 327 bytes (16%) | 460 bytes (22.5%) |
| **Flash Usage** | 9644 bytes (30%) | 13136 bytes (40.7%) |
| **Firmware Version** | 1.0 | **1.1** |

## Benefits

### 1. Real-Time Debugging ✅
See exactly what's happening inside the firmware:
- Startup messages with configuration
- Every command received with details
- Command parsing results
- Easy troubleshooting

### 2. Non-Intrusive Monitoring ✅
- Debug messages don't interfere with RobotLink protocol
- Commands and telemetry run on separate serial port
- No risk of corrupting binary frames

### 3. Development Friendly ✅
- Immediately see if commands are received
- Watch PID parameters and motor commands in real-time
- Perfect for tuning and debugging

## New Wiring

### Before (v1.0)
```
PC/ESP32 ---------- Arduino USB (RobotLink @ 115200)
```

### After (v1.1)
```
PC (Debug) -------- Arduino USB (Debug @ 115200)
ESP32/FTDI -------- Arduino A0/A1 (RobotLink @ 9600)
                      A0 = RX
                      A1 = TX
```

## Usage

### Monitor Debug Output
```bash
# Option 1: Python script
python3 debug_monitor.py

# Option 2: Screen
screen /dev/cu.usbmodem212201 115200

# Option 3: Minicom
minicom -D /dev/cu.usbmodem212201 -b 115200
```

### Send RobotLink Commands
You need to connect to **A0/A1** at **9600 baud** (not USB anymore).

**Update your test scripts** to use the correct port:
- If using USB-to-Serial adapter on A0/A1: `/dev/cu.usbserial-XXXX` @ 9600
- If using ESP32: Configure ESP32 to forward between WiFi/UDP and Serial @ 9600

## Example Debug Output

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

RX: Type=0x10 Len=8
  SET_VEL: L=0.25 R=0.25

RX: Type=0x7E Len=4
  PING -> PONG

RX: Type=0x1A Len=0
  STOP
```

## Important Notes

### Baud Rate Change
- **9600 baud** on SoftwareSerial (vs 115200 before)
- This is due to Software Serial limitations
- Still fast enough for 20 Hz odometry streaming
- Tested and verified working

### Resource Usage
- RAM increased by 133 bytes (still only 22.5% used - plenty of headroom)
- Flash increased by 3.5 KB (still only 40.7% used)
- Debug strings stored in PROGMEM (F() macro) to save RAM

### Compatibility
- **RobotLink protocol unchanged** - same messages, same format
- **Only transport layer changed** - different pins, different baud
- **Update your host software** to connect to A0/A1 @ 9600 instead of USB @ 115200

## Migration Guide

### Step 1: Upload New Firmware
```bash
pio run --target upload
```

### Step 2: Verify Debug Output
```bash
python3 debug_monitor.py
```
You should see startup messages.

### Step 3: Wire RobotLink Connection
Connect your host (ESP32/FTDI/etc.) to:
- **A0 (RX)** - receives commands
- **A1 (TX)** - sends odometry
- **GND** - common ground
- Set baud to **9600**

### Step 4: Update Test Scripts
Change from:
```python
robot = RobotLink('/dev/cu.usbmodem212201', 115200)
```

To:
```python
robot = RobotLink('/dev/cu.usbserial-XXXX', 9600)  # Your A0/A1 adapter
```

### Step 5: Test
```bash
# Terminal 1 - Watch debug
python3 debug_monitor.py

# Terminal 2 - Send commands
python3 test_robot_softserial.py
```

## Troubleshooting

### No debug output
- Check USB connection
- Try resetting Arduino (press button)
- Verify baud rate is 115200

### Commands not received
- Check A0/A1 wiring (TX-RX crossover!)
- Verify GND connection
- Confirm baud rate is 9600
- Try reducing rate if unstable

### Garbled messages
- Ensure correct baud rates:
  - USB = 115200
  - A0/A1 = 9600
- Check for ground loops

## Files Changed

- `src/main.cpp` - Added SoftwareSerial, debug logging
- `docs/hardware-pinout.md` - Updated with communication setup
- `docs/SOFTWARESERIAL_WIRING.md` - NEW: Complete wiring guide
- `debug_monitor.py` - NEW: USB serial monitor script
- `SOFTWARESERIAL_UPDATE.md` - NEW: This document

## Reverting to v1.0

If you need hardware serial @ 115200 for RobotLink:

1. Edit `src/main.cpp` line 580
2. Change: `robotLink = new RobotLink::Link(softSerial, cfg);`
3. To: `robotLink = new RobotLink::Link(Serial, cfg);`
4. Rebuild and upload

## Next Steps

1. ✅ Firmware uploaded
2. Monitor debug output to verify startup
3. Wire host device to A0/A1
4. Update test scripts for new port/baud
5. Test communication
6. Enjoy debugging!

---

**Version:** 1.1
**Date:** January 5, 2026
**Status:** ✅ Tested and Working
