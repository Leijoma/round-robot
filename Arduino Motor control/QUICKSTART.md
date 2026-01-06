# Quick Start Guide

## Step 1: Verify Protocol (No Hardware Required)

Test the RobotLink protocol implementation:

```bash
python3 test_protocol.py
```

Expected output: All 9 tests should pass ✓

## Step 2: Upload Firmware to Arduino

Connect your Arduino Uno via USB and run:

```bash
pio run --target upload
```

Expected output:
```
RAM:   [==        ]  16.0% (used 327 bytes from 2048 bytes)
Flash: [===       ]  29.9% (used 9644 bytes from 32256 bytes)
...
SUCCESS
```

## Step 3: Test Communication (Hardware Required)

**IMPORTANT**: Before running motor tests, ensure:
- Robot wheels can spin freely (lift robot or place on smooth surface)
- Motor power supply is connected
- All connections are secure

Run the comprehensive test suite:

```bash
python3 test_robot.py
```

The test will:
1. Auto-detect your Arduino's serial port
2. Run 6 test sequences:
   - Connection test (PING/PONG)
   - Configuration reading
   - Encoder reading (you'll need to manually rotate wheels)
   - Motor control (you'll be prompted before motors activate)
   - PID tuning
   - EEPROM persistence

## Step 4: Manual Testing (Alternative)

If you prefer manual control, you can use the Python RobotLink class directly:

```python
from test_robot import RobotLink

# Connect (adjust port as needed)
robot = RobotLink('/dev/ttyACM0', 115200)

# Test connection
if robot.ping():
    print("Connected!")

# Enable odometry streaming
robot.enable_stream(True, 50)

# Move forward at 0.1 m/s
robot.set_velocity(0.1, 0.1)

# Wait and observe
import time
for _ in range(10):
    odom = robot.read_odometry()
    if odom:
        print(f"Encoders: L={odom['dL_ticks']}, R={odom['dR_ticks']}")
    time.sleep(0.1)

# Stop
robot.stop()
robot.close()
```

## Common Serial Ports

- **Linux**: `/dev/ttyACM0` or `/dev/ttyUSB0`
- **macOS**: `/dev/cu.usbmodem*` or `/dev/tty.usbmodem*`
- **Windows**: `COM3`, `COM4`, etc.

The test script will auto-detect available ports.

## Troubleshooting

### "No serial ports found"
- Ensure Arduino is connected via USB
- Check USB cable (must be data cable, not power-only)
- Install Arduino drivers if on Windows

### "Permission denied" on Linux
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Motors don't respond
- Check motor power supply
- Verify Monster Moto Shield connections
- Try increasing deadband values
- Check motor wiring polarity

### Encoders not counting
- Verify encoder power (3.3V or 5V depending on encoder)
- Check A and B channel connections
- Test with multimeter: should see voltage pulses when wheel rotates

## Next Steps

After all tests pass:

1. **Fine-tune PID gains** for your specific motors
2. **Calibrate deadband** values for smooth low-speed operation
3. **Save configuration** to EEPROM: `robot.save_config()`
4. **Test on ground** with robot wheels on floor (performance will differ)
5. **Integrate** with your navigation/control system

## Need Help?

- See [README.md](README.md) for detailed usage
- See [docs/TESTING.md](docs/TESTING.md) for comprehensive testing guide
- See [docs/hardware-pinout.md](docs/hardware-pinout.md) for pin assignments
- See [docs/PROTOCOL.md](docs/PROTOCOL.md) for protocol details

Good luck! 🚀
