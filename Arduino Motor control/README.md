# Arduino Motor Control Firmware

Differential drive robot motor control firmware for Arduino Uno with Monster Moto Shield. Features PID velocity control with deadband compensation and RobotLink binary protocol for communication.

## Hardware

- **Platform**: Arduino Uno (ATmega328P)
- **Motor Driver**: Monster Moto Shield (H-Bridge)
- **Robot Type**: Differential Drive (Two-Wheel)
- **Wheel Diameter**: 82 mm
- **Wheelbase**: 240 mm
- **Encoder Resolution**: 360 ticks per revolution

## Features

### Core Functionality
- ✅ Dual PID velocity controllers (one per motor)
- ✅ Deadband compensation for smooth low-speed operation
- ✅ Quadrature encoder reading with hardware interrupts
- ✅ Odometry calculation and pose tracking
- ✅ EEPROM configuration persistence
- ✅ RobotLink binary protocol support
- ✅ 20Hz control loop for precise motion control

### PID Control
- **Default Gains**: Kp=10.0, Ki=5.0, Kd=0.1
- **Anti-windup**: Prevents integral saturation
- **Deadband Compensation**: Automatically compensates for motor friction
  - Forward deadband: 30 PWM units
  - Reverse deadband: 30 PWM units

### Protocol Support
The firmware implements the **RobotLink protocol** with 14 message types for comprehensive robot control.

**Core Protocol** (see [PROTOCOL.md](docs/PROTOCOL.md)):
- `MSG_ODOM (0x01)`: Odometry streaming (encoder deltas, pose, timestamp)
- `MSG_CMD_VEL (0x02)`: Differential drive commands (v, w)
- `MSG_PING/PONG (0x7E/0x7F)`: Connection testing

**Extended Protocol** ([Quick Reference](docs/PROTOCOL_QUICK_REF.md)):
- `MSG_SET_VEL (0x10)`: Direct motor velocity control
- `MSG_SET_PID (0x11)`: Runtime PID tuning
- `MSG_SET_DEADBAND (0x12)`: Deadband compensation adjustment
- `MSG_ENABLE_STREAM (0x14)`: Control odometry streaming
- `MSG_GET_CONFIG (0x15)`: Request robot configuration
- `MSG_CONFIG_RESP (0x16)`: Configuration response
- `MSG_SAVE_CONFIG (0x17)`: Persist settings to EEPROM
- `MSG_LOAD_CONFIG (0x18)`: Restore settings from EEPROM
- `MSG_ZERO_ENCODERS (0x19)`: Reset encoders and pose
- `MSG_STOP (0x1A)`: Emergency stop

See [PROTOCOL.md](docs/PROTOCOL.md) for complete specification with payload structures and [PROTOCOL_QUICK_REF.md](docs/PROTOCOL_QUICK_REF.md) for a quick reference card.

## Building and Uploading

### Using PlatformIO

```bash
# Build firmware
pio run

# Upload to Arduino
pio run --target upload

# Monitor serial output
pio device monitor -b 115200
```

### Manual Upload
1. Connect Arduino Uno via USB
2. Build the project: `pio run`
3. Upload: `pio run --target upload`

## Usage Examples

### Basic Velocity Control

Send velocity commands using the RobotLink protocol:

```python
# Example using Python (requires robotlink.py library)
from robotlink import RobotLink

# IMPORTANT: Connect to Arduino's SoftwareSerial (A0/A1), NOT USB!
# For direct serial connection, use USB-to-Serial adapter on A0/A1
# For wireless, connect via ESP32 WiFi bridge (see ESP32 firmware docs)

# Option 1: Direct serial connection (USB-to-Serial adapter on A0/A1)
robot = RobotLink(serial_port='/dev/ttyUSB0', baud=9600)

# Option 2: Via ESP32 WiFi/UDP bridge (recommended)
robot = RobotLink(host='192.168.68.52', port=5000)  # ESP32 IP address

# Move forward at 0.15 m/s
robot.send_cmd_vel(v=150, w=0)  # v in mm/s, w in mrad/s

# Turn in place
robot.send_cmd_vel(v=0, w=500)  # rotate at 500 mrad/s

# Direct motor control
robot.set_velocity(left=0.1, right=0.1)  # both motors at 0.1 m/s

# Stop
robot.stop()
```

**Note**: The Arduino's USB port (Hardware Serial) is used for **debug output only**. RobotLink protocol communication happens on **A0/A1 at 9600 baud**. See [SOFTWARESERIAL_UPDATE.md](SOFTWARESERIAL_UPDATE.md) for migration details.

### Configuration Management

```python
# Enable odometry streaming at 50ms intervals
robot.enable_stream(enable=True, interval_ms=50)

# Update PID gains
robot.set_pid(Kp=15.0, Ki=8.0, Kd=0.2)

# Update deadband compensation
robot.set_deadband(
    left_forward=35.0,
    left_reverse=32.0,
    right_forward=38.0,
    right_reverse=35.0
)

# Save to EEPROM (survives power cycles)
robot.save_config()

# Reset encoders
robot.zero_encoders()
```

### Reading Odometry

```python
# Register callback for odometry messages
def on_odom(payload):
    print(f"Time: {payload.timestamp_ms}ms")
    print(f"Encoder deltas: L={payload.dL_ticks}, R={payload.dR_ticks}")
    print(f"Pose: x={payload.x_mm}mm, y={payload.y_mm}mm, θ={payload.th_mrad}mrad")

robot.register_callback(MessageType.MSG_ODOM, on_odom)

# Enable streaming
robot.enable_stream(True, 50)

# Process messages
while True:
    robot.process_messages(timeout=0.1)
```

## Tuning the PID Controller

The default PID gains should work for most DC motors, but you may need to tune them for your specific setup:

### Manual Tuning Process

1. **Test default values**:
   ```python
   robot.set_velocity(0.1, 0.1)  # slow speed
   ```

2. **Adjust proportional gain (Kp)** if needed:
   - Too low: Sluggish response, doesn't reach target
   - Too high: Oscillation, overshooting
   - Start with 10.0 and adjust in steps of 2-5

3. **Adjust integral gain (Ki)**:
   - Too low: Steady-state error
   - Too high: Oscillation, instability
   - Start with 5.0 and adjust in steps of 1-2

4. **Adjust derivative gain (Kd)**:
   - Usually small (0.1-1.0)
   - Reduces overshoot but can amplify noise
   - Start with 0.1 and increase if needed

5. **Save when satisfied**:
   ```python
   robot.save_config()
   ```

### Deadband Tuning

If motors don't start smoothly or oscillate at low speeds:

1. **Find minimum PWM to start motor** (from standstill):
   - Manually test PWM values 20, 25, 30, 35...
   - Note the value where motor just starts moving

2. **Find minimum PWM to keep motor running**:
   - Start motor at high speed
   - Gradually reduce PWM until motor stops
   - Note the value just before stopping

3. **Set deadband values**:
   ```python
   robot.set_deadband(
       left_forward=<start_pwm>,
       left_reverse=<start_pwm>,
       right_forward=<start_pwm>,
       right_reverse=<start_pwm>
   )
   ```

4. **Test and adjust**:
   - Try very low speeds: `robot.set_velocity(0.05, 0.05)`
   - Motors should start smoothly without jerking
   - Fine-tune values if needed

## Pinout Reference

See [hardware-pinout.md](docs/hardware-pinout.md) for complete pin assignments.

**Quick Reference**:
- Left Motor: INA=7, INB=8, PWM=5
- Right Motor: INA=4, INB=9, PWM=6
- Left Encoder: A=2 (INT0), B=10
- Right Encoder: A=3 (INT1), B=11
- RobotLink Serial: RX=A0, TX=A1 (SoftwareSerial @ 9600 baud)
- Debug Serial: USB (Hardware Serial @ 115200 baud)

## Performance Specifications

- **Firmware Version**: 1.1 (SoftwareSerial)
- **Control Loop**: 20 Hz (50ms update interval)
- **Odometry Rate**: Configurable (default 20 Hz)
- **RobotLink Serial**: 9600 bps (SoftwareSerial on A0/A1)
- **Debug Serial**: 115200 bps (USB/Hardware Serial)
- **Maximum Velocity**: ~0.3 m/s (recommended operating range)
- **Minimum Velocity**: ~0.05 m/s (with proper deadband tuning)
- **RAM Usage**: 460 bytes (22.5% of 2KB)
- **Flash Usage**: 13136 bytes (40.7% of 32KB)

## Troubleshooting

### Motors don't move
- Check power supply to Monster Moto Shield
- Verify motor connections
- Check that target velocity is not zero
- Increase deadband values if needed

### Motors oscillate or jerk
- Reduce Kp gain
- Increase deadband values
- Check for mechanical binding

### Encoders not counting
- Verify encoder wiring (A, B, GND, VCC)
- Check pull-up resistors enabled
- Test with multimeter: should see pulses on encoder pins

### Serial communication fails
- **For RobotLink**: Verify baud rate is 9600 on A0/A1
- **For Debug**: Verify baud rate is 115200 on USB
- Check wiring: A0=RX, A1=TX (remember TX-RX crossover!)
- Ensure common ground between Arduino and host
- Try resetting Arduino
- Verify CRC16 calculation in host code
- See [SOFTWARESERIAL_UPDATE.md](SOFTWARESERIAL_UPDATE.md) for troubleshooting

### Odometry drift
- Calibrate wheel diameter and wheelbase
- Check for wheel slippage
- Verify encoder resolution matches actual encoder
- Consider encoder inversion settings

## Advanced Configuration

### Changing Robot Parameters

Edit [main.cpp](src/main.cpp) lines 29-33:

```cpp
const float WHEEL_DIAMETER = 0.082f;   // meters
const float WHEELBASE = 0.24f;         // meters
const float TICKS_PER_REV = 360.0f;    // encoder resolution
```

After changing, rebuild and upload firmware.

### Adjusting Control Loop Rate

Edit [main.cpp](src/main.cpp) line 146:

```cpp
const unsigned long CONTROL_INTERVAL = 50;  // 50ms = 20Hz
```

Lower values = faster control (but more CPU usage).

### Motor Direction Inversion

If motors run backward:
- Hardware inversion is applied to right motor (line 201)
- For left motor inversion, negate `pwmLeft` on line 200
- Or swap INA/INB pin connections

## License

See LICENSE file for details.

## References

- [Hardware Pinout Documentation](docs/hardware-pinout.md)
- [RobotLink Protocol Specification](docs/PROTOCOL.md)
- [Monster Moto Shield Datasheet](https://www.sparkfun.com/products/10182)
