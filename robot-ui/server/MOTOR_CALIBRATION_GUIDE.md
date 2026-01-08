# Motor Calibration Guide

## Purpose

The motor calibration script finds the minimum velocity/PWM required to overcome static friction for each motor individually. Different internal friction between the left and right motors causes asynchronous start at low speeds, resulting in unwanted initial rotation.

## Prerequisites

1. **Robot Setup**:
   - Place robot on blocks OR ensure clear space (robot will move ~20cm during test)
   - Arduino firmware uploaded and running
   - ESP32 WiFi bridge connected and running
   - Robot connected to WiFi

2. **Software**:
   - Python 3.7+
   - robotlink.py in same directory

## Running Calibration

```bash
cd robot-ui/server
python3 motor_calibration.py --host 192.168.68.52
```

**What happens**:
1. Script connects to ESP32
2. For each motor (left, right) in each direction (forward, reverse):
   - Tests velocities from 0 to 0.20 m/s in 0.01 m/s steps
   - Waits 0.8 seconds at each velocity
   - Measures encoder ticks (movement)
   - Identifies first velocity where motor actually starts moving (>5 ticks)

**Duration**: ~4 minutes total (4 tests × ~1 minute each)

## Interpreting Results

### Terminal Output

The script will display:

```
LEFT MOTOR:
  Forward:
    Start velocity:       0.04 m/s
    Recommended velocity: 0.05 m/s
    Estimated PWM:        ~20

  Reverse:
    Start velocity:       0.03 m/s
    Recommended velocity: 0.04 m/s
    Estimated PWM:        ~15

RIGHT MOTOR:
  Forward:
    Start velocity:       0.06 m/s
    Recommended velocity: 0.07 m/s
    Estimated PWM:        ~30

  Reverse:
    Start velocity:       0.05 m/s
    Recommended velocity: 0.06 m/s
    Estimated PWM:        ~25

RECOMMENDED ARDUINO DEADBAND VALUES:
pidLeft.deadbandForward = 25.0f;
pidLeft.deadbandReverse = 20.0f;
pidRight.deadbandForward = 35.0f;
pidRight.deadbandReverse = 30.0f;
```

### Key Metrics

- **Start velocity**: First velocity where motor moves (>5 encoder ticks)
- **Recommended velocity**: Start velocity + safety margin
- **Estimated PWM**: Rough PWM equivalent (~20% speed = ~100 PWM)

### CSV Output

A CSV file (`motor_calibration_YYYYMMDD_HHMMSS.csv`) is saved with raw data:

```csv
motor,direction,velocity_mps,ticks,distance_mm,moving
left,forward,0.00,0,0.0,False
left,forward,0.01,0,0.0,False
left,forward,0.02,0,0.0,False
left,forward,0.03,2,1.4,False
left,forward,0.04,8,5.7,True
...
```

Use this for manual analysis or plotting.

## Next Steps

### 1. Analyze Results

**Good results**:
- Clear transition from 0 ticks to >5 ticks
- Both motors start within ~0.03 m/s of each other
- Consistent forward and reverse start velocities

**Problems**:
- Large difference between motors (>0.05 m/s): Indicates significant friction mismatch
- No clear start point: May need mechanical inspection
- Reverse different from forward (>0.02 m/s): Normal but check if extreme

### 2. Update Arduino Firmware (Story 1.7)

Copy the recommended deadband values to `Arduino Motor control/src/main.cpp`:

```cpp
// In setup() or after loadConfig():
pidLeft.deadbandForward = 25.0f;   // From calibration
pidLeft.deadbandReverse = 20.0f;
pidRight.deadbandForward = 35.0f;  // From calibration
pidRight.deadbandReverse = 30.0f;
```

### 3. Test Synchronization (Story 1.8)

After updating firmware, run synchronization tests to verify:
- Both motors start simultaneously (<50ms delta)
- Straight-line movement with minimal rotation

## Troubleshooting

### "Failed to receive odometry data"

**Solutions**:
1. Check ESP32 is connected: `ping 192.168.68.52`
2. Verify firmware is running: Check serial monitor
3. Try different IP if ESP32 has different address

### "Motor did not start moving"

**Possible causes**:
1. Robot is physically blocked or held
2. Motor connections loose
3. Battery voltage too low (<11V)
4. PWM outputs not working

**Solutions**:
- Check physical setup
- Verify motor connections
- Charge battery
- Test with robot-ui manual control first

### Motors move erratically

**Causes**:
- PID oscillation at very low speeds (normal)
- Encoder noise

**Solutions**:
- This is expected at threshold velocities
- The calibration still finds the effective start point
- Real operation will be at higher speeds (0.15-0.20 m/s)

## Technical Notes

### Why velocity instead of PWM?

The RobotLink protocol uses velocity commands (m/s), not direct PWM. The Arduino firmware's PID controller converts velocity to PWM. By finding the minimum velocity that produces movement, we indirectly measure the effective PWM threshold.

### PWM estimation formula

```
PWM ≈ (velocity / 0.2) × 100
```

This assumes:
- 0.2 m/s = "full speed" = PWM 100 (approximate)
- Linear relationship (not fully accurate at low speeds)

The actual deadband compensation happens in the PID controller by adding the deadband offset to the computed PWM.

### Safety margin (+5 PWM)

The recommended deadband values include a +5 PWM safety margin above the measured start PWM. This ensures reliable operation even with:
- Battery voltage variation
- Temperature effects on friction
- Encoder measurement uncertainty

## See Also

- [SLAM_BACKLOG.md](../../SLAM_BACKLOG.md) - Story 1.6 details
- [robotlink.py](robotlink.py) - Communication protocol
- [Arduino Motor control/src/main.cpp](../../Arduino%20Motor%20control/src/main.cpp) - Firmware with PID controller
