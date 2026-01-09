# PID Tuning Guide

Complete guide for tuning the robot's motor PID controllers using the automated tuning system.

## Overview

This guide covers:
1. Hardware setup and preparation
2. Compiling and uploading the PID tuner firmware
3. Running the automated tuning script
4. Manual fine-tuning (if needed)
5. Applying tuned parameters to main firmware

**Time required**: 15-30 minutes
**Difficulty**: Beginner to Intermediate

## What Gets Tuned

The auto-tuner calibrates the following parameters for each motor:

### Deadband Compensation
- **Min Start PWM**: Minimum PWM to overcome static friction (stiction)
- **Min Running PWM**: Minimum PWM to maintain motion (dynamic friction)

### PID Gains
- **Kp**: Proportional gain (responds to current error)
- **Ki**: Integral gain (eliminates steady-state error)
- **Kd**: Derivative gain (dampens oscillations, typically 0)

### Feed-Forward Gains
- **Kff_v**: Velocity feed-forward (reduces lag)
- **Kff_a**: Acceleration feed-forward (improves transient response)

## Hardware Requirements

### Before You Start

1. **Arduino Uno** with motor control firmware
2. **USB cable** connected to computer
3. **Adequate power supply** for motors (not USB power!)
4. **Robot lifted off ground** or wheels free to spin
5. **Python 3.7+** with required packages:
   ```bash
   pip3 install pyserial numpy
   ```

### Safety Checklist

- [ ] Robot is securely lifted (wheels don't touch ground)
- [ ] Motors have proper power supply connected
- [ ] No loose wires or connections
- [ ] Emergency stop accessible (Ctrl+C or power switch)

## Step 1: Upload PID Tuner Firmware

The PID tuner firmware provides a serial console interface for parameter adjustment and telemetry streaming.

### Compile and Upload

```bash
cd "Arduino Motor control"
pio run -e pid_tuner --target upload
```

### Verify Connection

Open serial monitor at 9600 baud:
```bash
pio device monitor -e pid_tuner
```

You should see:
```
========================================
  PID Tuner v1.0
========================================

Commands available:
  v <left> <right>  - Set velocity targets
  kp <value>        - Set Kp gain
  ...
```

Type `status` to verify firmware is running correctly.

## Step 2: Run Automated Tuning

The `motor_tuner.py` script automates the entire tuning process.

### Basic Usage

```bash
# Tune both motors (recommended)
python3 motor_tuner.py --motor both

# Tune only left motor
python3 motor_tuner.py --motor left

# Specify serial port
python3 motor_tuner.py --port /dev/ttyUSB0 --motor both
```

### What the Script Does

The auto-tuner runs 4 phases for each motor:

#### Phase 1: Deadband Detection (2-3 minutes)
- Tests incremental PWM values from 20 to 150
- Finds minimum PWM to start motor from rest
- Finds minimum PWM to keep motor running
- Measures both forward and reverse directions

**Watch for**:
- Motor should start spinning at some PWM value
- If motor doesn't start by PWM 150, check:
  - Power supply voltage
  - Motor connections
  - Motor health

#### Phase 2: System Identification (5-10 seconds per test)
- Applies step input at 0.2 m/s
- Records velocity response
- Measures steady-state velocity and rise time
- Calculates system parameters (K, tau)

**Watch for**:
- Motor should reach approximately the target velocity
- Response should be relatively smooth
- If velocity is way off or oscillates wildly:
  - Check encoder connections
  - Verify TICKS_PER_REV is correct (714)

#### Phase 3: PID Calculation (instant)
- Uses Ziegler-Nichols method for PI tuning
- Calculates conservative gains for stability
- Computes feed-forward gains

#### Phase 4: Parameter Application (instant)
- Sends tuned parameters to firmware
- Parameters are active but NOT saved to EEPROM yet

### Reading the Output

```
--- Finding Min Start PWM (left motor, forward) ---
Testing PWM=20... Max velocity: 0.0000 m/s
Testing PWM=25... Max velocity: 0.0000 m/s
Testing PWM=30... Max velocity: 0.0015 m/s
Testing PWM=35... Max velocity: 0.0523 m/s
✓ Motor started! Min Start PWM = 35

--- Finding Min Running PWM (left motor, forward) ---
Starting motor at PWM=65...
Testing PWM=60... Avg velocity: 0.0821 m/s
Testing PWM=55... Avg velocity: 0.0623 m/s
Testing PWM=50... Avg velocity: 0.0412 m/s
Testing PWM=45... Avg velocity: 0.0203 m/s
Testing PWM=40... Avg velocity: 0.0089 m/s
Testing PWM=35... Avg velocity: 0.0021 m/s
✓ Motor stopped. Min Running PWM = 40

--- Step Response Test (left motor, target=0.2 m/s) ---
Applying step input: 0.2 m/s
Recording for 5.0 seconds...
Recorded 247 data points

System Identification Results:
  Target velocity: 0.200 m/s
  Steady-state velocity: 0.198 m/s
  Steady-state error: 0.002 m/s (1.0%)
  Rise time (tau): 0.312 s
  Average PWM: 98.2
  Process gain K: 0.002016 m/s per PWM

--- Calculating PID Gains ---
  Ziegler-Nichols PI Tuning:
    Kp = 953.234
    Ki = 763.511
    Kd = 0.000
  Feed-forward gains:
    Kff_v = 496.032 (PWM per m/s)
```

### Tuning Results Summary

At the end, you'll see:
```
============================================================
TUNING SUMMARY
============================================================

LEFT MOTOR:
  Deadband:
    Start PWM (fwd): 35
    Running PWM (fwd): 40
  System Parameters:
    Process gain K: 0.002016 m/s per PWM
    Time constant tau: 0.312 s
  PID Gains:
    Kp: 953.234
    Ki: 763.511
    Kd: 0.000
    Kff_v: 496.032

RIGHT MOTOR:
  (similar data...)

✓ Results saved to: motor_tuning_20260108_143022.json
```

## Step 3: Test the Tuned Parameters

Before saving to EEPROM, test the parameters with the serial console.

### Manual Testing

Connect to serial monitor:
```bash
pio device monitor -e pid_tuner
```

Try different velocities:
```
v 0.1 0.1    # Both motors slow (10 cm/s)
v 0.2 0.2    # Both motors medium (20 cm/s)
v 0.5 0.5    # Both motors fast (50 cm/s)
v 0 0        # Stop
```

### What to Look For

**Good response**:
- Motors reach target velocity within 1-2 seconds
- Steady-state error < 5%
- No oscillation or hunting
- Smooth acceleration

**Poor response**:
- Motors overshoot and oscillate → Reduce Kp
- Slow to reach target → Increase Kp
- Never quite reaches target → Increase Ki
- Oscillates at steady-state → Reduce Ki

## Step 4: Manual Fine-Tuning (Optional)

If auto-tuned parameters aren't perfect, you can manually adjust them.

### Adjusting PID Gains

**If motor overshoots and oscillates**:
```
kp 800     # Reduce Kp by 20-30%
ki 600     # Reduce Ki proportionally
```

**If motor is too slow to respond**:
```
kp 1100    # Increase Kp by 20-30%
ki 900     # Increase Ki proportionally
```

**If steady-state error remains**:
```
ki 1000    # Increase Ki gain
```

### Adjusting Deadband

**If motor doesn't start smoothly from rest**:
```
db 45 45   # Increase deadband
```

**If motor "jerks" at low speeds**:
```
db 35 35   # Reduce deadband slightly
```

### Adjusting Filter

**If velocity measurement is noisy**:
```
filter 0.2   # More filtering (slower response)
```

**If response feels sluggish**:
```
filter 0.4   # Less filtering (faster response)
```

### Adjusting Acceleration Limit

**If wheels slip during acceleration**:
```
accel 1.5    # Reduce max acceleration (m/s²)
```

## Step 5: Save to EEPROM

Once you're satisfied with the parameters, save them permanently:

```
save
```

The firmware will display:
```
Saving parameters to EEPROM...
✓ Saved successfully!
```

These parameters will now load automatically on every startup.

### Verify Saved Parameters

Reset the Arduino (power cycle or press reset button).

Reconnect and type:
```
status
```

Verify the parameters match your tuned values.

## Step 6: Update Main Firmware

Now that you have tuned parameters, update `main.cpp` to use them.

### Apply to main.cpp

Edit [Arduino Motor control/src/main.cpp](../src/main.cpp) and update the PID controller with your tuned values:

```cpp
// From tuning results:
leftPID.Kp = 953.2f;
leftPID.Ki = 763.5f;
leftPID.Kd = 0.0f;
leftPID.deadband_forward = 35.0f;
leftPID.deadband_reverse = 35.0f;
leftPID.Kff_v = 496.0f;
leftPID.filter_alpha = 0.3f;

rightPID.Kp = 945.1f;
rightPID.Ki = 755.2f;
rightPID.Kd = 0.0f;
rightPID.deadband_forward = 38.0f;
rightPID.deadband_reverse = 38.0f;
rightPID.Kff_v = 502.1f;
rightPID.filter_alpha = 0.3f;
```

### Rebuild and Upload Main Firmware

```bash
pio run -e uno --target upload
```

### Test with Robot UI

Start the robot UI and test velocity control:
```bash
cd ../robot-ui/server
python3 robot_ui_server.py
```

Open browser to `http://localhost:5000` and test:
- Forward/backward motion
- Turning (differential drive)
- Straight-line tracking
- Velocity response

## Troubleshooting

### Motor doesn't start during tuning

**Possible causes**:
1. Insufficient power supply voltage
2. Motor driver issue
3. Motor mechanical problem
4. Encoder wiring disconnected

**Solutions**:
- Check battery/power supply voltage (should be 7-12V)
- Verify motor runs with direct PWM: `pwm 100 0`
- Check for mechanical binding
- Verify encoder counts changing when motor runs

### Encoders not counting

**Symptoms**: Velocity always shows 0.000 m/s

**Solutions**:
- Run encoder test firmware first: `pio run -e encoder_test --target upload`
- Check encoder wiring (pins 2, 3, 10, 11)
- Verify encoders are powered
- Check for loose connections

### Tuned gains cause oscillation

**Symptoms**: Motor oscillates/buzzes at target velocity

**Solutions**:
1. Reduce Kp by 30-50%: `kp 600`
2. Reduce Ki proportionally: `ki 450`
3. Increase velocity filter: `filter 0.2`
4. Check mechanical coupling (loose wheels, friction)

### One motor behaves differently than the other

**This is normal!** Motors have manufacturing variations.

**Solutions**:
- Tune each motor individually
- Accept small differences in parameters
- If one motor is significantly worse:
  - Check for mechanical issues
  - Verify encoder quality
  - Consider replacing motor

### Parameters don't save to EEPROM

**Symptoms**: Parameters reset after power cycle

**Solutions**:
- Ensure `save` command completed successfully
- Check for EEPROM library in platformio.ini
- Try `load` command to verify EEPROM is accessible
- Re-upload firmware if EEPROM might be corrupted

## Advanced Topics

### Custom Tuning Algorithm

The auto-tuner uses Ziegler-Nichols PI tuning, which is conservative but reliable. For more aggressive tuning:

1. Multiply Kp by 1.5
2. Multiply Ki by 1.5
3. Test for stability
4. Back off if oscillation occurs

### Velocity-Dependent Tuning

For robots that operate at widely varying speeds, consider:
- Higher gains for high-speed operation
- Lower gains for precision low-speed control
- Implementing gain scheduling in firmware

### Angular Velocity Control (Future)

The current system tunes linear velocity. For angular control:
1. Tune both motors individually first
2. Use differential velocity for turning
3. Add heading PID controller (future enhancement)
4. Implement feed-forward for turn rate

## References

- [TUNING_THEORY.md](TUNING_THEORY.md) - Technical theory behind tuning
- [hardware-pinout.md](hardware-pinout.md) - Hardware configuration
- Ziegler-Nichols tuning method
- Back-calculation anti-windup algorithm

## Results Archive

Tuning results are saved as JSON files with timestamps:
- `motor_tuning_YYYYMMDD_HHMMSS.json`

These files contain:
- Deadband values
- System identification data
- Calculated PID gains
- Test conditions

Keep these files for reference and comparison!
