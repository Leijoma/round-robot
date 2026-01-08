# Robot Control Libraries

This directory contains three modular, reusable libraries for differential drive robot control.

## Libraries Overview

### 1. Encoder
**Purpose:** 2X quadrature encoder decoding with interrupt-driven accuracy

**Features:**
- 2X decoding using XOR logic (double resolution vs basic decoding)
- Direction reversal support for differential drive configurations
- Interrupt-driven for real-time accuracy
- Thread-safe count access

**Usage:**
```cpp
Encoder encoderLeft(PIN_A, PIN_B, true);  // reversed=true
encoderLeft.begin();
attachInterrupt(digitalPinToInterrupt(PIN_A), isrEncoderLeft, CHANGE);

int32_t count = encoderLeft.getCount();
encoderLeft.reset();
```

### 2. MotorControl
**Purpose:** H-bridge motor control with PWM and direction management

**Features:**
- Bidirectional control (-255 to +255 PWM range)
- Direction reversal support for differential drive
- Clean API for motor commands (stop, brake, setPWM)
- Automatic PWM saturation limiting

**Usage:**
```cpp
MotorControl motorLeft(PIN_INA, PIN_INB, PIN_PWM, true);  // reversed=true
motorLeft.begin();

motorLeft.setPWM(150);   // Forward at PWM 150
motorLeft.setPWM(-100);  // Reverse at PWM 100
motorLeft.stop();        // Coast to stop
motorLeft.brake();       // Active brake
```

### 3. PIDController
**Purpose:** Advanced PID controller with feedforward and deadband compensation

**Features:**
- PI + feedforward control (Kp, Ki, Kd, Kff_v, Kff_a)
- Deadband compensation (forward/reverse independent)
- Back-calculation anti-windup
- Low-pass velocity filtering
- Output saturation limits

**Usage:**
```cpp
PIDController pid;
pid.Kp = 50.0f;
pid.Ki = 60.0f;
pid.Kd = 0.0f;
pid.deadband_forward = 50.0f;
pid.deadband_reverse = 50.0f;
pid.filter_alpha = 0.3f;
pid.output_min = -255.0f;
pid.output_max = 255.0f;

float output = pid.update(currentVelocity, targetVelocity, dt);
float filtered = pid.getFilteredValue();
pid.reset();
```

## Tuned Parameters for This Robot

Based on systematic tuning (see [PID_TUNING_RESULTS.md](../../PID_TUNING_RESULTS.md)):

### Left Motor (Higher Friction)
```cpp
pidLeft.Kp = 50.0f;
pidLeft.Ki = 60.0f;  // Higher Ki compensates for deadband overshoot
pidLeft.Kd = 0.0f;
pidLeft.deadband_forward = 50.0f;
pidLeft.deadband_reverse = 50.0f;
pidLeft.filter_alpha = 0.3f;
```

### Right Motor (Standard)
```cpp
pidRight.Kp = 50.0f;
pidRight.Ki = 20.0f;
pidRight.Kd = 0.0f;
pidRight.deadband_forward = 35.0f;
pidRight.deadband_reverse = 35.0f;
pidRight.filter_alpha = 0.3f;
```

## Integration Guide

### Hardware Configuration Required

```cpp
// Robot parameters
const float WHEEL_DIAMETER = 0.0825f;  // 82.5mm
const float WHEELBASE = 0.244f;        // 244mm
const float TICKS_PER_REV = 714.0f;    // 2X decoding
const float METERS_PER_TICK = (PI * WHEEL_DIAMETER) / TICKS_PER_REV;

// Motor directions (tested and verified)
Encoder encoderLeft(PIN_ENC_L_A, PIN_ENC_L_B, true);    // Reversed
Encoder encoderRight(PIN_ENC_R_A, PIN_ENC_R_B, true);   // Reversed
MotorControl motorLeft(PIN_M1_INA, PIN_M1_INB, PIN_M1_PWM, true);   // Reversed
MotorControl motorRight(PIN_M2_INA, PIN_M2_INB, PIN_M2_PWM, false); // Normal
```

### Control Loop Structure

```cpp
// Timing
const unsigned long CONTROL_INTERVAL = 50;  // 50ms = 20 Hz

void loop() {
    if (millis() - lastUpdate >= CONTROL_INTERVAL) {
        float dt = (millis() - lastUpdate) / 1000.0f;
        lastUpdate = millis();

        // 1. Read encoders
        int32_t deltaL = encoderLeft.getCount() - prevCountL;
        int32_t deltaR = encoderRight.getCount() - prevCountR;

        // 2. Calculate velocities
        float velLeft = (deltaL * METERS_PER_TICK) / dt;
        float velRight = (deltaR * METERS_PER_TICK) / dt;

        // 3. Update PID controllers
        float pwmLeft = pidLeft.update(velLeft, targetVelLeft, dt);
        float pwmRight = pidRight.update(velRight, targetVelRight, dt);

        // 4. Apply to motors
        motorLeft.setPWM((int16_t)pwmLeft);
        motorRight.setPWM((int16_t)pwmRight);

        // 5. Update state
        prevCountL = encoderLeft.getCount();
        prevCountR = encoderRight.getCount();
    }
}
```

### Differential Drive Kinematics

For linear and angular velocity control:

```cpp
void setVelocity(float linear, float angular) {
    float halfWheelbase = WHEELBASE / 2.0f;
    targetVelLeft = linear - (angular * halfWheelbase);
    targetVelRight = linear + (angular * halfWheelbase);
}

// Calculate actual robot velocities from wheel velocities
float actualLinear = (velLeft + velRight) / 2.0f;
float actualAngular = (velRight - velLeft) / WHEELBASE;
```

## Performance Characteristics

| Parameter | Value | Notes |
|-----------|-------|-------|
| Control frequency | 20 Hz | 50ms update interval |
| Velocity range | 0.15-0.30 m/s | Optimal operating range |
| Straight-line accuracy | 0.7% | At 0.15 m/s |
| Angular tracking | ±15% | At 0.3-0.5 rad/s |
| Low-speed limit | >0.10 m/s | Below this, friction dominates |

## Example Implementations

See these test firmwares for complete examples:

1. **[velocity_control_test.cpp](../src/velocity_control_test.cpp)**
   - Per-motor velocity control
   - Real-time parameter tuning via serial
   - Direct wheel velocity commands

2. **[differential_drive_test.cpp](../src/differential_drive_test.cpp)**
   - Differential drive kinematics
   - Linear + angular velocity commands
   - Combined motion control

## Memory Usage

All three libraries combined:
- **RAM:** ~400 bytes (20% on Arduino Uno)
- **Flash:** ~12KB (40% on Arduino Uno)

Very efficient for embedded systems!

## Testing

Each library has been independently verified:
- **Encoder:** Verified 2X decoding, direction handling
- **MotorControl:** Verified H-bridge control, reversal
- **PIDController:** Tuned and validated for this robot

Test scripts available in project root:
- `test_straight_line.py` - Validates straight-line driving
- `test_differential_drive.py` - Tests angular velocity control

## Dependencies

None - all libraries are standalone and use only Arduino core functions.

## License

Created for the Round Robot project. Free to use and modify.
