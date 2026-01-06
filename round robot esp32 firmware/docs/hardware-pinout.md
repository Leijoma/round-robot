# Hardware Pinout Documentation

## Robot Configuration
- **Platform**: Arduino Uno (ATmega328P)
- **Motor Driver**: Monster Moto Shield (H-Bridge)
- **Robot Type**: Differential Drive (Two-Wheel)

## Physical Dimensions
- **Wheel Diameter**: 82 mm (radius: 41 mm)
- **Wheelbase**: 240 mm
- **Encoder Resolution**: 360 ticks per revolution

## Pin Assignments

### Left Motor (M1)
| Function | Pin | Description |
|----------|-----|-------------|
| INA | 7 | H-Bridge direction control A |
| INB | 8 | H-Bridge direction control B |
| PWM | 5 | PWM speed control |

### Right Motor (M2)
| Function | Pin | Description |
|----------|-----|-------------|
| INA | 4 | H-Bridge direction control A |
| INB | 9 | H-Bridge direction control B |
| PWM | 6 | PWM speed control |

### Left Encoder
| Function | Pin | Description |
|----------|-----|-------------|
| A (Phase A) | 2 | Quadrature encoder channel A (INT0) |
| B (Phase B) | 10 | Quadrature encoder channel B |

### Right Encoder
| Function | Pin | Description |
|----------|-----|-------------|
| A (Phase A) | 3 | Quadrature encoder channel A (INT1) |
| B (Phase B) | 11 | Quadrature encoder channel B |

## Motor-Encoder Mapping

```
LEFT  MOTOR (M1)  <->  LEFT  ENCODER (Pins 2, 10)
RIGHT MOTOR (M2)  <->  RIGHT ENCODER (Pins 3, 11)
```

## Direction Configuration

### Motor Inversion
- **Left Motor**: Not inverted
- **Right Motor**: Inverted (negated in software)

### Encoder Inversion
- **Left Encoder**: Not inverted (`INV_LEFT = false`)
- **Right Encoder**: Not inverted (`INV_RIGHT = false`)

## PID Velocity Control

### Overview
The firmware includes PID velocity controllers for both motors, allowing precise speed control in meters per second (m/s).

### Default PID Gains
- **Kp**: 10.0 (Proportional gain)
- **Ki**: 5.0 (Integral gain)
- **Kd**: 0.1 (Derivative gain)

### Control Modes
1. **Direct PWM Control**: Manual PWM control (-255 to +255)
   - Commands: `GO`, `L`, `R`
   - PID is disabled in this mode

2. **Velocity Control**: Automatic speed regulation using PID
   - Commands: `VG`, `VL`, `VR`
   - PID is enabled and maintains target velocity
   - Input in m/s (e.g., `VG 0.1` = 10 cm/s forward)

### PID Update Rate
- **Control Loop**: 50 ms (20 Hz)
- **Telemetry Output**: 200 ms (5 Hz)

### Serial Commands

#### Motor Control
| Command | Description | Example |
|---------|-------------|---------|
| `GO <pwm>` | Both motors at PWM value | `GO 100` |
| `L <pwm>` | Left motor at PWM value | `L 50` |
| `R <pwm>` | Right motor at PWM value | `R -50` |
| `VG <m/s>` | Both motors at velocity (PID) | `VG 0.15` |
| `VL <m/s>` | Left motor at velocity (PID) | `VL 0.1` |
| `VR <m/s>` | Right motor at velocity (PID) | `VR 0.2` |
| `STOP` | Stop all motors and disable PID | `STOP` |
| `ZERO` | Reset encoder counts | `ZERO` |

#### PID Tuning
| Command | Description | Example |
|---------|-------------|---------|
| `KP <value>` | Set proportional gain | `KP 15.0` |
| `KI <value>` | Set integral gain | `KI 8.0` |
| `KD <value>` | Set derivative gain | `KD 0.2` |
| `TUNEL` | Auto-tune left motor PID | `TUNEL` |
| `TUNER` | Auto-tune right motor PID | `TUNER` |
| `SAVE` | Save PID values to EEPROM | `SAVE` |
| `LOAD` | Load PID values from EEPROM | `LOAD` |

### Auto-Tuning Procedure

The firmware includes comprehensive automatic tuning that measures both motor friction characteristics and optimal PID gains.

**The auto-tuner performs 4 phases:**
1. **Deadband Detection (5-10s)**: Finds minimum PWM to start motor from standstill
2. **Running PWM Detection (3-4s)**: Finds minimum PWM to keep motor running
3. **Steady-State Measurement (3s)**: Measures motor velocity at test PWM
4. **Rise Time Measurement (3s)**: Measures system time constant and calculates PID gains

Total tuning time: ~15-20 seconds

**Deadband Compensation:**
DC motors have non-linear friction characteristics:
- **Static friction (stiction)**: Requires higher PWM to overcome initial resistance
- **Dynamic friction**: Lower PWM needed once moving

The auto-tuner automatically measures these values and compensates for them in the PID controller:
- When motor is **stationary**: PID output + Min Start PWM
- When motor is **running**: PID output + Min Running PWM

This eliminates the common problem of motors oscillating at low speeds or failing to start smoothly.

**How to use:**
1. Ensure robot wheels can spin freely (lift robot or place on low-friction surface)
2. Send `TUNEL` to tune left motor or `TUNER` to tune right motor
3. Wait ~15-20 seconds for complete tuning process
4. The tuner will print all measured values and auto-apply them
5. **Values are automatically saved to EEPROM** - no manual save needed!
6. Test with a velocity command like `VL 0.05` (very low speed) or `VG 0.15`

**Recommended operating speeds:** 0.05 to 0.3 m/s (5-30 cm/s)

### EEPROM Persistence

All PID parameters and deadband compensation values are automatically saved to EEPROM:
- **Auto-save**: After successful `TUNEL` or `TUNER` completion
- **Auto-load**: On every startup/reset
- **Manual save**: Use `SAVE` command after manual PID adjustments (e.g., after `KP 15.0`)
- **Manual load**: Use `LOAD` command to reload from EEPROM

**Stored values:**
- Left motor: Kp, Ki, Kd, Min Start PWM, Min Running PWM
- Right motor: Kp, Ki, Kd, Min Start PWM, Min Running PWM
- Data integrity: Magic number + checksum validation

**Workflow:**
```
1. First time setup:
   TUNEL          → Calibrate left motor
   TUNER          → Calibrate right motor
   (Values auto-saved to EEPROM)

2. After power cycle:
   (Values auto-loaded from EEPROM on startup)
   VG 0.2         → Works immediately with tuned values

3. Manual adjustment:
   KP 20.0        → Change Kp manually
   SAVE           → Save to EEPROM
```

### Telemetry Output Format
```
V <vel_left> <vel_right> T <target_left> <target_right> PWM <pwm_left> <pwm_right> P <p_left> <p_right> I <i_left> <i_right> D <d_left> <d_right> PID <ON/OFF>
```
Example:
```
V 0.1523 0.1498 T 0.150 0.150 PWM 125 -128 P 45.2 48.1 I 12.3 10.8 D 0.0 0.0 PID ON
```

**Field descriptions:**
- **V**: Current measured velocity (m/s) for left and right motors
- **T**: Target velocity (m/s) for left and right motors
- **PWM**: Actual PWM values sent to motors (-255 to +255)
- **P**: Proportional component of PID controller
- **I**: Integral component of PID controller
- **D**: Derivative component of PID controller (typically 0.0 with default tuning)
- **PID**: Controller status (ON when using velocity commands, OFF when using direct PWM)

**Auto-tune output example:**
```
TUNE START Motor=LEFT
TUNE Finding min start PWM...
TUNE Testing PWM=5
TUNE Testing PWM=10
...
TUNE Testing PWM=35
TUNE Min Start PWM found: 35
TUNE Finding min running PWM...
TUNE Testing running PWM=98
TUNE Testing running PWM=96
...
TUNE Testing running PWM=24
TUNE Min Running PWM found: 29
TUNE Measuring steady-state velocity...
TUNE STEADY VEL=0.2345 m/s
TUNE Measuring rise time...
TUNE Rise time (tau)=0.234 s
TUNE DONE
Deadband measurements:
  Min Start PWM=35.0
  Min Running PWM=29.0
System response:
  Process gain K=0.002345 m/s per PWM
  Time constant tau=0.234 s
Suggested PID gains:
  KP 42.500
  KI 4.550
  KD 1.250
All values auto-applied to selected motor.
```

## Technical Notes
- Encoder pins use `INPUT_PULLUP` mode
- Encoder A channels use hardware interrupts (INT0 and INT1)
- Encoder B channels are read in software for quadrature decoding
- Motor PWM range: -255 to +255 (positive = forward)
- Right motor is inverted in `setMotors()` to compensate for physical mounting direction
- PID includes anti-windup to prevent integral saturation
- Velocity calculation: `vel = (delta_ticks * meters_per_tick) / dt`
