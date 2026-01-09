# PID Tuning Theory and Implementation

Technical documentation covering the algorithms, mathematics, and design decisions behind the motor PID tuning system.

## Table of Contents

1. [System Overview](#system-overview)
2. [DC Motor Characteristics](#dc-motor-characteristics)
3. [Enhanced PID Controller](#enhanced-pid-controller)
4. [Deadband Compensation](#deadband-compensation)
5. [Anti-Windup Strategies](#anti-windup-strategies)
6. [Velocity Filtering](#velocity-filtering)
7. [System Identification](#system-identification)
8. [Ziegler-Nichols Tuning](#ziegler-nichols-tuning)
9. [Implementation Details](#implementation-details)

---

## System Overview

### Control Architecture

```
Target Velocity (v_target)
         |
         v
    [Velocity Ramp] ← Acceleration limiting
         |
         v
    [PID Controller] ← PI + Feed-forward
         |
         v
    [Deadband Comp] ← Friction compensation
         |
         v
       PWM → Motor → Encoder
         |____________↑
         (velocity feedback)
```

### Control Loop Specifications

- **Update Rate**: 50 ms (20 Hz)
- **Encoder Resolution**: 714 ticks/rev (2X decoding)
- **Velocity Range**: 0 - 0.7 m/s
- **PWM Range**: -255 to +255 (8-bit)

---

## DC Motor Characteristics

### Transfer Function

A DC motor with negligible inductance can be modeled as a first-order system:

```
       K
G(s) = ―――
       τs + 1
```

Where:
- **K**: Process gain (velocity per unit PWM)
- **τ (tau)**: Time constant (seconds)

### Static vs Dynamic Friction

DC motors exhibit two types of friction:

1. **Static Friction (Stiction)**
   - Resistance when motor is at rest
   - Requires higher PWM to overcome
   - Measured as "Min Start PWM"

2. **Dynamic Friction**
   - Resistance when motor is moving
   - Lower than static friction
   - Measured as "Min Running PWM"

**Mathematical Model**:
```
         ⎧ PWM_min_start    if |ω| = 0 and PWM ≠ 0
PWM_adj = ⎨ PWM_min_running  if |ω| > 0
         ⎩ 0                if PWM = 0
```

Where ω is angular velocity.

### Encoder Velocity Calculation

Velocity is calculated from encoder ticks:

```
       Δticks × meters_per_tick
v(t) = ――――――――――――――――――――――
              Δt
```

Where:
- `meters_per_tick = (π × D) / TICKS_PER_REV`
- `D = 0.0825 m` (wheel diameter)
- `TICKS_PER_REV = 714` (2X decoding)
- `Δt = 0.050 s` (control interval)

---

## Enhanced PID Controller

### Standard PID Equation

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt
```

Where:
- `u(t)`: Control output (PWM)
- `e(t) = v_target - v_measured`: Error
- `Kp, Ki, Kd`: PID gains

### Our Implementation: PI + Feed-Forward

We use a **PI controller with feed-forward** instead of full PID:

```
u(t) = Kff_v·v_target + Kff_a·a_target + Kp·e(t) + Ki·∫e(t)dt
```

**Why no D-term?**
- Encoder noise causes derivative kick
- DC motors are first-order systems (naturally low-pass)
- Properly tuned PI + feed-forward is sufficient

**Feed-forward terms**:
- **Kff_v**: Velocity feed-forward
  - Compensates for steady-state PWM requirement
  - Reduces steady-state error
  - Calculated as: `Kff_v = 1 / K` (inverse of process gain)

- **Kff_a**: Acceleration feed-forward (future enhancement)
  - Compensates for inertia during acceleration
  - Improves transient response
  - Calculated from motor inertia and friction

### Discrete-Time Implementation

In the microcontroller, we use discrete-time (sampled) control:

```cpp
// Time step
float dt = 0.050;  // 50 ms

// Error
float error = v_target - v_filtered;

// Proportional term
float P = Kp * error;

// Integral term (accumulate)
integral += error * dt;
float I = Ki * integral;

// Feed-forward
float accel = (v_target - v_target_prev) / dt;
float FF = Kff_v * v_target + Kff_a * accel;

// Output
float pwm = FF + P + I;
```

---

## Deadband Compensation

### Problem

Without compensation, the PID output must reach the friction threshold before the motor moves. This causes:
- Dead zone at low velocities
- Poor low-speed control
- Limit cycles (oscillation)

### Solution

Add constant offset based on motor state:

```cpp
if (v_target > 0.001f) {
    // Moving forward
    if (abs(v_measured) < 0.001f) {
        pwm += deadband_start_fwd;  // Motor at rest
    } else {
        pwm += deadband_running_fwd;  // Motor already moving
    }
}
else if (v_target < -0.001f) {
    // Moving backward
    if (abs(v_measured) < 0.001f) {
        pwm -= deadband_start_rev;
    } else {
        pwm -= deadband_running_rev;
    }
}
```

### Benefits

- Linear response even at low velocities
- Eliminates dead zone
- Improves settling time
- Reduces steady-state error

---

## Anti-Windup Strategies

### The Windup Problem

When the motor saturates (PWM limited to ±255), but the error persists, the integral term continues to accumulate. This causes:
- Overshoot when error changes sign
- Sluggish response
- Instability

### Simple Clamping (Not Used)

```cpp
integral = constrain(integral, -max_integral, max_integral);
```

**Problems**:
- Arbitrary limit
- Doesn't respond to actual saturation
- Still allows some windup

### Back-Calculation Anti-Windup (Our Method)

When the output saturates, feed back the excess to reduce the integral:

```cpp
// Calculate raw output
float pwm_raw = FF + P + I;

// Apply limits
float pwm = constrain(pwm_raw, -255.0f, 255.0f);

// Back-calculation
if (pwm != pwm_raw) {
    float excess = pwm_raw - pwm;
    integral -= excess / Ki * 0.5f;  // Factor 0.5 for stability
}
```

**How it works**:
1. Calculate what the output *would* be without limits
2. Apply limits to get actual output
3. If they differ, reduce integral proportionally
4. The factor 0.5 prevents over-correction

**Benefits**:
- Responds to actual saturation
- Self-adjusting
- Maintains good transient response
- Industry-standard approach

---

## Velocity Filtering

### Problem

Raw encoder-based velocity is noisy due to:
- Quantization (discrete ticks)
- Finite time intervals
- Mechanical vibration
- Electrical noise

### Low-Pass Filter

We use an exponential moving average (first-order IIR filter):

```cpp
v_filtered = α·v_measured + (1-α)·v_filtered_prev
```

Where:
- **α (alpha)**: Filter coefficient (0 to 1)
- α = 1: No filtering (use raw measurement)
- α = 0: Infinite filtering (no update)
- Typical: α = 0.3

**Transfer function**:
```
         α
H(z) = ―――――
       1-(1-α)z⁻¹
```

**Cutoff frequency**:
```
           α
fc = ――――――――――― × fs
      2π(1-α)
```

For α = 0.3, fs = 20 Hz: fc ≈ 1 Hz

### Trade-off

- **More filtering (lower α)**:
  - Smoother velocity estimate
  - Less noise in derivative
  - Slower response to changes
  - Phase lag in control

- **Less filtering (higher α)**:
  - Faster response
  - More noise
  - Potential derivative kick

**Recommended**: α = 0.3 for good balance

---

## System Identification

System identification determines the motor's dynamic characteristics.

### Step Response Method

1. Apply a step input (constant velocity target)
2. Record the velocity response
3. Extract system parameters

### Measured Parameters

#### Process Gain (K)

```
       v_steady
K = ―――――――――
      PWM_avg
```

Units: m/s per PWM

#### Time Constant (τ)

Time to reach 63.2% of steady-state value.

```
τ = t₆₃.₂%
```

For a first-order system, this is the time constant.

#### Steady-State Error

```
e_ss = |v_target - v_steady|
```

Should be < 5% with properly tuned controller.

### Data Collection

The step response test:
1. Starts from rest
2. Applies target velocity
3. Records for 5 seconds at 50 Hz
4. Analyzes last 25% of data for steady-state

---

## Ziegler-Nichols Tuning

### Method Selection

We use the **Ziegler-Nichols First-Order + Delay** method, which is suitable for our system model.

### Tuning Formulas

For a first-order system with time constant τ and process gain K:

**PI Controller**:
```
Kp = 0.9 / (K × τ)
Ki = Kp / (3 × τ)
```

**Conservative Tuning** (what we use):
```
Kp = 0.6 / (K × τ)
Ki = Kp / (4 × τ)
```

The conservative gains provide:
- Better stability margin
- Less overshoot
- Slower but smoother response
- Suitable for mechanical systems

### Gain Adjustment

After auto-tuning, you can adjust:

**Faster response** (more aggressive):
```
Kp_new = Kp × 1.5
Ki_new = Ki × 1.5
```

**Slower response** (more stable):
```
Kp_new = Kp × 0.7
Ki_new = Ki × 0.7
```

**Important**: Keep the ratio Ki/Kp constant to maintain stability!

---

## Implementation Details

### Control Loop Timing

```cpp
void loop() {
    unsigned long now = millis();

    if (now - lastControlTime >= CONTROL_INTERVAL) {
        float dt = (now - lastControlTime) / 1000.0f;

        // Update control
        updateVelocityControl(dt);

        lastControlTime = now;
    }
}
```

**Critical**: Use actual measured `dt`, not assumed value, for accuracy.

### Velocity Ramping

Limits acceleration to prevent wheel slip:

```cpp
float ramp_target = current_velocity;
float delta = target_velocity - current_velocity;
float max_change = max_accel * dt;

if (delta > max_change) {
    delta = max_change;
} else if (delta < -max_change) {
    delta = -max_change;
}

ramp_target = current_velocity + delta;
```

Typical: `max_accel = 2.0 m/s²`

### Encoder Counting

Using 2X quadrature decoding with XOR logic:

```cpp
void isrEncoderLeft() {
    bool a = digitalRead(PIN_ENC_L_A);
    bool b = digitalRead(PIN_ENC_L_B);

    // XOR logic: (A==B) means forward
    if (a == b) {
        encoderLeftCount++;
    } else {
        encoderLeftCount--;
    }
}
```

Interrupt attached to `CHANGE` mode (both edges) for 2X resolution.

### EEPROM Storage

Parameters saved with integrity check:

```cpp
struct PIDParams {
    uint32_t magic = 0xDEADBEEF;
    float Kp, Ki, Kd;
    float deadband_fwd, deadband_rev;
    float Kff_v, Kff_a;
    float filter_alpha;
    uint32_t checksum;
};
```

Checksum validates data integrity on load.

---

## Performance Metrics

### Rise Time

Time to reach 90% of target velocity.

**Target**: < 2 seconds

### Settling Time

Time to stay within 5% of target.

**Target**: < 3 seconds

### Steady-State Error

Error after system has settled.

**Target**: < 2% (e.g., 0.004 m/s at 0.2 m/s target)

### Overshoot

Maximum overshoot beyond target.

**Target**: < 10%

### Recommended Operating Range

- **Velocity**: 0.05 - 0.7 m/s
- **Acceleration**: 0.5 - 3.0 m/s²
- **PWM**: 40 - 240 (deadband compensated)

---

## Troubleshooting Theory

### Oscillation

**Symptoms**: Velocity oscillates around target

**Causes**:
1. Kp too high (proportional kick)
2. Ki too high (integral windup)
3. Insufficient filtering (noisy derivative)
4. Mechanical resonance

**Solutions**:
- Reduce Kp by 30-50%
- Reduce Ki proportionally
- Increase filtering (reduce α)
- Check for loose mechanical connections

### Sluggish Response

**Symptoms**: Slow to reach target, but stable

**Causes**:
1. Kp too low
2. Excessive filtering
3. Insufficient feed-forward

**Solutions**:
- Increase Kp by 20-30%
- Increase Ki proportionally
- Reduce filtering (increase α)
- Increase Kff_v

### Steady-State Error

**Symptoms**: Never quite reaches target velocity

**Causes**:
1. Ki too low (insufficient integral action)
2. Model mismatch
3. Incorrect deadband

**Solutions**:
- Increase Ki
- Re-run system identification
- Re-tune deadband

### Integral Windup

**Symptoms**: Large overshoot after step change

**Causes**:
- Anti-windup not working
- Limits too tight

**Solutions**:
- Verify back-calculation is active
- Check for saturation
- Reduce Ki if persistent

---

## References

### Books

1. Åström, K. J., & Murray, R. M. (2008). *Feedback Systems: An Introduction for Scientists and Engineers*. Princeton University Press.

2. Franklin, G. F., Powell, J. D., & Emami-Naeini, A. (2014). *Feedback Control of Dynamic Systems* (7th ed.). Pearson.

3. Visioli, A. (2006). *Practical PID Control*. Springer.

### Papers

1. Ziegler, J. G., & Nichols, N. B. (1942). "Optimum Settings for Automatic Controllers". *Transactions of the ASME*, 64, 759-768.

2. Åström, K. J., & Hägglund, T. (1984). "Automatic Tuning of Simple Regulators with Specifications on Phase and Amplitude Margins". *Automatica*, 20(5), 645-651.

### Online Resources

- [PID Controller - Wikipedia](https://en.wikipedia.org/wiki/PID_controller)
- [Control Tutorials for MATLAB](http://ctms.engin.umich.edu/)
- [Ziegler-Nichols Method Explained](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method)

---

## Appendix: Mathematical Derivations

### A1: First-Order System Response

For a first-order system with step input:

```
G(s) = K/(τs + 1)
Input: U(s) = A/s (step of magnitude A)
Output: Y(s) = G(s)·U(s) = KA/(s(τs + 1))
```

Inverse Laplace transform:

```
y(t) = KA(1 - e^(-t/τ))
```

At t = τ:
```
y(τ) = KA(1 - e^(-1)) ≈ 0.632·KA
```

Hence, τ is the time to reach 63.2% of final value.

### A2: Discrete-Time Integration

Rectangular integration (forward Euler):

```
I[k] = I[k-1] + e[k]·Δt
```

Trapezoidal integration (more accurate):

```
I[k] = I[k-1] + (e[k] + e[k-1])·Δt/2
```

We use rectangular for simplicity; trapezoidal would improve accuracy slightly.

### A3: Exponential Moving Average

Recursive form:
```
y[k] = α·x[k] + (1-α)·y[k-1]
```

Equivalent to IIR filter:
```
H(z) = α/(1 - (1-α)z⁻¹)
```

Time constant relation:
```
τ_filter = -Δt / ln(1-α)
```

For α = 0.3, Δt = 0.05s:
```
τ_filter ≈ 0.14 s
```

---

## Version History

- **v1.0** (2026-01-08): Initial implementation with PI + feed-forward, back-calculation anti-windup, velocity filtering, and Ziegler-Nichols tuning
