# Encoder Pull-up Asymmetry Diagnosis

## Problem
You're seeing 10% tick count difference between forward and reverse motion. This suggests the encoder decoding is direction-dependent.

## Current Configuration
- **Pull-ups**: Internal (~20-50kΩ on Arduino)
- **Decoding**: 2X quadrature (XOR method)
- **Interrupts**: CHANGE on channel A only
- **Sampling**: Channel B read via `digitalRead()` after interrupt

## Why This Causes Asymmetry

### 1. Slow Edge Transitions
Internal pull-ups create slow rise times:
- **Fall time**: ~100ns (encoder output actively pulls low)
- **Rise time**: ~10-50µs (weak pull-up charges line capacitance)

### 2. Direction-Dependent Phase Relationship
**Forward rotation (A leads B by 90°):**
```
A: ─┐   ┌─    Interrupt fires here ↓
B:   ─┐   ┌─   B is read ~3µs later (may still be transitioning!)
```

**Reverse rotation (B leads A by 90°):**
```
A: ─┐   ┌─    Interrupt fires here ↓
B: ┌─    ─┐   B is read ~3µs later (stable)
```

### 3. Metastable Reads
When `digitalRead(_pinB)` happens during a transition:
- Random HIGH/LOW results
- Different probability depending on edge direction
- Results in tick count errors that accumulate differently in each direction

## Quick Hardware Test

**You need an oscilloscope to verify this:**

1. **Probe encoder outputs** while motor is running
2. **Measure rise/fall times** on both channels
3. **Expected if pull-ups are the problem:**
   - Fall time: <1µs (sharp)
   - Rise time: >10µs (slow, RC curve)
   - If rise time >> fall time, pull-ups are too weak

## Solutions (in order of effectiveness)

### Solution 1: Add External Pull-up Resistors (RECOMMENDED)
**Hardware change - most reliable fix**

Add external pull-up resistors to encoder channels:
- **Value**: 1kΩ - 4.7kΩ (much stronger than internal ~30kΩ)
- **Placement**: Between encoder output and +5V
- **Effect**: 10-20x faster rise times = symmetric edges

**Implementation:**
```
Encoder Channel A ───┬─── Arduino Pin (disable internal pull-up)
                     │
                    [4.7kΩ]
                     │
                    +5V

Encoder Channel B ───┬─── Arduino Pin (disable internal pull-up)
                     │
                    [4.7kΩ]
                     │
                    +5V
```

**Code change needed:**
```cpp
// In Encoder.cpp::begin()
pinMode(_pinA, INPUT);  // No internal pull-up
pinMode(_pinB, INPUT);  // No internal pull-up
```

### Solution 2: Add Input Schmitt Trigger
**Hardware change - best for noisy environments**

Use a 74HC14 or 74HCT14 Schmitt trigger buffer between encoders and Arduino:
- Converts slow edges to fast edges
- Built-in hysteresis eliminates bounce
- Very clean signals to Arduino

### Solution 3: Software Debouncing/Filtering
**Software change - less effective but no hardware needed**

Add a small delay after interrupt to let signal settle:
```cpp
void Encoder::handleInterrupt() {
    delayMicroseconds(10);  // Let channel B settle
    bool a = digitalRead(_pinA);
    bool b = digitalRead(_pinB);
    // ... rest of code
}
```

**Downside**: May miss counts at high speed

### Solution 4: Upgrade to 4X Decoding
**Software change - more robust algorithm**

Interrupt on both A and B channels (CHANGE mode):
- More interrupts = more samples = less sensitivity to edge timing
- Can validate direction using state machine
- Requires more CPU but Arduino should handle it

### Solution 5: Use Hardware Timer/Counter (BEST)
**Hardware change - most accurate**

Arduino Mega has hardware quadrature decoder on some pins:
- Zero software overhead
- No missed counts
- Immune to edge timing issues

## Immediate Test

**Test if pull-ups are the issue WITHOUT hardware changes:**

1. Run the forward/reverse test I created: `python3 test_encoder_direction.py`
2. Note the exact asymmetry percentage
3. Try running at HALF speed (slower encoder frequency = more time for edges to settle)
4. Run test again

**If asymmetry DECREASES at lower speed** → Pull-up issue confirmed!

## Recommended Action

1. **First**: Run the diagnostic test to quantify the problem
2. **Then**: Add 4.7kΩ external pull-ups (easiest, most effective)
3. **Test**: Verify asymmetry is reduced to <1%

## Why 10% Specifically?

A 10% error suggests about 1 in 10 edge transitions are being misread. This is consistent with:
- Slow rise time (~10-20µs)
- Interrupt-to-read delay (~3-5µs)
- Reading channel B while it's mid-transition
- Direction-dependent phase means this happens more in one direction

The asymmetry compounds over many encoder cycles, resulting in the 10% cumulative error you're seeing.
