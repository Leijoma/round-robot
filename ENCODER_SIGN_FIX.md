# Encoder Sign Mismatch - Critical Fix Required

## Problem Found
After driving forward and backward, your encoders show:
- **Left encoder: +1794**
- **Right encoder: -2214**

This is **completely wrong**! Both encoders should have the **same sign** for straight-line motion.

## Root Cause
One encoder has the wrong polarity setting. The `reverse` flag is incorrect on one of them.

## Current Configuration (main.cpp lines 49-50)
```cpp
Encoder encoderLeft(PIN_ENC_L_A, PIN_ENC_L_B, true);    // Reversed
Encoder encoderRight(PIN_ENC_R_A, PIN_ENC_R_B, true);   // Reversed
```

Both are set to `reverse = true`, but clearly one is still counting backwards.

## Why This Happens

Your differential drive robot has two scenarios:

### Scenario A: Motors mounted same orientation
```
LEFT MOTOR    RIGHT MOTOR
[→ Forward]   [→ Forward]
```
Both encoders count UP when driving forward → Both should have `reverse = false` (or both `true`)

### Scenario B: Motors mounted opposite (most common)
```
LEFT MOTOR    RIGHT MOTOR
[→ Forward]   [← Forward]
```
One motor physically points backward → Encoders naturally have opposite polarity

**BUT** you're seeing opposite signs even with both set to `reverse = true`, which means:
- **One encoder's A/B channels are physically swapped**
- OR one encoder reverse flag is not taking effect
- OR the comment "// Reversed" is wrong for one side

## Quick Test to Identify Which Encoder is Wrong

1. **Place robot on blocks** (wheels off ground)
2. **Spin LEFT wheel forward** by hand
3. **Watch encoder count in UI**
   - Should count **positive** (forward motion)
   - If negative → Left encoder reverse flag is wrong
4. **Spin RIGHT wheel forward** by hand
5. **Watch encoder count in UI**
   - Should count **positive** (forward motion)
   - If negative → Right encoder reverse flag is wrong

## The Fix

Based on your test results, change **ONE** of these lines in main.cpp:

### If LEFT encoder counts negative when wheel spins forward:
```cpp
Encoder encoderLeft(PIN_ENC_L_A, PIN_ENC_L_B, false);   // Changed to false
Encoder encoderRight(PIN_ENC_R_A, PIN_ENC_R_B, true);   // Keep true
```

### If RIGHT encoder counts negative when wheel spins forward:
```cpp
Encoder encoderLeft(PIN_ENC_L_A, PIN_ENC_L_B, true);    // Keep true
Encoder encoderRight(PIN_ENC_R_A, PIN_ENC_R_B, false);  // Changed to false
```

## Alternative Hardware Fix

If you can't get the reverse flags to work, **physically swap the A and B wires** on one encoder:
- This reverses the counting direction in hardware
- More reliable than software reverse flag

## Why This is Critical

With opposite encoder polarities:
- Robot thinks it's **spinning in place** when driving straight
- Odometry calculates **completely wrong position**
- ICP scan matching will **fail to converge**
- Your 10% asymmetry problem is **impossible to diagnose** with this larger error

**Fix this FIRST**, then we can address the pull-up asymmetry issue.

## Expected Result After Fix

After driving forward then backward (returning to start):
- **Left encoder: ~1794**
- **Right encoder: ~1794** (same sign, similar magnitude)

Small differences are OK, but signs must match!
