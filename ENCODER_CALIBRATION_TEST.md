# Encoder Calibration Test

## Problem
Odometry shows robot traveled ~0.5m but physically returned to origin. Either:
- Missing encoder counts (interrupts on RISING only, should be CHANGE)
- Wrong TICKS_PER_REV value (currently 310)
- Spurious/bounced counts

## Current Settings
- Arduino: `TICKS_PER_REV = 310.0f`
- Server: `ticks_per_revolution: 310`
- Interrupt mode: **RISING only** (line 681-682 in main.cpp)
- Decoding: 1X (single edge, uses B channel for direction)

## Test 1: Measure Actual Ticks Per Revolution

### Procedure:
1. Zero encoders using UI button
2. Mark a spot on one wheel (e.g., with tape)
3. Manually rotate wheel EXACTLY 1 full revolution (mark returns to same position)
4. Record encoder count from UI

### Expected Results:
- If RISING edge only + 310 PPR encoder → Should see ~155 counts
- If CHANGE mode + 310 PPR encoder → Should see ~310 counts
- If encoder is 310 CPR (counts per rev with quadrature) → Need 4X decoding

### Fix Options:
**Option A: Change to CHANGE interrupt (2X decoding)**
```cpp
// Line 681-682, change from RISING to CHANGE
attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncoderLeft, CHANGE);
attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncoderRight, CHANGE);
```

**Option B: Update TICKS_PER_REV based on measurement**
```cpp
// If you measure N ticks per revolution:
const float TICKS_PER_REV = N;  // Update in main.cpp line 37
```

## Test 2: Measure Linear Distance Accuracy

### Procedure:
1. Zero encoders
2. Mark robot position on floor
3. Drive robot forward 1 meter (measure with tape measure)
4. Compare:
   - Actual distance: 1.000 m
   - Encoder counts: (left + right) / 2
   - Calculated: counts * METERS_PER_TICK

### Formula:
```
METERS_PER_TICK = PI * WHEEL_DIAMETER / TICKS_PER_REV
                = 3.14159 * 0.081 / 310
                = 0.000821 m/tick
```

For 1 meter travel:
- Expected counts = 1.0 / 0.000821 = ~1218 ticks per wheel

### Correction:
```
Actual_TICKS_PER_REV = (Measured_counts / 1.0) * (PI * 0.081)
```

## Quick Test via UI:
1. Open http://localhost:5001
2. Click "Zero Encoders & Reset"
3. Rotate left wheel 1 full revolution by hand
4. Check "Encoder Left" count in Odometry panel
5. Record the value here: ___________

## Recommended Fix Order:
1. ✓ First do quick 1-revolution test (easiest)
2. Based on result, decide RISING vs CHANGE
3. If needed, do 1-meter linear test for fine calibration
4. Update firmware and test closed-loop return to origin
