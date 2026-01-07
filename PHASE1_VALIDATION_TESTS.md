# Phase 1 Validation Tests - Execution Guide

**Story**: 1.5 - Run Validation Tests and Document Results
**Date Created**: 2026-01-07
**Status**: Ready to Execute

## Prerequisites

✅ Story 1.1 Complete: ESP32 odometry at 10 Hz (host-controlled)
✅ Story 1.2 Complete: LIDAR target RPM set to 240
✅ Story 1.3 Complete: Arduino wheelbase verified (240mm)
✅ Story 1.4 Complete: Validation script created

## Test Environment Setup

1. **Power on robot**
   - Ensure battery fully charged
   - ESP32 boots and connects to WiFi (192.168.68.52)

2. **Verify connections**
   - Arduino ↔ ESP32 serial @ 9600 baud
   - ESP32 ↔ LIDAR serial @ 115200 baud
   - ESP32 WiFi connected

3. **Open terminal in robot-ui/server directory**
   ```bash
   cd robot-ui/server
   ```

## Test Scenarios

### Test 1: Stationary Test (60 seconds)

**Purpose**: Measure sensor noise when robot is completely still

**Setup**:
- Place robot on flat surface
- DO NOT TOUCH robot during test
- Ensure robot is powered on and connected

**Execute**:
```bash
python3 validate_sensor_data.py --host 192.168.68.52 --duration 60 --stillness-test --csv test1_stationary.csv
```

**Expected Results**:
- ✓ Odometry rate: 10 Hz ±0.5 Hz
- ✓ LIDAR rate: 4 Hz ±0.2 Hz
- ✓ Encoder drift: <5 ticks over 10 seconds
- ✓ No data loss warnings

**Record**:
- [ ] Test completed
- [ ] Results saved to CSV
- [ ] Odometry rate: _____ Hz
- [ ] LIDAR rate: _____ Hz
- [ ] Left encoder drift: _____ ticks
- [ ] Right encoder drift: _____ ticks
- [ ] Noise assessment: _____

---

### Test 2: Straight Line Test (60 seconds)

**Purpose**: Verify encoder counts per sample at target speed (0.15 m/s)

**Setup**:
- Clear space with at least 10 meters straight path
- Robot on flat surface

**Execute**:
```bash
# Terminal 1: Start validation script
python3 validate_sensor_data.py --host 192.168.68.52 --duration 60 --csv test2_straight.csv

# Terminal 2: Start Flask server and control robot
python3 robot_ui_server.py
# Open browser: http://localhost:5001
# Set velocity: v=0.15 m/s, w=0 rad/s
# Drive for 60 seconds
```

**Expected Results**:
- ✓ Average speed: 0.15 m/s ±0.02 m/s
- ✓ Encoder counts/sample: 20-25 ticks (optimal for 360 ticks/rev)
- ✓ Consistent left/right encoder readings
- ✓ LIDAR scans stable at 240 RPM

**Record**:
- [ ] Test completed
- [ ] Results saved to CSV
- [ ] Average speed: _____ m/s
- [ ] Left ticks/sample: _____ ±_____
- [ ] Right ticks/sample: _____ ±_____
- [ ] LIDAR RPM: _____ ±_____

---

### Test 3: Rotation Test (60 seconds)

**Purpose**: Verify rotation odometry at 0.3 rad/s

**Setup**:
- Robot on flat surface with space to rotate
- Mark initial orientation

**Execute**:
```bash
# Terminal 1: Start validation script
python3 validate_sensor_data.py --host 192.168.68.52 --duration 60 --csv test3_rotation.csv

# Terminal 2: Start Flask server and control robot
python3 robot_ui_server.py
# Open browser: http://localhost:5001
# Set velocity: v=0 m/s, w=0.3 rad/s
# Rotate for 60 seconds
```

**Expected Results**:
- ✓ Rotation rate: ~0.3 rad/s
- ✓ Left/right encoders show opposite signs (counter-rotation)
- ✓ LIDAR continues scanning at 240 RPM during rotation
- ✓ No encoder slippage

**Record**:
- [ ] Test completed
- [ ] Results saved to CSV
- [ ] Rotations completed: _____
- [ ] Left encoder total: _____
- [ ] Right encoder total: _____
- [ ] Expected ratio (L/R ≈ -1): _____

---

### Test 4: Mixed Motion Test (120 seconds)

**Purpose**: Verify system under realistic usage conditions

**Setup**:
- Open space for maneuvering
- Mix of forward, backward, rotation, curves

**Execute**:
```bash
# Terminal 1: Start validation script
python3 validate_sensor_data.py --host 192.168.68.52 --duration 120 --csv test4_mixed.csv

# Terminal 2: Start Flask server and manually control robot
python3 robot_ui_server.py
# Open browser: http://localhost:5001
# Manual control for 120 seconds:
#   - Forward motion
#   - Backward motion
#   - Left/right turns
#   - Figure-8 pattern
```

**Expected Results**:
- ✓ No data loss during complex maneuvers
- ✓ Odometry rate stable at 10 Hz
- ✓ LIDAR rate stable at 4 Hz
- ✓ No timestamp discontinuities
- ✓ No error messages in ESP32 or Flask logs

**Record**:
- [ ] Test completed
- [ ] Results saved to CSV
- [ ] Maneuvers performed: _____
- [ ] Data loss events: _____
- [ ] Timing issues: _____
- [ ] Notes: _____

---

## Data Analysis

After completing all tests, analyze the CSV files:

```bash
# Generate summary statistics
python3 -c "
import pandas as pd
import glob

for csv_file in glob.glob('test*.csv'):
    print(f'\n=== {csv_file} ===')
    df = pd.read_csv(csv_file)
    # Add analysis code here
"
```

## Acceptance Criteria Checklist

Phase 1 validation passes if ALL criteria are met:

- [ ] **Test 1 (Stationary)**: Encoder noise <5 ticks
- [ ] **Test 2 (Straight)**: Speed 0.15 m/s ±0.02, ticks/sample 20-25
- [ ] **Test 3 (Rotation)**: Smooth rotation, no encoder slip
- [ ] **Test 4 (Mixed)**: No data loss, stable frequencies
- [ ] **Odometry rate**: 10 Hz ±0.5 Hz in ALL tests
- [ ] **LIDAR rate**: 4 Hz ±0.2 Hz in ALL tests
- [ ] **No errors**: No warnings or errors in any test

## Go/No-Go Decision

**GO** to Phase 2 if:
- All 4 tests pass acceptance criteria
- Sensor data quality is sufficient for SLAM
- No critical issues identified

**NO-GO** (address issues first) if:
- Odometry frequency out of spec
- High encoder noise (>5 ticks stationary)
- Data loss during motion
- LIDAR unstable

## Results Summary

**Tested by**: _____
**Test date**: _____
**Overall result**: [ ] PASS / [ ] FAIL

**Key findings**:
-
-
-

**Issues identified**:
1.
2.
3.

**Decision**: [ ] GO to Phase 2 / [ ] Address issues first

**Next steps**:
-
-
