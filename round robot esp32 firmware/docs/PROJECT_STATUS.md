# Round Robot Project - Current Status

**Last Updated**: 2026-01-09 01:45

---

## 📊 Current Phase

**Phase 1**: Sensor Optimization & Motor Synchronization
**Active Story**: Ready to upload odometry fixes and test

---

## ✅ Recently Completed

### 2026-01-09: CRITICAL - Odometry Precision Bug Fixes
- ✅ **ROOT CAUSE FOUND**: Float-to-int accumulation errors causing 21% odometry error
- ✅ **Arduino Firmware** - Fixed pose accumulation precision (main.cpp:82-84, 201-203)
  - Changed pose_x_mm, pose_y_mm, pose_th_mrad from int32_t to float
  - Removed int casting in updateOdometry() preventing 0.3-0.7mm loss per iteration
  - Only cast to int when transmitting over protocol
- ✅ **Python Host** - Fixed wrong robot parameters in SLAM dead reckoning
  - motion_model.py had wheel_diameter=80mm, ticks=360 (2x wrong!)
  - Fixed to wheel_diameter=79mm, ticks=714 (calibrated values)
  - Host was calculating 2x distance per encoder tick vs Arduino
- ✅ **Protocol Enhancements**:
  - Added MSG_SET_ROBOT_PARAMS (0x24) for configurable geometry
  - Added MSG_ACK/NACK (0x1F/0x7D) for command acknowledgment
  - Added MSG_STATUS_EXTENDED (0x1E) for per-motor PID reporting
  - Added heading hold controller with configurable gain
- ✅ **EEPROM Updates**:
  - Bumped EEPROM version to v3 (adds headingHoldKp, robot geometry)
  - Added automatic migration from v2 to v3
  - Robot geometry now loaded from EEPROM and configurable via UI
- ✅ **Expected Improvement**: Odometry error should drop from 21% to 1-2%
- ✅ Committed and pushed: b6f7954
- ⏳ **NEXT**: Upload firmware to Arduino and test odometry accuracy

### 2026-01-08: Per-Motor PID Configuration & Testing (Stories 1.7-1.8)
- ✅ Implemented per-motor PID parameters with EEPROM persistence
- ✅ Updated EEPROM structure to version 2 (separate left/right PID values)
- ✅ Added MSG_SET_PID_PER_MOTOR (0x1C) protocol message
- ✅ Updated firmware saveConfig/loadConfig for per-motor storage
- ✅ Applied tuned parameters:
  - Left motor: Kp=50, Ki=60, Kd=0, Deadband=50 PWM
  - Right motor: Kp=50, Ki=20, Kd=0, Deadband=35 PWM
- ✅ Added UI controls for per-motor PID tuning
- ✅ **Firmware uploaded and tested - robot control working!**
- ✅ Memory usage: 49.0% Flash, 26.2% RAM
- ✅ Committed: 412bebb, ec73f53

### 2026-01-07: PID Library Integration
- ✅ Integrated modular PID velocity control libraries into main firmware
- ✅ Created Encoder, MotorControl, PIDController libraries
- ✅ Achieved 0.7% velocity matching at 0.15 m/s
- ✅ Committed: fd4a80e "Integrate tuned PID velocity control libraries"

### 2026-01-06-07: Motor Calibration (Story 1.6)
- ✅ Created motor_calibration.py script
- ✅ Identified per-motor friction characteristics
- ✅ Determined optimal deadband values
- ✅ Documented calibration methodology

---

## 🔄 In Progress

### Story 1.8: Test Motor Synchronization
**Status**: Testing in progress
**Location**: Parallel Claude Code window + manual testing
**Next Steps**:
- Verify synchronized motor start (<50ms delta)
- Test straight-line movement accuracy
- Validate per-motor PID effectiveness
- Document test results

### UI Server Development
**Status**: Active development in parallel window
**Purpose**: Enhanced controls for per-motor PID tuning
**Waiting**: New UI version ready for testing

---

## 📅 Next Up

### Sensor Optimization (Stories 1.1-1.5)
Once motor synchronization testing is complete:

1. **Story 1.1**: Update ESP32 Odometry Frequency (5 Hz → 10 Hz)
   - File: `round robot esp32 firmware/src/main.cpp:104`
   - Change: `intervalMs = 200` → `100`

2. **Story 1.2**: Update LIDAR Target RPM (220 → 240)
   - File: `round robot esp32 firmware/src/main.cpp:22`
   - Change: `LIDAR_TARGET_RPM 220` → `240`

3. **Story 1.3**: Verify Arduino Wheelbase Configuration
   - File: `Arduino Motor control/src/main.cpp:39`
   - Confirm: `WHEELBASE = 0.244f` (244mm measured)

4. **Story 1.4**: Create Sensor Data Validation Script
   - New file: `robot-ui/server/validate_sensor_data.py`
   - Purpose: Log and analyze sensor data at new frequencies

5. **Story 1.5**: Run Validation Tests
   - Test scenarios: Stationary, straight line, rotation, mixed
   - Document results in SLAM_BACKLOG.md

---

## 🎯 Phase 1 Progress

### Completed Stories
- [x] Story 1.6: Motor Calibration Script
- [x] Story 1.7: Implement Dual-Rate PID & Motor Sync

### Active Stories
- [ ] Story 1.8: Test Motor Synchronization (testing)

### Pending Stories
- [ ] Story 1.1: Update ESP32 Odometry Frequency
- [ ] Story 1.2: Update LIDAR Target RPM
- [ ] Story 1.3: Verify Arduino Wheelbase
- [ ] Story 1.4: Create Sensor Validation Script
- [ ] Story 1.5: Run Validation Tests

**Phase 1 Completion**: ~40% (2/5 major story groups done)

---

## 📈 Key Metrics

### Motor Control Performance
- Velocity matching: **0.7%** at 0.15 m/s
- Left motor Ki: **60** (3x higher due to friction)
- Right motor Ki: **20** (standard)
- Synchronized start: Testing in progress

### System Resources
- Arduino Flash: **49.0%** (15,804 / 32,256 bytes)
- Arduino RAM: **26.2%** (536 / 2,048 bytes)
- EEPROM version: **2** (per-motor PID support)

### Communication Status
- Arduino ↔ ESP32: 9600 baud (SoftwareSerial)
- ESP32 ↔ LIDAR: 115200 baud
- Odometry rate: 5 Hz (target: 10 Hz)
- LIDAR RPM: 220 (target: 240)

---

## ⚠️ Known Issues

### Active Issues
1. **UI Server Development**: New version being developed in parallel window
2. **Motor Sync Testing**: In progress, results pending

### Resolved Issues
- ✅ Motor friction mismatch (per-motor PID + deadband)
- ✅ PID parameter persistence (EEPROM v2)
- ✅ Velocity matching (0.7% achieved)

---

## 🔗 Quick Links

- **Trello Board**: https://trello.com/b/4gGl7sUn/round-robot-slam-nivå-1
- **GitHub Repository**: https://github.com/Leijoma/round-robot
- **Latest Commit**: 412bebb (2026-01-08)
- **SLAM Backlog**: [SLAM_BACKLOG.md](SLAM_BACKLOG.md)
- **Project Context**: [.claude/project_context.md](.claude/project_context.md)

---

## 📝 Recent Commits

```
412bebb (2026-01-08) Add per-motor PID configuration with EEPROM persistence
fd4a80e (2026-01-08) Integrate tuned PID velocity control libraries
b0b6905 (2026-01-07) [Previous motor calibration work]
```

---

## 🎓 Lessons Learned

### Motor Control Insights
- Left motor has significantly higher static friction than right motor
- Per-motor Ki tuning is critical: Left needs 60, Right needs 20
- Deadband compensation must be per-motor: Left=50 PWM, Right=35 PWM
- EEPROM persistence prevents loss of tuned parameters across reboots

### Implementation Patterns
- Modular library architecture enables code reuse
- Protocol versioning (EEPROM v1→v2) allows backward compatibility
- UI controls for runtime tuning accelerate development

---

**Status Summary**: Motor PID configuration complete and uploaded. Testing motor synchronization in parallel. UI server updates in progress.
