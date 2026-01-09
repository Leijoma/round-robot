/**
 * PID Tuner - Test Firmware for Motor Control Tuning
 *
 * Minimal firmware focused solely on PID tuning with enhanced features:
 * - PI + Feed-forward control
 * - Velocity ramping (acceleration limiting)
 * - Low-pass filtering on velocity measurements
 * - Back-calculation anti-windup
 * - Real-time parameter adjustment via serial console
 * - High-rate CSV data streaming
 * - EEPROM parameter persistence
 * - Automated test modes (step, sweep, ramp)
 *
 * Communication: USB Serial, 9600 baud
 * Control Rate: 50 Hz (20ms)
 * Encoders: 714 ticks/rev (2X decoding with XOR logic)
 *
 * Focus: 0 - 0.7 m/s velocity range
 */

#include <Arduino.h>
#include <EEPROM.h>

// ============================================================
// Robot Configuration
// ============================================================
const float WHEEL_DIAMETER = 0.0825f;  // meters (82.5mm)
const float WHEELBASE = 0.244f;        // meters (244mm)
const float TICKS_PER_REV = 714.0f;    // 2X decoding, calibrated
const float METERS_PER_TICK = (PI * WHEEL_DIAMETER) / TICKS_PER_REV;

// ============================================================
// Pin Definitions
// ============================================================
// Motors (H-Bridge)
const uint8_t PIN_M1_INA = 7;   // Left motor direction A
const uint8_t PIN_M1_INB = 8;   // Left motor direction B
const uint8_t PIN_M1_PWM = 5;   // Left motor PWM
const uint8_t PIN_M2_INA = 4;   // Right motor direction A
const uint8_t PIN_M2_INB = 9;   // Right motor direction B
const uint8_t PIN_M2_PWM = 6;   // Right motor PWM

// Encoders
const uint8_t PIN_ENC_L_A = 2;  // INT0
const uint8_t PIN_ENC_L_B = 10;
const uint8_t PIN_ENC_R_A = 3;  // INT1
const uint8_t PIN_ENC_R_B = 11;

// ============================================================
// Enhanced PID Controller with Feed-Forward
// ============================================================
class EnhancedPID {
public:
  // PI gains
  float Kp = 10.0f;
  float Ki = 5.0f;
  float Kd = 0.0f;  // Typically not used for velocity control

  // Feed-forward gains
  float Kff_v = 0.0f;  // Velocity feed-forward
  float Kff_a = 0.0f;  // Acceleration feed-forward

  // Deadband compensation
  float deadband_forward = 30.0f;
  float deadband_reverse = 30.0f;

  // Filtering
  float filter_alpha = 0.3f;  // 0 = no filter, 1 = maximum filter

  // Anti-windup
  float max_integral = 100.0f;

  EnhancedPID() : _integral(0), _prev_error(0), _prev_target(0),
                  _vel_filtered(0), _prev_vel_filtered(0) {}

  float update(float velocity, float target, float dt) {
    if (dt <= 0) return 0;

    // Stop condition
    if (fabs(target) < 0.001f) {
      reset();
      return 0;
    }

    // 1. Low-pass filter on velocity measurement
    _vel_filtered = filter_alpha * velocity + (1.0f - filter_alpha) * _vel_filtered;

    // 2. Calculate feed-forward term (predictive)
    float accel = (target - _prev_target) / dt;
    float ff_term = Kff_v * target + Kff_a * accel;

    // 3. PI control on filtered velocity
    float error = target - _vel_filtered;

    // Proportional
    float p_term = Kp * error;

    // Integral
    _integral += error * dt;
    float i_term = Ki * _integral;

    // Derivative (optional, typically 0)
    float d_term = 0;
    if (Kd > 0) {
      float derivative = (error - _prev_error) / dt;
      d_term = Kd * derivative;
    }

    // Store for next iteration
    _prev_error = error;
    _prev_target = target;
    _prev_vel_filtered = _vel_filtered;

    // 4. Combine terms
    float pwm_raw = ff_term + p_term + i_term + d_term;

    // 5. Apply deadband
    if (target > 0.001f) {
      pwm_raw += deadband_forward;
    } else if (target < -0.001f) {
      pwm_raw -= deadband_reverse;
    }

    // 6. Clamp to PWM range
    float pwm = constrain(pwm_raw, -255.0f, 255.0f);

    // 7. Back-calculation anti-windup
    // If output was clamped, reduce integral to prevent windup
    if (pwm != pwm_raw) {
      float excess = pwm_raw - pwm;
      _integral -= excess / Ki * 0.5f;  // Back-calculate with factor
    }

    // Clamp integral
    _integral = constrain(_integral, -max_integral, max_integral);

    return pwm;
  }

  void reset() {
    _integral = 0;
    _prev_error = 0;
    _prev_target = 0;
    _vel_filtered = 0;
    _prev_vel_filtered = 0;
  }

  // Getters for telemetry
  float getIntegral() const { return _integral; }
  float getFilteredVel() const { return _vel_filtered; }
  float getPTerm(float error) const { return Kp * error; }
  float getITerm() const { return Ki * _integral; }

private:
  float _integral;
  float _prev_error;
  float _prev_target;
  float _vel_filtered;
  float _prev_vel_filtered;
};

// ============================================================
// Velocity Ramping (Acceleration Limiting)
// ============================================================
class VelocityRamp {
public:
  float max_accel = 2.0f;  // m/s²

  VelocityRamp() : _current_vel(0) {}

  float update(float target, float dt) {
    if (dt <= 0) return _current_vel;

    float delta = target - _current_vel;
    float max_change = max_accel * dt;

    if (delta > max_change) {
      delta = max_change;
    } else if (delta < -max_change) {
      delta = -max_change;
    }

    _current_vel += delta;
    return _current_vel;
  }

  void reset() {
    _current_vel = 0;
  }

  float getCurrent() const {
    return _current_vel;
  }

private:
  float _current_vel;
};

// ============================================================
// Global Variables
// ============================================================
// Encoders
volatile int32_t encoderLeftCount = 0;
volatile int32_t encoderRightCount = 0;

// Controllers
EnhancedPID pidLeft, pidRight;
VelocityRamp rampLeft, rampRight;

// Velocity tracking
float currentVelLeft = 0;
float currentVelRight = 0;
int32_t prevEncoderLeft = 0;
int32_t prevEncoderRight = 0;

// Targets
float targetVelLeft = 0;
float targetVelRight = 0;

// Timing
unsigned long lastControlUpdate = 0;
const unsigned long CONTROL_INTERVAL = 20;  // 20ms = 50Hz

// Streaming
bool streamEnabled = false;
unsigned long lastStreamOutput = 0;
unsigned long streamInterval = 20;  // Default 50Hz

// Safety
unsigned long lastCommandTime = 0;
const unsigned long SAFETY_TIMEOUT = 2000;  // 2 seconds

// Test mode
enum TestMode { NONE, STEP, SWEEP, RAMP };
TestMode currentTest = NONE;
unsigned long testStartTime = 0;
float testParam1 = 0, testParam2 = 0, testParam3 = 0;
int testStep = 0;

// Direct PWM override (for deadband testing)
bool usePWMOverride = false;
float pwmOverrideLeft = 0;
float pwmOverrideRight = 0;

// ============================================================
// Encoder ISRs - 2X Decoding with XOR Logic
// ============================================================
void isrEncoderLeft() {
  bool a = digitalRead(PIN_ENC_L_A);
  bool b = digitalRead(PIN_ENC_L_B);
  if (a == b) {
    encoderLeftCount++;
  } else {
    encoderLeftCount--;
  }
}

void isrEncoderRight() {
  bool a = digitalRead(PIN_ENC_R_A);
  bool b = digitalRead(PIN_ENC_R_B);
  if (a == b) {
    encoderRightCount++;
  } else {
    encoderRightCount--;
  }
}

// ============================================================
// Motor Control (H-Bridge)
// ============================================================
void setMotor(uint8_t pinINA, uint8_t pinINB, uint8_t pinPWM, int16_t pwm) {
  if (pwm > 0) {
    digitalWrite(pinINA, HIGH);
    digitalWrite(pinINB, LOW);
    analogWrite(pinPWM, constrain(pwm, 0, 255));
  } else if (pwm < 0) {
    digitalWrite(pinINA, LOW);
    digitalWrite(pinINB, HIGH);
    analogWrite(pinPWM, constrain(-pwm, 0, 255));
  } else {
    digitalWrite(pinINA, LOW);
    digitalWrite(pinINB, LOW);
    analogWrite(pinPWM, 0);
  }
}

void setMotors(int16_t left, int16_t right) {
  // Left motor inverted in hardware
  setMotor(PIN_M1_INA, PIN_M1_INB, PIN_M1_PWM, -left);
  setMotor(PIN_M2_INA, PIN_M2_INB, PIN_M2_PWM, right);
}

void stopMotors() {
  targetVelLeft = 0;
  targetVelRight = 0;
  setMotors(0, 0);
  pidLeft.reset();
  pidRight.reset();
  rampLeft.reset();
  rampRight.reset();
}

// ============================================================
// Velocity Calculation
// ============================================================
void updateVelocities(float dt) {
  // Get encoder counts atomically
  noInterrupts();
  int32_t encL = encoderLeftCount;
  int32_t encR = encoderRightCount;
  interrupts();

  // Calculate delta ticks
  int32_t deltaL = encL - prevEncoderLeft;
  int32_t deltaR = encR - prevEncoderRight;

  prevEncoderLeft = encL;
  prevEncoderRight = encR;

  // Calculate velocities (m/s)
  currentVelLeft = (deltaL * METERS_PER_TICK) / dt;
  currentVelRight = (deltaR * METERS_PER_TICK) / dt;
}

// ============================================================
// Control Loop Update
// ============================================================
void updateControl(float dt) {
  // 1. Calculate velocities
  updateVelocities(dt);

  float pwmLeft, pwmRight;

  if (usePWMOverride) {
    // Direct PWM control (bypasses PID for deadband testing)
    pwmLeft = pwmOverrideLeft;
    pwmRight = pwmOverrideRight;
  } else {
    // 2. Apply acceleration limiting (ramping)
    float rampedLeft = rampLeft.update(targetVelLeft, dt);
    float rampedRight = rampRight.update(targetVelRight, dt);

    // 3. PID update
    pwmLeft = pidLeft.update(currentVelLeft, rampedLeft, dt);
    pwmRight = pidRight.update(currentVelRight, rampedRight, dt);
  }

  // 4. Apply to motors
  setMotors((int16_t)pwmLeft, (int16_t)pwmRight);
}

// ============================================================
// CSV Streaming Output
// ============================================================
void streamData() {
  if (!streamEnabled) return;

  unsigned long now = millis();
  if (now - lastStreamOutput < streamInterval) return;
  lastStreamOutput = now;

  // CSV format: time,target_L,target_R,vel_L,vel_R,vel_filt_L,vel_filt_R,
  //             pwm_L,pwm_R,enc_L,enc_R,error_L,error_R,ramp_L,ramp_R

  noInterrupts();
  int32_t encL = encoderLeftCount;
  int32_t encR = encoderRightCount;
  interrupts();

  float rampedL = rampLeft.getCurrent();
  float rampedR = rampRight.getCurrent();
  float errorL = rampedL - currentVelLeft;
  float errorR = rampedR - currentVelRight;

  Serial.print(now);
  Serial.print(',');
  Serial.print(targetVelLeft, 4);
  Serial.print(',');
  Serial.print(targetVelRight, 4);
  Serial.print(',');
  Serial.print(currentVelLeft, 4);
  Serial.print(',');
  Serial.print(currentVelRight, 4);
  Serial.print(',');
  Serial.print(pidLeft.getFilteredVel(), 4);
  Serial.print(',');
  Serial.print(pidRight.getFilteredVel(), 4);
  Serial.print(',');

  // Get current PWM (recalculate for output)
  float pwmL = pidLeft.update(currentVelLeft, rampedL, streamInterval / 1000.0f);
  float pwmR = pidRight.update(currentVelRight, rampedR, streamInterval / 1000.0f);

  Serial.print(pwmL, 1);
  Serial.print(',');
  Serial.print(pwmR, 1);
  Serial.print(',');
  Serial.print(encL);
  Serial.print(',');
  Serial.print(encR);
  Serial.print(',');
  Serial.print(errorL, 4);
  Serial.print(',');
  Serial.print(errorR, 4);
  Serial.print(',');
  Serial.print(rampedL, 4);
  Serial.print(',');
  Serial.println(rampedR, 4);
}

// ============================================================
// EEPROM Save/Load
// ============================================================
struct TuningParams {
  float kp_left, ki_left, kd_left;
  float kp_right, ki_right, kd_right;
  float kff_v_left, kff_a_left;
  float kff_v_right, kff_a_right;
  float db_fwd_left, db_rev_left;
  float db_fwd_right, db_rev_right;
  float filter_alpha;
  float max_accel;
  uint16_t magic;  // 0xABCD for validation
};

void saveParams() {
  TuningParams params;
  params.kp_left = pidLeft.Kp;
  params.ki_left = pidLeft.Ki;
  params.kd_left = pidLeft.Kd;
  params.kp_right = pidRight.Kp;
  params.ki_right = pidRight.Ki;
  params.kd_right = pidRight.Kd;
  params.kff_v_left = pidLeft.Kff_v;
  params.kff_a_left = pidLeft.Kff_a;
  params.kff_v_right = pidRight.Kff_v;
  params.kff_a_right = pidRight.Kff_a;
  params.db_fwd_left = pidLeft.deadband_forward;
  params.db_rev_left = pidLeft.deadband_reverse;
  params.db_fwd_right = pidRight.deadband_forward;
  params.db_rev_right = pidRight.deadband_reverse;
  params.filter_alpha = pidLeft.filter_alpha;
  params.max_accel = rampLeft.max_accel;
  params.magic = 0xABCD;

  EEPROM.put(0, params);
  Serial.println("# Parameters saved to EEPROM");
}

void loadParams() {
  TuningParams params;
  EEPROM.get(0, params);

  if (params.magic != 0xABCD) {
    Serial.println("# No valid parameters in EEPROM");
    return;
  }

  pidLeft.Kp = params.kp_left;
  pidLeft.Ki = params.ki_left;
  pidLeft.Kd = params.kd_left;
  pidRight.Kp = params.kp_right;
  pidRight.Ki = params.ki_right;
  pidRight.Kd = params.kd_right;
  pidLeft.Kff_v = params.kff_v_left;
  pidLeft.Kff_a = params.kff_a_left;
  pidRight.Kff_v = params.kff_v_right;
  pidRight.Kff_a = params.kff_a_right;
  pidLeft.deadband_forward = params.db_fwd_left;
  pidLeft.deadband_reverse = params.db_rev_left;
  pidRight.deadband_forward = params.db_fwd_right;
  pidRight.deadband_reverse = params.db_rev_right;
  pidLeft.filter_alpha = params.filter_alpha;
  pidRight.filter_alpha = params.filter_alpha;
  rampLeft.max_accel = params.max_accel;
  rampRight.max_accel = params.max_accel;

  Serial.println("# Parameters loaded from EEPROM");
}

// ============================================================
// Test Modes
// ============================================================
void startStepTest(float velocity, float duration) {
  Serial.print("# Starting step test: ");
  Serial.print(velocity);
  Serial.print(" m/s for ");
  Serial.print(duration);
  Serial.println(" seconds");

  currentTest = STEP;
  testStartTime = millis();
  testParam1 = velocity;  // Target velocity
  testParam2 = duration * 1000;  // Duration in ms
  testStep = 0;

  targetVelLeft = 0;
  targetVelRight = 0;
}

void updateStepTest() {
  unsigned long elapsed = millis() - testStartTime;

  if (testStep == 0) {
    // Step to target
    targetVelLeft = testParam1;
    targetVelRight = testParam1;
    testStep = 1;
  } else if (testStep == 1 && elapsed > testParam2) {
    // Step back to zero
    targetVelLeft = 0;
    targetVelRight = 0;
    testStep = 2;
  } else if (testStep == 2 && elapsed > testParam2 + 2000) {
    // Done
    Serial.println("# Step test complete");
    currentTest = NONE;
    stopMotors();
  }
}

void startSweepTest(float start_vel, float end_vel, float step_size) {
  Serial.println("# Starting velocity sweep test");
  currentTest = SWEEP;
  testStartTime = millis();
  testParam1 = start_vel;
  testParam2 = end_vel;
  testParam3 = step_size;
  testStep = 0;

  targetVelLeft = start_vel;
  targetVelRight = start_vel;
}

void updateSweepTest() {
  unsigned long elapsed = millis() - testStartTime;

  // Hold each velocity for 3 seconds
  if (elapsed > 3000) {
    float nextVel = targetVelLeft + testParam3;

    if ((testParam3 > 0 && nextVel > testParam2) ||
        (testParam3 < 0 && nextVel < testParam2)) {
      // Done
      Serial.println("# Sweep test complete");
      currentTest = NONE;
      stopMotors();
    } else {
      targetVelLeft = nextVel;
      targetVelRight = nextVel;
      testStartTime = millis();
    }
  }
}

void startRampTest(float start_vel, float end_vel, float ramp_time) {
  Serial.println("# Starting acceleration ramp test");
  currentTest = RAMP;
  testStartTime = millis();
  testParam1 = start_vel;
  testParam2 = end_vel;
  testParam3 = ramp_time * 1000;  // Duration in ms

  targetVelLeft = start_vel;
  targetVelRight = start_vel;
}

void updateRampTest() {
  unsigned long elapsed = millis() - testStartTime;

  if (elapsed < testParam3) {
    // Linear ramp
    float progress = elapsed / testParam3;
    float vel = testParam1 + (testParam2 - testParam1) * progress;
    targetVelLeft = vel;
    targetVelRight = vel;
  } else if (elapsed < testParam3 + 2000) {
    // Hold at end velocity for 2 seconds
    targetVelLeft = testParam2;
    targetVelRight = testParam2;
  } else {
    // Done
    Serial.println("# Ramp test complete");
    currentTest = NONE;
    stopMotors();
  }
}

void updateTest() {
  switch (currentTest) {
    case STEP:
      updateStepTest();
      break;
    case SWEEP:
      updateSweepTest();
      break;
    case RAMP:
      updateRampTest();
      break;
    default:
      break;
  }
}

// ============================================================
// Command Processing
// ============================================================
void printHelp() {
  Serial.println(F("\n=== PID Tuner Commands ==="));
  Serial.println(F("Velocity Control:"));
  Serial.println(F("  v <left> <right>     - Set target velocities (m/s)"));
  Serial.println(F("\nPID Gains (applies to both motors):"));
  Serial.println(F("  kp <value>           - Set Kp gain"));
  Serial.println(F("  ki <value>           - Set Ki gain"));
  Serial.println(F("  kd <value>           - Set Kd gain"));
  Serial.println(F("\nFeed-Forward:"));
  Serial.println(F("  ff_v <value>         - Set velocity FF gain"));
  Serial.println(F("  ff_a <value>         - Set acceleration FF gain"));
  Serial.println(F("\nDeadband:"));
  Serial.println(F("  db <fwd> <rev>       - Set deadband (PWM)"));
  Serial.println(F("\nFiltering & Acceleration:"));
  Serial.println(F("  filter <alpha>       - Set velocity filter (0-1)"));
  Serial.println(F("  accel <value>        - Set max acceleration (m/s^2)"));
  Serial.println(F("\nTest Modes:"));
  Serial.println(F("  step <vel> <dur>     - Step response test"));
  Serial.println(F("  sweep <st> <end> <step> - Velocity sweep"));
  Serial.println(F("  ramp <st> <end> <time> - Acceleration ramp"));
  Serial.println(F("\nData & Config:"));
  Serial.println(F("  stream <on/off>      - Toggle data streaming"));
  Serial.println(F("  rate <hz>            - Set stream rate (10-100 Hz)"));
  Serial.println(F("  save                 - Save params to EEPROM"));
  Serial.println(F("  load                 - Load params from EEPROM"));
  Serial.println(F("  reset                - Reset PID controllers"));
  Serial.println(F("  stop                 - Emergency stop"));
  Serial.println(F("  status               - Show current parameters"));
  Serial.println(F("  help                 - Show this help"));
  Serial.println();
}

void printStatus() {
  Serial.println(F("\n=== Current Parameters ==="));
  Serial.print(F("Kp: ")); Serial.println(pidLeft.Kp);
  Serial.print(F("Ki: ")); Serial.println(pidLeft.Ki);
  Serial.print(F("Kd: ")); Serial.println(pidLeft.Kd);
  Serial.print(F("Feed-forward velocity: ")); Serial.println(pidLeft.Kff_v);
  Serial.print(F("Feed-forward accel: ")); Serial.println(pidLeft.Kff_a);
  Serial.print(F("Deadband fwd: ")); Serial.println(pidLeft.deadband_forward);
  Serial.print(F("Deadband rev: ")); Serial.println(pidLeft.deadband_reverse);
  Serial.print(F("Filter alpha: ")); Serial.println(pidLeft.filter_alpha);
  Serial.print(F("Max accel: ")); Serial.println(rampLeft.max_accel);
  Serial.print(F("Streaming: ")); Serial.println(streamEnabled ? "ON" : "OFF");
  Serial.print(F("Stream rate: ")); Serial.print(1000.0f / streamInterval); Serial.println(" Hz");
  Serial.println();
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  lastCommandTime = millis();  // Reset safety timeout

  // Parse command
  int spaceIdx = cmd.indexOf(' ');
  String command = (spaceIdx > 0) ? cmd.substring(0, spaceIdx) : cmd;
  String args = (spaceIdx > 0) ? cmd.substring(spaceIdx + 1) : "";

  command.toLowerCase();

  if (command == "v") {
    // Set velocity
    int comma = args.indexOf(' ');
    if (comma > 0) {
      usePWMOverride = false;  // Disable PWM override when using velocity control
      targetVelLeft = args.substring(0, comma).toFloat();
      targetVelRight = args.substring(comma + 1).toFloat();
      Serial.print("# Target velocities: L=");
      Serial.print(targetVelLeft);
      Serial.print(" R=");
      Serial.println(targetVelRight);
    }
  }
  else if (command == "pwm") {
    // Direct PWM control (bypasses PID for deadband testing)
    int comma = args.indexOf(' ');
    if (comma > 0) {
      usePWMOverride = true;
      pwmOverrideLeft = constrain(args.substring(0, comma).toFloat(), -255.0f, 255.0f);
      pwmOverrideRight = constrain(args.substring(comma + 1).toFloat(), -255.0f, 255.0f);
      Serial.print("# Direct PWM: L=");
      Serial.print(pwmOverrideLeft);
      Serial.print(" R=");
      Serial.println(pwmOverrideRight);
    }
  }
  else if (command == "kp") {
    pidLeft.Kp = pidRight.Kp = args.toFloat();
    Serial.print("# Kp = "); Serial.println(pidLeft.Kp);
  }
  else if (command == "ki") {
    pidLeft.Ki = pidRight.Ki = args.toFloat();
    Serial.print("# Ki = "); Serial.println(pidLeft.Ki);
  }
  else if (command == "kd") {
    pidLeft.Kd = pidRight.Kd = args.toFloat();
    Serial.print("# Kd = "); Serial.println(pidLeft.Kd);
  }
  else if (command == "ff_v") {
    pidLeft.Kff_v = pidRight.Kff_v = args.toFloat();
    Serial.print("# FF velocity = "); Serial.println(pidLeft.Kff_v);
  }
  else if (command == "ff_a") {
    pidLeft.Kff_a = pidRight.Kff_a = args.toFloat();
    Serial.print("# FF accel = "); Serial.println(pidLeft.Kff_a);
  }
  else if (command == "db") {
    int comma = args.indexOf(' ');
    if (comma > 0) {
      pidLeft.deadband_forward = pidRight.deadband_forward = args.substring(0, comma).toFloat();
      pidLeft.deadband_reverse = pidRight.deadband_reverse = args.substring(comma + 1).toFloat();
      Serial.print("# Deadband: fwd=");
      Serial.print(pidLeft.deadband_forward);
      Serial.print(" rev=");
      Serial.println(pidLeft.deadband_reverse);
    }
  }
  else if (command == "filter") {
    pidLeft.filter_alpha = pidRight.filter_alpha = constrain(args.toFloat(), 0.0f, 1.0f);
    Serial.print("# Filter alpha = "); Serial.println(pidLeft.filter_alpha);
  }
  else if (command == "accel") {
    rampLeft.max_accel = rampRight.max_accel = args.toFloat();
    Serial.print("# Max accel = "); Serial.println(rampLeft.max_accel);
  }
  else if (command == "stream") {
    args.toLowerCase();
    if (args == "on" || args == "1") {
      streamEnabled = true;
      Serial.println("# CSV Header:");
      Serial.println("time_ms,target_L,target_R,vel_L,vel_R,vel_filt_L,vel_filt_R,pwm_L,pwm_R,enc_L,enc_R,error_L,error_R,ramp_L,ramp_R");
    } else {
      streamEnabled = false;
      Serial.println("# Streaming OFF");
    }
  }
  else if (command == "rate") {
    float hz = constrain(args.toFloat(), 10.0f, 100.0f);
    streamInterval = 1000.0f / hz;
    Serial.print("# Stream rate = "); Serial.print(hz); Serial.println(" Hz");
  }
  else if (command == "step") {
    // step <vel> <duration>
    int sp1 = args.indexOf(' ');
    if (sp1 > 0) {
      float vel = args.substring(0, sp1).toFloat();
      float dur = args.substring(sp1 + 1).toFloat();
      startStepTest(vel, dur);
    }
  }
  else if (command == "sweep") {
    // sweep <start> <end> <step>
    int sp1 = args.indexOf(' ');
    int sp2 = args.indexOf(' ', sp1 + 1);
    if (sp1 > 0 && sp2 > 0) {
      float start = args.substring(0, sp1).toFloat();
      float end = args.substring(sp1 + 1, sp2).toFloat();
      float step = args.substring(sp2 + 1).toFloat();
      startSweepTest(start, end, step);
    }
  }
  else if (command == "ramp") {
    // ramp <start> <end> <time>
    int sp1 = args.indexOf(' ');
    int sp2 = args.indexOf(' ', sp1 + 1);
    if (sp1 > 0 && sp2 > 0) {
      float start = args.substring(0, sp1).toFloat();
      float end = args.substring(sp1 + 1, sp2).toFloat();
      float time = args.substring(sp2 + 1).toFloat();
      startRampTest(start, end, time);
    }
  }
  else if (command == "save") {
    saveParams();
  }
  else if (command == "load") {
    loadParams();
  }
  else if (command == "reset") {
    pidLeft.reset();
    pidRight.reset();
    rampLeft.reset();
    rampRight.reset();
    Serial.println("# Controllers reset");
  }
  else if (command == "stop") {
    currentTest = NONE;
    stopMotors();
    Serial.println("# Emergency stop");
  }
  else if (command == "zero") {
    noInterrupts();
    encoderLeftCount = 0;
    encoderRightCount = 0;
    interrupts();
    prevEncoderLeft = 0;
    prevEncoderRight = 0;
    currentVelLeft = 0;
    currentVelRight = 0;
    Serial.println("# Encoders zeroed");
  }
  else if (command == "status") {
    printStatus();
  }
  else if (command == "help") {
    printHelp();
  }
  else {
    Serial.print("# Unknown command: ");
    Serial.println(command);
    Serial.println("# Type 'help' for commands");
  }
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println(F("\n========================================"));
  Serial.println(F("  PID Tuner v1.0"));
  Serial.println(F("  Motor Control Tuning Firmware"));
  Serial.println(F("========================================"));
  Serial.println();

  // Setup motor pins
  pinMode(PIN_M1_INA, OUTPUT);
  pinMode(PIN_M1_INB, OUTPUT);
  pinMode(PIN_M1_PWM, OUTPUT);
  pinMode(PIN_M2_INA, OUTPUT);
  pinMode(PIN_M2_INB, OUTPUT);
  pinMode(PIN_M2_PWM, OUTPUT);

  // Setup encoder pins
  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);

  // Attach interrupts - CHANGE mode for 2X decoding
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncoderRight, CHANGE);

  // Stop motors
  stopMotors();

  // Try to load saved parameters
  loadParams();

  Serial.println(F("Hardware initialized"));
  Serial.println(F("Type 'help' for command list"));
  Serial.println();

  lastCommandTime = millis();
  lastControlUpdate = millis();
}

// ============================================================
// Main Loop
// ============================================================
void loop() {
  unsigned long now = millis();

  // Control loop (50 Hz)
  if (now - lastControlUpdate >= CONTROL_INTERVAL) {
    float dt = (now - lastControlUpdate) / 1000.0f;
    lastControlUpdate = now;

    // Update control
    updateControl(dt);

    // Update test mode
    if (currentTest != NONE) {
      updateTest();
    }
  }

  // Stream data
  streamData();

  // Process serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }

  // Safety timeout - stop if no command for 2 seconds (disabled during streaming)
  if (!streamEnabled && now - lastCommandTime > SAFETY_TIMEOUT &&
      (targetVelLeft != 0 || targetVelRight != 0)) {
    stopMotors();
    Serial.println("# Safety timeout - motors stopped");
    lastCommandTime = now;
  }
}
