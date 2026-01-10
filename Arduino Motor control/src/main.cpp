// main.cpp - Complete updated version (odometry sign fix + atomic cleanup + optional debug)
//
// Changes vs your last version:
// 1) Odometry: deltaTheta uses standard differential-drive convention (right - left)
// 2) Removed redundant noInterrupts()/interrupts() around getCount()/getErrorCount()
//    because Encoder::getCount() already reads atomically.
// 3) sendOdometry(): removed nested atomic block for encoder reads.
// 4) Added optional odometry debug (~1 Hz) showing delta ticks and derived dC/dTheta.
// 5) Fixed one small logic hazard: keep ODOM_INTERVAL constant but use streamInterval for send rate.
//
// IMPORTANT:
// - Encoder reverse flags must be set so that FORWARD motion produces POSITIVE ticks on both wheels.
//   If your debug shows otherwise, flip encoderLeft/encoderRight reverse booleans (ONLY here).

#include <Arduino.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "RobotLink.h"
#include "RobotLinkMessages.h"
#include <Encoder.h>
#include <MotorControl.h>
#include <PIDController.h>

// ============================================================
// Build Options
// ============================================================
#define ODOM_DEBUG 1   // set to 0 to disable debug prints

// ============================================================
// Hardware Pin Definitions (Monster Moto Shield + Encoders)
// ============================================================
// Left Motor (M1)
const uint8_t PIN_M1_INA = 7;
const uint8_t PIN_M1_INB = 8;
const uint8_t PIN_M1_PWM = 5;

// Right Motor (M2)
const uint8_t PIN_M2_INA = 4;
const uint8_t PIN_M2_INB = 9;
const uint8_t PIN_M2_PWM = 6;

// Left Encoder
const uint8_t PIN_ENC_L_A = 2;  // INT0
const uint8_t PIN_ENC_L_B = 10;

// Right Encoder
const uint8_t PIN_ENC_R_A = 3;  // INT1
const uint8_t PIN_ENC_R_B = 11;

// SoftwareSerial pins for RobotLink protocol
const uint8_t PIN_SOFT_RX = A0;  // Arduino RX <- ESP32 GPIO27 TX
const uint8_t PIN_SOFT_TX = A1;  // Arduino TX -> ESP32 GPIO26 RX

// ============================================================
// Robot Configuration (loaded from EEPROM, these are defaults)
// ============================================================
float WHEEL_DIAMETER = 0.0790f;  // meters (79mm, measured with robot weight)
float WHEELBASE = 0.244f;        // meters (244mm)
float TICKS_PER_REV = 594.0f;    // encoder ticks per revolution (x2 decoding assumed)
float METERS_PER_TICK = (PI * WHEEL_DIAMETER) / TICKS_PER_REV;

// ============================================================
// Library Objects - Motors, Encoders, PID
// ============================================================
// IMPORTANT SIGN CONVENTION (recommended):
// Forward motion must yield POSITIVE ticks for BOTH wheels.
// If your debug shows opposite sign, change these reverse flags.
Encoder encoderLeft(PIN_ENC_L_A, PIN_ENC_L_B, true);     // currently reversed
Encoder encoderRight(PIN_ENC_R_A, PIN_ENC_R_B, true);    // currently reversed

MotorControl motorLeft(PIN_M1_INA, PIN_M1_INB, PIN_M1_PWM, true);    // Reversed
MotorControl motorRight(PIN_M2_INA, PIN_M2_INB, PIN_M2_PWM, false);  // Normal

// ============================================================
// Global Variables
// ============================================================
// PID Controllers
PIDController pidLeft;
PIDController pidRight;

// Velocity targets (m/s)
float targetVelLeft = 0;
float targetVelRight = 0;

// Ramped targets for smooth deceleration (m/s)
float rampedTargetLeft = 0;
float rampedTargetRight = 0;

// Current velocities (m/s)
float currentVelLeft = 0;
float currentVelRight = 0;

// PWM outputs
int16_t pwmLeft = 0;
int16_t pwmRight = 0;

// cmd_vel targets (v, w)
float target_v = 0;  // Linear velocity (m/s)
float target_w = 0;  // Angular velocity (rad/s)
float ramped_v = 0;  // Ramped linear velocity for smooth accel/decel
float ramped_w = 0;  // Ramped angular velocity for smooth accel/decel
bool use_cmd_vel = false;  // Use cmd_vel mode instead of direct wheel velocities

// Heading hold gain (for angular velocity error correction)
float headingHoldKp = 15.0f;

// Odometry pose (float to avoid accumulation errors)
float pose_x_mm = 0.0f;
float pose_y_mm = 0.0f;
float pose_th_mrad = 0.0f;

// Previous encoder values for velocity delta calculation
int32_t prevEncoderLeft = 0;
int32_t prevEncoderRight = 0;

// Timing
unsigned long lastControlUpdate = 0;
unsigned long lastOdomSend = 0;
const unsigned long CONTROL_INTERVAL = 50;  // 50ms = 20Hz
const unsigned long ODOM_INTERVAL = 50;     // 50ms = 20Hz (kept for reference)

// Minimum velocity threshold
const float MIN_VELOCITY_THRESHOLD = 0.08f;  // m/s

// Deceleration rate (m/s²)
const float MAX_DECELERATION = 10.0f;  // TESTING

// SoftwareSerial for ESP32 communication
SoftwareSerial softSerial(PIN_SOFT_RX, PIN_SOFT_TX);

// RobotLink protocol
RobotLink::Link* robotLink = nullptr;

// Configuration
bool streamEnabled = false;
uint16_t streamInterval = 50;  // ms

// ============================================================
// Encoder Interrupt Service Routines
// ============================================================
void isrEncoderLeft() {
  encoderLeft.handleInterrupt();
}

void isrEncoderRight() {
  encoderRight.handleInterrupt();
}

// ============================================================
// Motor Control Functions
// ============================================================
void setMotors(int16_t left, int16_t right) {
  motorLeft.setPWM(left);
  motorRight.setPWM(right);
  pwmLeft = left;
  pwmRight = right;
}

void stopMotors() {
  targetVelLeft = 0;
  targetVelRight = 0;
  target_v = 0;
  target_w = 0;
  rampedTargetLeft = 0;
  rampedTargetRight = 0;
  ramped_v = 0;
  ramped_w = 0;
  use_cmd_vel = false;
  setMotors(0, 0);
  pidLeft.reset();
  pidRight.reset();
}

// ============================================================
// Velocity Calculation
// ============================================================
void updateVelocities(float dt) {
  // getCount() already handles atomicity
  int32_t encL = encoderLeft.getCount();
  int32_t encR = encoderRight.getCount();

  int32_t deltaL = encL - prevEncoderLeft;
  int32_t deltaR = encR - prevEncoderRight;

  prevEncoderLeft = encL;
  prevEncoderRight = encR;

  currentVelLeft = (deltaL * METERS_PER_TICK) / dt;
  currentVelRight = (deltaR * METERS_PER_TICK) / dt;
}

// ============================================================
// Odometry Update
// ============================================================
static int32_t prevOdomEncoderLeft = 0;
static int32_t prevOdomEncoderRight = 0;

void updateOdometry(float /*dt*/) {
  // getCount() already handles atomicity
  int32_t encL = encoderLeft.getCount();
  int32_t encR = encoderRight.getCount();

  int32_t deltaL = encL - prevOdomEncoderLeft;
  int32_t deltaR = encR - prevOdomEncoderRight;

  prevOdomEncoderLeft = encL;
  prevOdomEncoderRight = encR;

  float deltaDistLeft  = deltaL * METERS_PER_TICK * 1000.0f;  // mm
  float deltaDistRight = deltaR * METERS_PER_TICK * 1000.0f;  // mm

  // Standard differential drive kinematics:
  // dS = (dR + dL)/2
  // dTheta = (dR - dL)/wheelbase
  float deltaDistCenter = (deltaDistLeft + deltaDistRight) / 2.0f;
  float deltaTheta = (deltaDistRight - deltaDistLeft) / (WHEELBASE * 1000.0f);  // radians

#if ODOM_DEBUG
  static uint32_t dbg = 0;
  if (++dbg % 20 == 0) { // ~1 Hz at 20 Hz loop
    Serial.print(F("ODOM dL="));  Serial.print(deltaL);
    Serial.print(F(" dR="));      Serial.print(deltaR);
    Serial.print(F(" dC="));      Serial.print(deltaDistCenter, 1);
    Serial.print(F("mm dTh="));   Serial.println(deltaTheta, 6);
  }
#endif

  float theta = pose_th_mrad / 1000.0f;  // radians

  // Midpoint integration
  pose_x_mm += deltaDistCenter * cos(theta + deltaTheta / 2.0f);
  pose_y_mm += deltaDistCenter * sin(theta + deltaTheta / 2.0f);
  pose_th_mrad += deltaTheta * 1000.0f;

  // Wrap angle to [-pi, pi] in mrad
  while (pose_th_mrad > 3142) pose_th_mrad -= 6283;
  while (pose_th_mrad < -3142) pose_th_mrad += 6283;
}

// ============================================================
// Velocity Ramping for Smooth Acceleration/Deceleration
// ============================================================
float rampVelocity(float current, float target, float dt) {
  float maxDelta = MAX_DECELERATION * dt;
  float delta = target - current;

  if (delta > maxDelta) delta = maxDelta;
  else if (delta < -maxDelta) delta = -maxDelta;

  float newVel = current + delta;

  // Snap to zero near deadband when target is ~0
  if (abs(newVel) < MIN_VELOCITY_THRESHOLD && abs(target) < 0.01f) {
    return 0.0f;
  }

  return newVel;
}

// ============================================================
// PID Control Update
// ============================================================
void updateControl(float dt) {
  updateVelocities(dt);

  rampedTargetLeft  = rampVelocity(rampedTargetLeft,  targetVelLeft,  dt);
  rampedTargetRight = rampVelocity(rampedTargetRight, targetVelRight, dt);

  float correctedTargetLeft, correctedTargetRight;

  if (use_cmd_vel) {
    ramped_v = rampVelocity(ramped_v, target_v, dt);
    ramped_w = rampVelocity(ramped_w, target_w, dt);

    float wheelbase_half = WHEELBASE / 2.0f;
    float baseTargetLeft  = ramped_v - ramped_w * wheelbase_half;
    float baseTargetRight = ramped_v + ramped_w * wheelbase_half;

    bool wantToMove = (abs(ramped_v) > 0.01f || abs(ramped_w) > 0.01f);

    float actual_w = 0.0f;
    float w_error = 0.0f;
    float correction = 0.0f;

    if (wantToMove) {
      actual_w = (currentVelRight - currentVelLeft) / WHEELBASE;
      w_error = target_w - actual_w;

      correction = headingHoldKp * w_error * wheelbase_half;

      correctedTargetLeft  = baseTargetLeft  - correction;
      correctedTargetRight = baseTargetRight + correction;
    } else {
      correctedTargetLeft  = baseTargetLeft;
      correctedTargetRight = baseTargetRight;
    }

    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500 && (abs(target_v) > 0.01f || abs(target_w) > 0.01f)) {
      lastDebug = millis();
      Serial.print(F("cmd_vel: v="));
      Serial.print(target_v, 2);
      Serial.print(F(" w="));
      Serial.print(target_w, 3);
      Serial.print(F(" → tgt["));
      Serial.print(baseTargetLeft, 2);
      Serial.print(F(","));
      Serial.print(baseTargetRight, 2);
      Serial.print(F("] act_w="));
      Serial.print(actual_w, 3);
      Serial.print(F(" err="));
      Serial.print(w_error, 3);
      Serial.print(F(" corr="));
      Serial.println(correction, 3);
    }
  } else {
    correctedTargetLeft  = rampedTargetLeft;
    correctedTargetRight = rampedTargetRight;
  }

  float pwmLf = pidLeft.update(currentVelLeft, correctedTargetLeft, dt);
  float pwmRf = pidRight.update(currentVelRight, correctedTargetRight, dt);

  int16_t pwmL = (int16_t)pwmLf;
  int16_t pwmR = (int16_t)pwmRf;

  setMotors(pwmL, pwmR);
}

// ============================================================
// EEPROM Configuration Management
// ============================================================
const uint16_t EEPROM_MAGIC = 0xAB12;
const uint16_t EEPROM_VERSION = 3;

struct EEPROMConfig {
  uint16_t magic;
  uint16_t version;
  float leftKp, leftKi, leftKd;
  float rightKp, rightKi, rightKd;
  float deadbandLeftFwd, deadbandLeftRev;
  float deadbandRightFwd, deadbandRightRev;
  float wheelDiameter;
  float wheelbase;
  float ticksPerRev;
  float headingHoldKp;
  uint8_t checksum;
} __attribute__((packed));

uint8_t calcChecksum(const EEPROMConfig& cfg) {
  uint8_t sum = 0;
  const uint8_t* ptr = (const uint8_t*)&cfg;
  for (size_t i = 0; i < sizeof(EEPROMConfig) - 1; i++) {
    sum ^= ptr[i];
  }
  return sum;
}

bool saveConfig() {
  Serial.println(F(">>> saveConfig() called"));

  EEPROMConfig cfg;
  cfg.magic = EEPROM_MAGIC;
  cfg.version = EEPROM_VERSION;

  cfg.leftKp = pidLeft.Kp;
  cfg.leftKi = pidLeft.Ki;
  cfg.leftKd = pidLeft.Kd;
  cfg.rightKp = pidRight.Kp;
  cfg.rightKi = pidRight.Ki;
  cfg.rightKd = pidRight.Kd;

  cfg.deadbandLeftFwd = pidLeft.deadband_forward;
  cfg.deadbandLeftRev = pidLeft.deadband_reverse;
  cfg.deadbandRightFwd = pidRight.deadband_forward;
  cfg.deadbandRightRev = pidRight.deadband_reverse;

  cfg.wheelDiameter = WHEEL_DIAMETER;
  cfg.wheelbase = WHEELBASE;
  cfg.ticksPerRev = TICKS_PER_REV;
  cfg.headingHoldKp = headingHoldKp;

  cfg.checksum = calcChecksum(cfg);

  Serial.print(F("  Saving: magic=0x")); Serial.print(cfg.magic, HEX);
  Serial.print(F(", ver=")); Serial.print(cfg.version);
  Serial.print(F(", checksum=0x")); Serial.println(cfg.checksum, HEX);

  EEPROM.put(0, cfg);

  Serial.println(F("  EEPROM.put() completed"));
  Serial.println(F("<<< saveConfig() done"));
  return true;
}

bool loadConfig() {
  Serial.println(F(">>> loadConfig() called"));

  EEPROMConfig cfg;
  EEPROM.get(0, cfg);

  Serial.print(F("  Read from EEPROM: magic=0x")); Serial.print(cfg.magic, HEX);
  Serial.print(F(", ver=")); Serial.print(cfg.version);
  Serial.print(F(", checksum=0x")); Serial.println(cfg.checksum, HEX);

  if (cfg.magic != EEPROM_MAGIC) {
    Serial.print(F("EEPROM: Invalid magic number 0x")); Serial.print(cfg.magic, HEX);
    Serial.print(F(" (expected 0x")); Serial.print(EEPROM_MAGIC, HEX); Serial.println(F(")"));
    return false;
  }

  if (cfg.version != 2 && cfg.version != EEPROM_VERSION) {
    Serial.print(F("EEPROM: Unsupported version "));
    Serial.println(cfg.version);
    return false;
  }

  if (cfg.version == EEPROM_VERSION) {
    uint8_t expectedChecksum = calcChecksum(cfg);
    if (cfg.checksum != expectedChecksum) {
      Serial.print(F("EEPROM: Checksum mismatch - got 0x"));
      Serial.print(cfg.checksum, HEX);
      Serial.print(F(", expected 0x"));
      Serial.println(expectedChecksum, HEX);
      return false;
    }
  }

  Serial.println(F("  Loading values from EEPROM:"));

  pidLeft.Kp = cfg.leftKp;
  pidLeft.Ki = cfg.leftKi;
  pidLeft.Kd = cfg.leftKd;

  pidRight.Kp = cfg.rightKp;
  pidRight.Ki = cfg.rightKi;
  pidRight.Kd = cfg.rightKd;

  pidLeft.deadband_forward = cfg.deadbandLeftFwd;
  pidLeft.deadband_reverse = cfg.deadbandLeftRev;
  pidRight.deadband_forward = cfg.deadbandRightFwd;
  pidRight.deadband_reverse = cfg.deadbandRightRev;

  WHEEL_DIAMETER = cfg.wheelDiameter;
  WHEELBASE = cfg.wheelbase;
  TICKS_PER_REV = cfg.ticksPerRev;
  METERS_PER_TICK = (PI * WHEEL_DIAMETER) / TICKS_PER_REV;

  Serial.print(F("    Robot Geometry: wheel_diam=")); Serial.print(WHEEL_DIAMETER * 1000);
  Serial.print(F("mm, wheelbase=")); Serial.print(WHEELBASE * 1000);
  Serial.print(F("mm, ticks/rev=")); Serial.println(TICKS_PER_REV, 0);

  if (cfg.version == EEPROM_VERSION) {
    headingHoldKp = cfg.headingHoldKp;
    Serial.print(F("    Heading Hold Kp=")); Serial.println(headingHoldKp);
  }

  if (cfg.version == 2) {
    Serial.println(F("EEPROM: Migrating from v2 to v3"));
    saveConfig();
  }

  Serial.println(F("<<< loadConfig() successful"));
  return true;
}

// ============================================================
// RobotLink Protocol Helpers
// ============================================================
void sendAck(uint8_t originalMsgType, uint8_t status = 0) {
  RobotLink::AckPayload ack;
  ack.originalMsgType = originalMsgType;
  ack.status = status;
  ack.reserved[0] = 0;
  ack.reserved[1] = 0;
  robotLink->sendFrame(RobotLink::MSG_ACK, (const uint8_t*)&ack, sizeof(ack));
}

void sendNack(uint8_t originalMsgType, uint8_t errorCode) {
  RobotLink::NackPayload nack;
  nack.originalMsgType = originalMsgType;
  nack.errorCode = errorCode;
  nack.reserved[0] = 0;
  nack.reserved[1] = 0;
  robotLink->sendFrame(RobotLink::MSG_NACK, (const uint8_t*)&nack, sizeof(nack));
}

// ============================================================
// RobotLink Protocol Handlers
// ============================================================
void handleFrame(uint8_t type, const uint8_t* payload, uint8_t len) {
  switch (type) {
    case RobotLink::MSG_CMD_VEL: {
      if (len != 4) break;

      int16_t v_mm_s = RobotLink::Link::rd_i16_le(&payload[0]);
      int16_t w_mrad_s = RobotLink::Link::rd_i16_le(&payload[2]);

      target_v = v_mm_s / 1000.0f;
      target_w = w_mrad_s / 1000.0f;
      use_cmd_vel = true;

      static unsigned long lastLog = 0;
      if (millis() - lastLog > 500) {
        lastLog = millis();
        Serial.print(F("CMD_VEL: v="));
        Serial.print(target_v, 2);
        Serial.print(F(" w="));
        Serial.print(target_w, 3);
        if (abs(target_w) < 0.01f && abs(target_v) > 0.01f) {
          Serial.print(F(" [HEADING HOLD]"));
        }
        Serial.println();
      }

      sendAck(type);
      break;
    }

    case RobotLink::MSG_SET_VEL: {
      if (len != sizeof(RobotLink::SetVelPayload)) break;

      RobotLink::SetVelPayload* msg = (RobotLink::SetVelPayload*)payload;
      targetVelLeft = msg->velLeft;
      targetVelRight = msg->velRight;
      use_cmd_vel = false;

      Serial.print(F("SET_VEL: L="));
      Serial.print(targetVelLeft, 2);
      Serial.print(F(" R="));
      Serial.println(targetVelRight, 2);

      if (targetVelLeft == 0.0f && targetVelRight == 0.0f) {
        Serial.println(F("  --> STOP command"));
      }

      sendAck(type);
      break;
    }

    case RobotLink::MSG_SET_PID: {
      if (len != sizeof(RobotLink::SetPidPayload)) {
        sendNack(type, RobotLink::ERR_INVALID_PAYLOAD);
        break;
      }

      RobotLink::SetPidPayload* msg = (RobotLink::SetPidPayload*)payload;
      pidLeft.Kp = pidRight.Kp = msg->Kp;
      pidLeft.Ki = pidRight.Ki = msg->Ki;
      pidLeft.Kd = pidRight.Kd = msg->Kd;

      bool saved = saveConfig();
      Serial.println(F("PID updated and saved to EEPROM"));

      sendAck(type, saved ? 0 : RobotLink::ERR_EEPROM_WRITE_FAILED);
      break;
    }

    case RobotLink::MSG_SET_PID_PER_MOTOR: {
      if (len != sizeof(RobotLink::SetPidPerMotorPayload)) {
        sendNack(type, RobotLink::ERR_INVALID_PAYLOAD);
        break;
      }

      RobotLink::SetPidPerMotorPayload* msg = (RobotLink::SetPidPerMotorPayload*)payload;
      pidLeft.Kp = msg->leftKp;
      pidLeft.Ki = msg->leftKi;
      pidLeft.Kd = msg->leftKd;
      pidRight.Kp = msg->rightKp;
      pidRight.Ki = msg->rightKi;
      pidRight.Kd = msg->rightKd;

      bool saved = saveConfig();
      Serial.print(F("Per-motor PID updated: L("));
      Serial.print(pidLeft.Kp); Serial.print(F(","));
      Serial.print(pidLeft.Ki); Serial.print(F(","));
      Serial.print(pidLeft.Kd); Serial.print(F(") R("));
      Serial.print(pidRight.Kp); Serial.print(F(","));
      Serial.print(pidRight.Ki); Serial.print(F(","));
      Serial.print(pidRight.Kd); Serial.println(F(")"));

      sendAck(type, saved ? 0 : RobotLink::ERR_EEPROM_WRITE_FAILED);
      break;
    }

    case RobotLink::MSG_SET_DEADBAND: {
      if (len != sizeof(RobotLink::SetDeadbandPayload)) {
        Serial.print(F("ERROR: MSG_SET_DEADBAND wrong size: "));
        Serial.print(len);
        Serial.print(F(" expected "));
        Serial.println(sizeof(RobotLink::SetDeadbandPayload));
        sendNack(type, RobotLink::ERR_INVALID_PAYLOAD);
        break;
      }

      RobotLink::SetDeadbandPayload* msg = (RobotLink::SetDeadbandPayload*)payload;

      Serial.println(F(">>> MSG_SET_DEADBAND received"));
      Serial.print(F("  Received: leftFwd=")); Serial.print(msg->leftForward);
      Serial.print(F(", leftRev=")); Serial.print(msg->leftReverse);
      Serial.print(F(", rightFwd=")); Serial.print(msg->rightForward);
      Serial.print(F(", rightRev=")); Serial.println(msg->rightReverse);

      pidLeft.deadband_forward = msg->leftForward;
      pidLeft.deadband_reverse = msg->leftReverse;
      pidRight.deadband_forward = msg->rightForward;
      pidRight.deadband_reverse = msg->rightReverse;

      Serial.print(F("  Applied: pidLeft.fwd=")); Serial.print(pidLeft.deadband_forward);
      Serial.print(F(", pidLeft.rev=")); Serial.print(pidLeft.deadband_reverse);
      Serial.print(F(", pidRight.fwd=")); Serial.print(pidRight.deadband_forward);
      Serial.print(F(", pidRight.rev=")); Serial.println(pidRight.deadband_reverse);

      bool saved = saveConfig();
      Serial.println(F("Deadband updated and saved to EEPROM"));

      sendAck(type, saved ? 0 : RobotLink::ERR_EEPROM_WRITE_FAILED);
      break;
    }

    case RobotLink::MSG_SET_HEADING_HOLD_KP: {
      if (len != sizeof(RobotLink::SetHeadingHoldKpPayload)) {
        sendNack(type, RobotLink::ERR_INVALID_PAYLOAD);
        break;
      }

      RobotLink::SetHeadingHoldKpPayload* msg = (RobotLink::SetHeadingHoldKpPayload*)payload;
      headingHoldKp = msg->Kp;

      bool saved = saveConfig();
      Serial.print(F("Heading Hold Kp updated: "));
      Serial.println(headingHoldKp, 2);

      sendAck(type, saved ? 0 : RobotLink::ERR_EEPROM_WRITE_FAILED);
      break;
    }

    case RobotLink::MSG_SET_ROBOT_PARAMS: {
      if (len != sizeof(RobotLink::SetRobotParamsPayload)) {
        sendNack(type, RobotLink::ERR_INVALID_PAYLOAD);
        break;
      }

      RobotLink::SetRobotParamsPayload* msg = (RobotLink::SetRobotParamsPayload*)payload;

      WHEEL_DIAMETER = msg->wheelDiameter;
      WHEELBASE = msg->wheelbase;
      TICKS_PER_REV = msg->ticksPerRev;
      METERS_PER_TICK = (PI * WHEEL_DIAMETER) / TICKS_PER_REV;

      bool saved = saveConfig();

      Serial.print(F("Robot params updated: wheel_diam="));
      Serial.print(WHEEL_DIAMETER * 1000, 1);
      Serial.print(F("mm, wheelbase="));
      Serial.print(WHEELBASE * 1000, 1);
      Serial.print(F("mm, ticks/rev="));
      Serial.println(TICKS_PER_REV, 0);

      sendAck(type, saved ? 0 : RobotLink::ERR_EEPROM_WRITE_FAILED);
      break;
    }

    case RobotLink::MSG_ENABLE_STREAM: {
      if (len != sizeof(RobotLink::EnableStreamPayload)) break;

      RobotLink::EnableStreamPayload* msg = (RobotLink::EnableStreamPayload*)payload;
      streamEnabled = msg->enable != 0;
      streamInterval = msg->intervalMs;
      break;
    }

    case RobotLink::MSG_ZERO_ENCODERS: {
      encoderLeft.reset();
      encoderRight.reset();

      prevEncoderLeft = 0;
      prevEncoderRight = 0;
      prevOdomEncoderLeft = 0;
      prevOdomEncoderRight = 0;

      pose_x_mm = 0;
      pose_y_mm = 0;
      pose_th_mrad = 0;

      break;
    }

    case RobotLink::MSG_STOP: {
      stopMotors();
      break;
    }

    case RobotLink::MSG_RESET_POSE: {
      pose_x_mm = 0.0f;
      pose_y_mm = 0.0f;
      pose_th_mrad = 0.0f;

      prevOdomEncoderLeft = encoderLeft.getCount();
      prevOdomEncoderRight = encoderRight.getCount();

      Serial.println(F("Pose reset to origin"));
      break;
    }

    case RobotLink::MSG_SAVE_CONFIG: {
      bool saved = saveConfig();
      sendAck(type, saved ? 0 : RobotLink::ERR_EEPROM_WRITE_FAILED);
      break;
    }

    case RobotLink::MSG_LOAD_CONFIG: {
      loadConfig();
      break;
    }

    case RobotLink::MSG_GET_CONFIG: {
      RobotLink::ConfigPayload cfg;
      cfg.wheelDiameter = WHEEL_DIAMETER;
      cfg.wheelbase = WHEELBASE;
      cfg.ticksPerRev = TICKS_PER_REV;
      cfg.invertLeft = 0;
      cfg.invertRight = 1;
      cfg.balanceEnable = 0;
      cfg.reserved = 0;
      cfg.balanceGain = 0;

      robotLink->sendStruct(RobotLink::MSG_CONFIG_RESP, cfg);
      break;
    }

    case RobotLink::MSG_STATUS: {
      uint8_t payloadOut[44];
      uint8_t offset = 0;

      auto writeFloat = [&payloadOut, &offset](float value) {
        memcpy(&payloadOut[offset], &value, 4);
        offset += 4;
      };

      writeFloat(pidLeft.Kp);
      writeFloat(pidLeft.Ki);
      writeFloat(pidLeft.Kd);

      writeFloat(pidLeft.deadband_forward);
      writeFloat(pidLeft.deadband_reverse);
      writeFloat(pidRight.deadband_forward);
      writeFloat(pidRight.deadband_reverse);

      payloadOut[offset++] = 1;
      payloadOut[offset++] = streamEnabled ? 1 : 0;

      RobotLink::Link::wr_u16_le(&payloadOut[offset], streamInterval);
      offset += 2;

      RobotLink::Link::wr_u32_le(&payloadOut[offset], 0);
      offset += 4;
      RobotLink::Link::wr_u32_le(&payloadOut[offset], 0);
      offset += 4;

      RobotLink::Link::wr_u32_le(&payloadOut[offset], millis() / 1000);
      offset += 4;

      robotLink->sendFrame(RobotLink::MSG_STATUS, payloadOut, 44);
      break;
    }

    case RobotLink::MSG_STATUS_EXTENDED: {
      RobotLink::StatusExtendedPayload status;

      status.leftKp = pidLeft.Kp;
      status.leftKi = pidLeft.Ki;
      status.leftKd = pidLeft.Kd;

      status.rightKp = pidRight.Kp;
      status.rightKi = pidRight.Ki;
      status.rightKd = pidRight.Kd;

      status.deadband[0] = pidLeft.deadband_forward;
      status.deadband[1] = pidLeft.deadband_reverse;
      status.deadband[2] = pidRight.deadband_forward;
      status.deadband[3] = pidRight.deadband_reverse;

      status.headingHoldKp = headingHoldKp;

      status.pidEnabled = 1;
      status.streamEnabled = streamEnabled ? 1 : 0;

      status.streamInterval = streamInterval;

      status.framesReceived = 0;
      status.framesSent = 0;
      status.uptime = millis() / 1000;

      robotLink->sendStruct(RobotLink::MSG_STATUS_EXTENDED, status);
      break;
    }

    case RobotLink::MSG_PING: {
      robotLink->sendFrame(RobotLink::MSG_PONG, payload, len);
      break;
    }
  }
}

// ============================================================
// Odometry Frame Send
// ============================================================
void sendOdometry() {
  // getCount()/getErrorCount() already handle atomicity internally
  int32_t encL = encoderLeft.getCount();
  int32_t encR = encoderRight.getCount();

  uint32_t errL = encoderLeft.getErrorCount();
  uint32_t errR = encoderRight.getErrorCount();

  static int32_t lastSentL = 0;
  static int32_t lastSentR = 0;

  int32_t deltaL = encL - lastSentL;
  int32_t deltaR = encR - lastSentR;

  lastSentL = encL;
  lastSentR = encR;

  uint8_t payload[26];
  uint32_t t_ms = millis();

  int32_t pose_x_mm_int = (int32_t)pose_x_mm;
  int32_t pose_y_mm_int = (int32_t)pose_y_mm;
  int16_t pose_th_mrad_int = (int16_t)pose_th_mrad;

  RobotLink::Link::wr_u32_le(&payload[0], t_ms);
  RobotLink::Link::wr_i32_le(&payload[4], deltaL);
  RobotLink::Link::wr_i32_le(&payload[8], deltaR);
  RobotLink::Link::wr_i32_le(&payload[12], pose_x_mm_int);
  RobotLink::Link::wr_i32_le(&payload[16], pose_y_mm_int);
  RobotLink::Link::wr_i16_le(&payload[20], pose_th_mrad_int);
  RobotLink::Link::wr_u16_le(&payload[22], (uint16_t)(errL & 0xFFFF));
  RobotLink::Link::wr_u16_le(&payload[24], (uint16_t)(errR & 0xFFFF));

  robotLink->sendFrame(RobotLink::MSG_ODOM, payload, 26);
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println(F("\n=== Arduino Motor Control Firmware ==="));
  Serial.println(F("Version: 1.1"));
  Serial.println(F("Debug enabled on USB Serial (115200)"));
  Serial.println(F("RobotLink on SoftwareSerial A0/A1 (9600)"));
  Serial.println();

  softSerial.begin(9600);
  Serial.println(F("SoftwareSerial initialized at 9600 baud"));

  pinMode(PIN_M1_INA, OUTPUT);
  pinMode(PIN_M1_INB, OUTPUT);
  pinMode(PIN_M1_PWM, OUTPUT);
  pinMode(PIN_M2_INA, OUTPUT);
  pinMode(PIN_M2_INB, OUTPUT);
  pinMode(PIN_M2_PWM, OUTPUT);

  // Encoders: using external 4.7k pull-ups
  pinMode(PIN_ENC_L_A, INPUT);
  pinMode(PIN_ENC_L_B, INPUT);
  pinMode(PIN_ENC_R_A, INPUT);
  pinMode(PIN_ENC_R_B, INPUT);

  encoderLeft.begin(false);   // false = external pullups
  encoderRight.begin(false);

  encoderLeft.setMinPulseUs(10);
  encoderRight.setMinPulseUs(10);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncoderRight, CHANGE);

  motorLeft.begin();
  motorRight.begin();

  // Default tuned PID parameters (can be overwritten by EEPROM load)
  pidLeft.Kp = 50.0f;
  pidLeft.Ki = 60.0f;
  pidLeft.Kd = 0.0f;
  pidLeft.deadband_forward = 50.0f;
  pidLeft.deadband_reverse = 90.0f;
  pidLeft.filter_alpha = 0.3f;
  pidLeft.output_min = -255.0f;
  pidLeft.output_max = 255.0f;

  pidRight.Kp = 50.0f;
  pidRight.Ki = 20.0f;
  pidRight.Kd = 0.0f;
  pidRight.deadband_forward = 35.0f;
  pidRight.deadband_reverse = 35.0f;
  pidRight.filter_alpha = 0.3f;
  pidRight.output_min = -255.0f;
  pidRight.output_max = 255.0f;

  stopMotors();

  Serial.println(F("Loading configuration from EEPROM..."));
  if (loadConfig()) {
    Serial.println(F("✓ Configuration loaded from EEPROM"));
  } else {
    Serial.println(F("✗ EEPROM invalid or first boot - using defaults"));
    saveConfig();
    Serial.println(F("✓ Default configuration saved to EEPROM"));
  }

  Serial.println(F("\n--- Current Configuration ---"));
  Serial.print(F("PID: Kp=")); Serial.print(pidLeft.Kp);
  Serial.print(F(", Ki=")); Serial.print(pidLeft.Ki);
  Serial.print(F(", Kd=")); Serial.println(pidLeft.Kd);

  Serial.print(F("Deadband Left:  Fwd=")); Serial.print(pidLeft.deadband_forward);
  Serial.print(F(", Rev=")); Serial.println(pidLeft.deadband_reverse);

  Serial.print(F("Deadband Right: Fwd=")); Serial.print(pidRight.deadband_forward);
  Serial.print(F(", Rev=")); Serial.println(pidRight.deadband_reverse);
  Serial.println();

  RobotLink::Config cfg;
  cfg.maxPayload = 64;
  cfg.rejectOversize = true;
  robotLink = new RobotLink::Link(softSerial, cfg);
  Serial.println(F("RobotLink protocol initialized"));

  Serial.println(F("\n=== Setup Complete ==="));
  Serial.println(F("Ready for commands from ESP32"));
  Serial.println();

  lastControlUpdate = millis();
  lastOdomSend = millis();
}

// ============================================================
// Main Loop
// ============================================================
void loop() {
  unsigned long now = millis();

  robotLink->poll(handleFrame);

  if (now - lastControlUpdate >= CONTROL_INTERVAL) {
    float dt = (now - lastControlUpdate) / 1000.0f;
    lastControlUpdate = now;

    updateControl(dt);
    updateOdometry(dt);
  }

  if (streamEnabled && (now - lastOdomSend >= streamInterval)) {
    lastOdomSend = now;
    sendOdometry();
  }
}
