#include <Arduino.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "RobotLink.h"
#include "RobotLinkMessages.h"
#include <Encoder.h>
#include <MotorControl.h>
#include <PIDController.h>

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
// Robot Configuration
// ============================================================
const float WHEEL_DIAMETER = 0.0825f;  // meters (82.5mm, measured)
const float WHEELBASE = 0.244f;        // meters (244mm, measured physical wheelbase)
const float TICKS_PER_REV = 714.0f;    // encoder ticks per revolution (CHANGE mode, 2X decoding, calibrated)
const float METERS_PER_TICK = (PI * WHEEL_DIAMETER) / TICKS_PER_REV;

// PID Controller from library

// ============================================================
// Library Objects - Motors, Encoders, PID
// ============================================================
// Direction settings verified through motor_encoder_test.cpp
Encoder encoderLeft(PIN_ENC_L_A, PIN_ENC_L_B, true);    // Reversed
Encoder encoderRight(PIN_ENC_R_A, PIN_ENC_R_B, true);   // Reversed
MotorControl motorLeft(PIN_M1_INA, PIN_M1_INB, PIN_M1_PWM, true);   // Reversed
MotorControl motorRight(PIN_M2_INA, PIN_M2_INB, PIN_M2_PWM, false); // Normal

// ============================================================
// Global Variables
// ============================================================
// PID Controllers
PIDController pidLeft;
PIDController pidRight;

// Velocity targets (m/s)
float targetVelLeft = 0;
float targetVelRight = 0;

// Current velocities (m/s)
float currentVelLeft = 0;
float currentVelRight = 0;

// PWM outputs
int16_t pwmLeft = 0;
int16_t pwmRight = 0;

// Odometry pose
int32_t pose_x_mm = 0;
int32_t pose_y_mm = 0;
int16_t pose_th_mrad = 0;

// Previous encoder values for delta calculation
int32_t prevEncoderLeft = 0;
int32_t prevEncoderRight = 0;

// Timing
unsigned long lastControlUpdate = 0;
unsigned long lastOdomSend = 0;
const unsigned long CONTROL_INTERVAL = 50;  // 50ms = 20Hz
const unsigned long ODOM_INTERVAL = 50;     // 50ms = 20Hz

// Minimum velocity threshold (from motor calibration Story 1.6)
// Below this velocity, both motors stop to avoid unstable low-speed regime
// Lowered to 0.08 m/s to allow SLAM target speeds (0.15-0.20 m/s)
const float MIN_VELOCITY_THRESHOLD = 0.08f;  // m/s

// SoftwareSerial for ESP32 communication
SoftwareSerial softSerial(PIN_SOFT_RX, PIN_SOFT_TX);

// RobotLink protocol
RobotLink::Link* robotLink = nullptr;

// Configuration
bool streamEnabled = false;
uint16_t streamInterval = 50;  // ms

// ============================================================
// Encoder Interrupt Service Routines - 2X Decoding with XOR Logic
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
  setMotors(0, 0);
  pidLeft.reset();
  pidRight.reset();
}

// ============================================================
// Velocity Calculation
// ============================================================
void updateVelocities(float dt) {
  // Get current encoder counts atomically
  noInterrupts();
  int32_t encL = encoderLeft.getCount();
  int32_t encR = encoderRight.getCount();
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
// Odometry Update
// ============================================================
// Previous encoder values for odometry integration
static int32_t prevOdomEncoderLeft = 0;
static int32_t prevOdomEncoderRight = 0;

void updateOdometry(float dt) {
  // Get current encoder counts
  noInterrupts();
  int32_t encL = encoderLeft.getCount();
  int32_t encR = encoderRight.getCount();
  interrupts();

  // Calculate DELTA ticks since last odometry update
  int32_t deltaL = encL - prevOdomEncoderLeft;
  int32_t deltaR = encR - prevOdomEncoderRight;

  prevOdomEncoderLeft = encL;
  prevOdomEncoderRight = encR;

  // Calculate DELTA distances traveled by each wheel (mm)
  float deltaDistLeft = deltaL * METERS_PER_TICK * 1000.0f;
  float deltaDistRight = deltaR * METERS_PER_TICK * 1000.0f;

  // Calculate center distance and angle change
  float deltaDistCenter = (deltaDistLeft + deltaDistRight) / 2.0f;
  // Note: Both encoders are inverted in hardware, so we swap the subtraction order
  float deltaTheta = (deltaDistLeft - deltaDistRight) / (WHEELBASE * 1000.0f);  // radians

  // Current heading in radians
  float theta = pose_th_mrad / 1000.0f;

  // Update pose using INCREMENTAL differential drive kinematics
  // Apply rotation first, then translation in global frame
  pose_x_mm += (int32_t)(deltaDistCenter * cos(theta + deltaTheta / 2.0f));
  pose_y_mm += (int32_t)(deltaDistCenter * sin(theta + deltaTheta / 2.0f));
  pose_th_mrad += (int16_t)(deltaTheta * 1000.0f);

  // Wrap angle to [-pi, pi]
  while (pose_th_mrad > 3142) pose_th_mrad -= 6283;
  while (pose_th_mrad < -3142) pose_th_mrad += 6283;
}

// ============================================================
// PID Control Update
// ============================================================
void updateControl(float dt) {
  // Calculate velocities
  updateVelocities(dt);

  // Update PID controllers with minimum velocity threshold
  // Threshold at 0.08 m/s to avoid unstable low-speed regime
  float pwmLf = pidLeft.update(currentVelLeft, targetVelLeft, dt);
  float pwmRf = pidRight.update(currentVelRight, targetVelRight, dt);

  // Convert to int16 and apply
  int16_t pwmL = (int16_t)pwmLf;
  int16_t pwmR = (int16_t)pwmRf;

  // DEBUG: Print PID output
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 500 && (targetVelLeft != 0 || targetVelRight != 0)) {
    lastDebug = millis();
    Serial.print(F("PID: tgt["));
    Serial.print(targetVelLeft, 2);
    Serial.print(F(","));
    Serial.print(targetVelRight, 2);
    Serial.print(F("] cur["));
    Serial.print(currentVelLeft, 2);
    Serial.print(F(","));
    Serial.print(currentVelRight, 2);
    Serial.print(F("] pwm["));
    Serial.print(pwmL);
    Serial.print(F(","));
    Serial.print(pwmR);
    Serial.println(F("]"));
  }

  setMotors(pwmL, pwmR);
}

// ============================================================
// EEPROM Configuration Management
// ============================================================
const uint16_t EEPROM_MAGIC = 0xAB12;
const uint16_t EEPROM_VERSION = 1;

struct EEPROMConfig {
  uint16_t magic;
  uint16_t version;
  float Kp, Ki, Kd;
  float deadbandLeftFwd, deadbandLeftRev;
  float deadbandRightFwd, deadbandRightRev;
  float wheelDiameter;
  float wheelbase;
  float ticksPerRev;
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

void saveConfig() {
  EEPROMConfig cfg;
  cfg.magic = EEPROM_MAGIC;
  cfg.version = EEPROM_VERSION;
  cfg.Kp = pidLeft.Kp;
  cfg.Ki = pidLeft.Ki;
  cfg.Kd = pidLeft.Kd;
  cfg.deadbandLeftFwd = pidLeft.deadband_forward;
  cfg.deadbandLeftRev = pidLeft.deadband_reverse;
  cfg.deadbandRightFwd = pidRight.deadband_forward;
  cfg.deadbandRightRev = pidRight.deadband_reverse;
  cfg.wheelDiameter = WHEEL_DIAMETER;
  cfg.wheelbase = WHEELBASE;
  cfg.ticksPerRev = TICKS_PER_REV;
  cfg.checksum = calcChecksum(cfg);

  EEPROM.put(0, cfg);
}

bool loadConfig() {
  EEPROMConfig cfg;
  EEPROM.get(0, cfg);

  // Validate magic and checksum
  if (cfg.magic != EEPROM_MAGIC || cfg.version != EEPROM_VERSION) {
    return false;
  }

  uint8_t expectedChecksum = calcChecksum(cfg);
  if (cfg.checksum != expectedChecksum) {
    return false;
  }

  // Load configuration
  pidLeft.Kp = pidRight.Kp = cfg.Kp;
  pidLeft.Ki = pidRight.Ki = cfg.Ki;
  pidLeft.Kd = pidRight.Kd = cfg.Kd;
  pidLeft.deadband_forward = cfg.deadbandLeftFwd;
  pidLeft.deadband_reverse = cfg.deadbandLeftRev;
  pidRight.deadband_forward = cfg.deadbandRightFwd;
  pidRight.deadband_reverse = cfg.deadbandRightRev;

  return true;
}

// ============================================================
// RobotLink Protocol Handlers
// ============================================================
void handleFrame(uint8_t type, const uint8_t* payload, uint8_t len) {
  switch (type) {
    case RobotLink::MSG_CMD_VEL: {
      // Original protocol: (v, w) -> (left, right) velocities
      if (len != 4) break;

      int16_t v_mm_s = RobotLink::Link::rd_i16_le(&payload[0]);
      int16_t w_mrad_s = RobotLink::Link::rd_i16_le(&payload[2]);

      // Convert to m/s
      float v = v_mm_s / 1000.0f;
      float w = w_mrad_s / 1000.0f;

      // Differential drive kinematics
      float wheelbase_half = WHEELBASE / 2.0f;
      targetVelLeft = v - w * wheelbase_half;
      targetVelRight = v + w * wheelbase_half;
      break;
    }

    case RobotLink::MSG_SET_VEL: {
      // Direct velocity control
      if (len != sizeof(RobotLink::SetVelPayload)) break;

      RobotLink::SetVelPayload* msg = (RobotLink::SetVelPayload*)payload;
      targetVelLeft = msg->velLeft;
      targetVelRight = msg->velRight;

      // DEBUG: Log all velocity commands
      Serial.print(F("SET_VEL: L="));
      Serial.print(targetVelLeft, 2);
      Serial.print(F(" R="));
      Serial.println(targetVelRight, 2);

      if (targetVelLeft == 0.0f && targetVelRight == 0.0f) {
        Serial.println(F("  --> STOP command"));
      }

      break;
    }

    case RobotLink::MSG_SET_PID: {
      if (len != sizeof(RobotLink::SetPidPayload)) break;

      RobotLink::SetPidPayload* msg = (RobotLink::SetPidPayload*)payload;
      pidLeft.Kp = pidRight.Kp = msg->Kp;
      pidLeft.Ki = pidRight.Ki = msg->Ki;
      pidLeft.Kd = pidRight.Kd = msg->Kd;

      // Auto-save to EEPROM when updated from server
      saveConfig();
      Serial.println(F("PID updated and saved to EEPROM"));
      break;
    }

    case RobotLink::MSG_SET_DEADBAND: {
      if (len != sizeof(RobotLink::SetDeadbandPayload)) break;

      RobotLink::SetDeadbandPayload* msg = (RobotLink::SetDeadbandPayload*)payload;
      pidLeft.deadband_forward = msg->leftForward;
      pidLeft.deadband_reverse = msg->leftReverse;
      pidRight.deadband_forward = msg->rightForward;
      pidRight.deadband_reverse = msg->rightReverse;

      // Auto-save to EEPROM when updated from server
      saveConfig();
      Serial.println(F("Deadband updated and saved to EEPROM"));
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
      noInterrupts();
      encoderLeft.reset();
      encoderRight.reset();
      interrupts();

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
      // Reset pose to origin without zeroing encoders
      // This allows resetting pose while preserving encoder continuity
      pose_x_mm = 0;
      pose_y_mm = 0;
      pose_th_mrad = 0;
      prevOdomEncoderLeft = encoderLeft.getCount();
      prevOdomEncoderRight = encoderRight.getCount();
      Serial.println(F("Pose reset to origin"));
      break;
    }

    case RobotLink::MSG_SAVE_CONFIG: {
      saveConfig();
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
      // Send current robot status: PID, deadband, stream settings, uptime
      // Payload format: <fffFFFFBBHIII> (44 bytes)
      // 3 × float: Kp, Ki, Kd
      // 4 × float: deadband (left_fwd, left_rev, right_fwd, right_rev)
      // 2 × uint8: pid_enabled, stream_enabled
      // 1 × uint16: stream_interval
      // 3 × uint32: frames_received, frames_sent, uptime

      uint8_t payload[44];
      uint8_t offset = 0;

      // Helper to write float32 in little-endian
      auto writeFloat = [&payload, &offset](float value) {
        memcpy(&payload[offset], &value, 4);
        offset += 4;
      };

      // PID parameters (12 bytes)
      writeFloat(pidLeft.Kp);
      writeFloat(pidLeft.Ki);
      writeFloat(pidLeft.Kd);

      // Deadband values (16 bytes)
      writeFloat(pidLeft.deadband_forward);
      writeFloat(pidLeft.deadband_reverse);
      writeFloat(pidRight.deadband_forward);
      writeFloat(pidRight.deadband_reverse);

      // Flags (2 bytes)
      payload[offset++] = 1;  // pid_enabled (always on in current firmware)
      payload[offset++] = streamEnabled ? 1 : 0;

      // Stream interval (2 bytes)
      RobotLink::Link::wr_u16_le(&payload[offset], streamInterval);
      offset += 2;

      // Frame statistics (12 bytes) - not tracked yet, send zeros
      RobotLink::Link::wr_u32_le(&payload[offset], 0);  // frames_received
      offset += 4;
      RobotLink::Link::wr_u32_le(&payload[offset], 0);  // frames_sent
      offset += 4;

      // Uptime in seconds (4 bytes)
      RobotLink::Link::wr_u32_le(&payload[offset], millis() / 1000);
      offset += 4;

      robotLink->sendFrame(RobotLink::MSG_STATUS, payload, 44);
      break;
    }

    case RobotLink::MSG_PING: {
      // Echo back as pong
      robotLink->sendFrame(RobotLink::MSG_PONG, payload, len);
      break;
    }
  }
}

void sendOdometry() {
  // Get current encoder counts atomically
  noInterrupts();
  int32_t encL = encoderLeft.getCount();
  int32_t encR = encoderRight.getCount();
  interrupts();

  // Calculate delta ticks since last send
  static int32_t lastSentL = 0;
  static int32_t lastSentR = 0;

  int32_t deltaL = encL - lastSentL;  // Fixed: int32_t instead of int16_t to prevent overflow
  int32_t deltaR = encR - lastSentR;

  lastSentL = encL;
  lastSentR = encR;

  // Build ODOM message (updated protocol format with int32 deltas)
  uint8_t payload[22];  // Increased from 18 to 22 bytes
  uint32_t t_ms = millis();

  RobotLink::Link::wr_u32_le(&payload[0], t_ms);        // 4 bytes: timestamp
  RobotLink::Link::wr_i32_le(&payload[4], deltaL);      // 4 bytes: delta_left (was int16 @ offset 4)
  RobotLink::Link::wr_i32_le(&payload[8], deltaR);      // 4 bytes: delta_right (was int16 @ offset 6)
  RobotLink::Link::wr_i32_le(&payload[12], pose_x_mm);  // 4 bytes: x_mm (was @ offset 8)
  RobotLink::Link::wr_i32_le(&payload[16], pose_y_mm);  // 4 bytes: y_mm (was @ offset 12)
  RobotLink::Link::wr_i16_le(&payload[20], pose_th_mrad); // 2 bytes: theta_mrad (was @ offset 16)

  robotLink->sendFrame(RobotLink::MSG_ODOM, payload, 22);  // Updated size
}

// ============================================================
// Setup
// ============================================================
void setup() {
  // Initialize USB serial for debugging
  Serial.begin(115200);
  delay(100);

  Serial.println(F("\n=== Arduino Motor Control Firmware ==="));
  Serial.println(F("Version: 1.1"));
  Serial.println(F("Debug enabled on USB Serial (115200)"));
  Serial.println(F("RobotLink on SoftwareSerial A0/A1 (9600)"));
  Serial.println();

  // Initialize SoftwareSerial for ESP32 communication
  softSerial.begin(9600);
  Serial.println(F("SoftwareSerial initialized at 9600 baud"));

  // Initialize motor pins
  pinMode(PIN_M1_INA, OUTPUT);
  pinMode(PIN_M1_INB, OUTPUT);
  pinMode(PIN_M1_PWM, OUTPUT);
  pinMode(PIN_M2_INA, OUTPUT);
  pinMode(PIN_M2_INB, OUTPUT);
  pinMode(PIN_M2_PWM, OUTPUT);

  // Initialize encoder pins
  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);

  // Attach interrupts for 2X quadrature decoding with XOR logic
  // CHANGE mode triggers on both RISING and FALLING edges of channel A
  // Read both A and B in ISR to determine direction correctly
  // Arduino Uno: pins 2,3 support interrupts
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncoderRight, CHANGE);

  // Initialize motors
  motorLeft.begin();
  motorRight.begin();

  // Apply TUNED PID parameters (from systematic tuning)
  // Left motor: Higher Ki and deadband due to higher static friction
  pidLeft.Kp = 50.0f;
  pidLeft.Ki = 60.0f;  // 3x higher than right - compensates for deadband overshoot
  pidLeft.Kd = 0.0f;
  pidLeft.deadband_forward = 50.0f;
  pidLeft.deadband_reverse = 50.0f;
  pidLeft.filter_alpha = 0.3f;
  pidLeft.output_min = -255.0f;
  pidLeft.output_max = 255.0f;

  // Right motor: Standard settings work well
  pidRight.Kp = 50.0f;
  pidRight.Ki = 20.0f;
  pidRight.Kd = 0.0f;
  pidRight.deadband_forward = 35.0f;
  pidRight.deadband_reverse = 35.0f;
  pidRight.filter_alpha = 0.3f;
  pidRight.output_min = -255.0f;
  pidRight.output_max = 255.0f;

  // Initialize motors to stopped state
  stopMotors();

  // Load configuration from EEPROM
  Serial.println(F("Loading configuration from EEPROM..."));
  if (loadConfig()) {
    Serial.println(F("✓ Configuration loaded from EEPROM"));
  } else {
    Serial.println(F("✗ EEPROM invalid or first boot - using defaults"));
    saveConfig();
    Serial.println(F("✓ Default configuration saved to EEPROM"));
  }

  // Override left motor deadband - left motor has higher friction
  // 44 - lower to allow better speed control (left motor efficient when running)
  pidLeft.deadband_forward = 44.0f;
  pidLeft.deadband_reverse = 44.0f;
  Serial.println(F("✓ Left motor deadband set to 44 (better control)"));

  // Override Kd for startup boost (D-term on error provides initial kick)
  pidLeft.Kd = pidRight.Kd = 2.0f;
  Serial.println(F("✓ Kd set to 2.0 for D-term startup boost"));

  // Save updated configuration to EEPROM
  saveConfig();
  Serial.println(F("✓ Updated configuration saved to EEPROM"));

  // Print loaded configuration
  Serial.println(F("\n--- Current Configuration ---"));
  Serial.print(F("PID: Kp=")); Serial.print(pidLeft.Kp);
  Serial.print(F(", Ki=")); Serial.print(pidLeft.Ki);
  Serial.print(F(", Kd=")); Serial.println(pidLeft.Kd);

  Serial.print(F("Deadband Left:  Fwd=")); Serial.print(pidLeft.deadband_forward);
  Serial.print(F(", Rev=")); Serial.println(pidLeft.deadband_reverse);

  Serial.print(F("Deadband Right: Fwd=")); Serial.print(pidRight.deadband_forward);
  Serial.print(F(", Rev=")); Serial.println(pidRight.deadband_reverse);
  Serial.println();

  // Initialize RobotLink protocol on SoftwareSerial
  RobotLink::Config cfg;
  cfg.maxPayload = 64;
  cfg.rejectOversize = true;
  robotLink = new RobotLink::Link(softSerial, cfg);
  Serial.println(F("RobotLink protocol initialized"));

  Serial.println(F("\n=== Setup Complete ==="));
  Serial.println(F("Ready for commands from ESP32"));
  Serial.println();

  // Initialize timing
  lastControlUpdate = millis();
  lastOdomSend = millis();
}

// ============================================================
// Main Loop
// ============================================================
void loop() {
  unsigned long now = millis();

  // Poll for incoming messages
  robotLink->poll(handleFrame);

  // Control loop update (20 Hz)
  if (now - lastControlUpdate >= CONTROL_INTERVAL) {
    float dt = (now - lastControlUpdate) / 1000.0f;
    lastControlUpdate = now;

    // Update control and odometry
    updateControl(dt);
    updateOdometry(dt);
  }

  // Send odometry if streaming enabled
  if (streamEnabled && (now - lastOdomSend >= streamInterval)) {
    lastOdomSend = now;
    sendOdometry();
  }
}
