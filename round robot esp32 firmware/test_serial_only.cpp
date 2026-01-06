#include <Arduino.h>
#include "RobotLink.h"
#include "RobotLinkMessages.h"

// Arduino Serial Connection
#define ARDUINO_RX_PIN 26  // ESP32 RX <- Arduino TX
#define ARDUINO_TX_PIN 27  // ESP32 TX -> Arduino RX
#define ARDUINO_BAUD 9600

HardwareSerial arduinoSerial(1);
RobotLink::Link* arduinoLink = nullptr;

uint32_t lastEnableStream = 0;
uint32_t frameCount = 0;
uint32_t motorStartTime = 0;
bool motorsRunning = false;
bool motorsStarted = false;
int16_t maxDeltaL = 0;
int16_t maxDeltaR = 0;

// Callback when frame received from Arduino
void onArduinoFrame(uint8_t type, const uint8_t* payload, uint8_t len) {
  frameCount++;

  if (type == RobotLink::MSG_ODOM && len == 18) {
    uint32_t t_ms = RobotLink::Link::rd_u32_le(&payload[0]);
    int16_t dL = RobotLink::Link::rd_i16_le(&payload[4]);
    int16_t dR = RobotLink::Link::rd_i16_le(&payload[6]);
    int32_t x_mm = RobotLink::Link::rd_i32_le(&payload[8]);
    int32_t y_mm = RobotLink::Link::rd_i32_le(&payload[12]);
    int16_t th_mrad = RobotLink::Link::rd_i16_le(&payload[16]);

    // Track max deltas for motion detection
    if (abs(dL) > abs(maxDeltaL)) maxDeltaL = dL;
    if (abs(dR) > abs(maxDeltaR)) maxDeltaR = dR;

    // Print first few and motion events
    if (frameCount <= 5 || (motorsRunning && (abs(dL) > 5 || abs(dR) > 5))) {
      Serial.printf("[RX #%lu] Type=0x%02X ODOM: t=%lu dL=%d dR=%d x=%d y=%d th=%d\n",
                    frameCount, type, t_ms, dL, dR, x_mm, y_mm, th_mrad);
    }
  } else {
    Serial.printf("[RX #%lu] Type=0x%02X Len=%d\n", frameCount, type, len);
  }
}

void setup() {
  // USB Serial for debugging
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n===========================================");
  Serial.println("ESP32 <-> Arduino Serial Test (9600 baud)");
  Serial.println("===========================================\n");

  // Initialize Serial1 for Arduino
  arduinoSerial.begin(ARDUINO_BAUD, SERIAL_8N1, ARDUINO_RX_PIN, ARDUINO_TX_PIN);
  Serial.printf("Serial1 initialized: RX=GPIO%d TX=GPIO%d @ %d baud\n",
                ARDUINO_RX_PIN, ARDUINO_TX_PIN, ARDUINO_BAUD);

  // Create RobotLink instance
  RobotLink::Config cfg;
  cfg.maxPayload = 64;
  arduinoLink = new RobotLink::Link(arduinoSerial, cfg);

  Serial.println("RobotLink protocol ready\n");
  Serial.println("Waiting 2 seconds before sending ENABLE_STREAM...\n");

  delay(2000);

  // Send ENABLE_STREAM command
  RobotLink::EnableStreamPayload cmd;
  cmd.enable = 1;
  cmd.intervalMs = 50;  // 20 Hz

  if (arduinoLink->sendStruct(RobotLink::MSG_ENABLE_STREAM, cmd)) {
    Serial.println(">>> ENABLE_STREAM sent to Arduino (50ms interval)\n");
  } else {
    Serial.println("!!! Failed to send ENABLE_STREAM\n");
  }

  lastEnableStream = millis();
}

void loop() {
  uint32_t now = millis();

  // Process incoming frames from Arduino
  while (arduinoLink->poll(onArduinoFrame)) {
    // Frame processed in callback
  }

  // Motor test sequence
  if (!motorsStarted && frameCount >= 40) {
    // After receiving ~40 baseline odometry frames (2 seconds at 20 Hz)
    Serial.println("\n=== STARTING MOTOR TEST ===");
    Serial.printf("Baseline frames: %lu\n", frameCount);
    Serial.println("Sending SET_VEL: 0.5 m/s both wheels\n");

    // Send motor command: 0.5 m/s both wheels
    RobotLink::SetVelPayload velCmd;
    velCmd.velLeft = 0.5f;
    velCmd.velRight = 0.5f;
    if (arduinoLink->sendStruct(RobotLink::MSG_SET_VEL, velCmd)) {
      motorsStarted = true;
      motorsRunning = true;
      motorStartTime = now;
      Serial.println(">>> SET_VEL command sent\n");
    } else {
      Serial.println("!!! Failed to send SET_VEL\n");
    }
  }

  // Stop motors after 5 seconds
  if (motorsRunning && (now - motorStartTime >= 5000)) {
    Serial.println("\n=== STOPPING MOTORS ===");

    // Send STOP command (0.0 m/s both wheels)
    RobotLink::SetVelPayload stopCmd;
    stopCmd.velLeft = 0.0f;
    stopCmd.velRight = 0.0f;
    if (arduinoLink->sendStruct(RobotLink::MSG_SET_VEL, stopCmd)) {
      motorsRunning = false;
      Serial.println(">>> STOP command sent\n");
    } else {
      Serial.println("!!! Failed to send STOP\n");
    }

    // Print results
    delay(1000);  // Wait for final odometry frames

    Serial.println("\n======================================================================");
    Serial.println("TEST RESULTS");
    Serial.println("======================================================================");
    Serial.printf("Total frames received: %lu\n", frameCount);
    Serial.printf("Max encoder delta: L=%d R=%d ticks\n", maxDeltaL, maxDeltaR);
    Serial.printf("RobotLink stats:\n");
    Serial.printf("  Frames OK: %lu\n", arduinoLink->framesOk());
    Serial.printf("  Bad CRC: %lu\n", arduinoLink->framesBadCrc());
    Serial.printf("  Bad Len: %lu\n", arduinoLink->framesBadLen());
    Serial.printf("  Bytes dropped: %lu\n", arduinoLink->bytesDropped());

    bool motionDetected = (abs(maxDeltaL) > 5 || abs(maxDeltaR) > 5);

    if (motionDetected && frameCount > 80) {
      Serial.println("\n>>> SUCCESS! Motor control and odometry working!");
      Serial.println("  - ESP32 -> Arduino Serial1: WORKING");
      Serial.println("  - Motor commands: RECEIVED");
      Serial.println("  - Encoders: DETECTING MOTION");
      Serial.println("  - Odometry feedback: OPERATIONAL");
    } else if (frameCount > 80) {
      Serial.println("\n>>> Communication working, but no motor motion detected");
      Serial.println("  - Check motor power supply");
      Serial.println("  - Check motor wiring");
      Serial.println("  - Check encoder connections");
    } else {
      Serial.println("\n>>> Test incomplete - not enough odometry frames");
    }

    Serial.println("======================================================================\n");
  }

  delay(1);  // Small delay to prevent busy-waiting
}
