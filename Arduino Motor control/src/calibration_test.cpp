/**
 * Calibration Test - Measure TICKS_PER_REV
 *
 * Runs one motor at a time so user can count wheel rotations
 */

#include <Arduino.h>

// Motor Pins (H-Bridge control)
const uint8_t PIN_M1_INA = 7;
const uint8_t PIN_M1_INB = 8;
const uint8_t PIN_M1_PWM = 5;
const uint8_t PIN_M2_INA = 4;
const uint8_t PIN_M2_INB = 9;
const uint8_t PIN_M2_PWM = 6;

// Encoder Pins
const uint8_t PIN_ENC_L_A = 2;
const uint8_t PIN_ENC_L_B = 10;
const uint8_t PIN_ENC_R_A = 3;
const uint8_t PIN_ENC_R_B = 11;

volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;

// Encoder ISRs - 2X decoding with XOR logic
void isrEncoderLeft() {
  // Read both A and B to determine direction
  bool a = digitalRead(PIN_ENC_L_A);
  bool b = digitalRead(PIN_ENC_L_B);

  // XOR gives direction: (A rising & B high) or (A falling & B low) = forward
  if (a == b) {
    encoderLeftCount++;
  } else {
    encoderLeftCount--;
  }
}

void isrEncoderRight() {
  // Read both A and B to determine direction
  bool a = digitalRead(PIN_ENC_R_A);
  bool b = digitalRead(PIN_ENC_R_B);

  // XOR gives direction: (A rising & B high) or (A falling & B low) = forward
  if (a == b) {
    encoderRightCount++;
  } else {
    encoderRightCount--;
  }
}

// Motor Control
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

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("========================================");
  Serial.println("  TICKS_PER_REV Calibration Test");
  Serial.println("========================================");
  Serial.println();
  Serial.println("This test will run each motor at PWM 80");
  Serial.println("for 30 seconds. Count the wheel rotations!");
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

  // Attach interrupts - CHANGE mode for 2X decoding (both edges)
  // Triggers on both rising and falling edges of channel A
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncoderRight, CHANGE);

  // Stop motors
  setMotor(PIN_M1_INA, PIN_M1_INB, PIN_M1_PWM, 0);
  setMotor(PIN_M2_INA, PIN_M2_INB, PIN_M2_PWM, 0);

  Serial.println("Hardware initialized");
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  'l' = LEFT motor FORWARD");
  Serial.println("  'L' = LEFT motor BACKWARD");
  Serial.println("  'r' = RIGHT motor FORWARD");
  Serial.println("  'R' = RIGHT motor BACKWARD");
  Serial.println("========================================");
  Serial.println();
}

void runTest(bool isLeft, bool isForward) {
  const char* motorName = isLeft ? "LEFT" : "RIGHT";
  const char* direction = isForward ? "FORWARD" : "BACKWARD";

  Serial.println("========================================");
  Serial.print("Testing ");
  Serial.print(motorName);
  Serial.print(" motor ");
  Serial.println(direction);
  Serial.println("========================================");
  Serial.println("Get ready to count wheel rotations!");
  Serial.println("Starting in 3 seconds...");

  delay(3000);

  // Reset encoder
  noInterrupts();
  if (isLeft) {
    encoderLeftCount = 0;
  } else {
    encoderRightCount = 0;
  }
  interrupts();

  Serial.println();
  Serial.println(">>> MOTOR RUNNING - COUNT NOW!");
  Serial.println();

  // Run motor at PWM 80 for 30 seconds
  // Left motor is inverted in hardware
  int16_t pwm = isForward ? 80 : -80;
  if (isLeft) {
    setMotor(PIN_M1_INA, PIN_M1_INB, PIN_M1_PWM, -pwm);  // Left inverted
  } else {
    setMotor(PIN_M2_INA, PIN_M2_INB, PIN_M2_PWM, pwm);
  }

  // Print countdown and encoder counts
  for (int i = 30; i > 0; i--) {
    noInterrupts();
    long count = isLeft ? encoderLeftCount : encoderRightCount;
    interrupts();

    Serial.print("Time remaining: ");
    Serial.print(i);
    Serial.print("s  |  Encoder counts: ");
    Serial.println(count);
    delay(1000);
  }

  // Stop motor
  if (isLeft) {
    setMotor(PIN_M1_INA, PIN_M1_INB, PIN_M1_PWM, 0);
  } else {
    setMotor(PIN_M2_INA, PIN_M2_INB, PIN_M2_PWM, 0);
  }

  // Get final count
  noInterrupts();
  long finalCount = isLeft ? encoderLeftCount : encoderRightCount;
  interrupts();

  Serial.println();
  Serial.println(">>> MOTOR STOPPED");
  Serial.println();
  Serial.println("========================================");
  Serial.println("RESULTS:");
  Serial.println("========================================");
  Serial.print("Motor: ");
  Serial.println(motorName);
  Serial.print("Total encoder counts: ");
  Serial.println(finalCount);
  Serial.println();
  Serial.println("Now enter the number of rotations you counted.");
  Serial.println("Formula: TICKS_PER_REV = counts / rotations");
  Serial.println("========================================");
  Serial.println();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'l') {
      runTest(true, true);  // Left motor, forward
    } else if (cmd == 'L') {
      runTest(true, false);  // Left motor, backward
    } else if (cmd == 'r') {
      runTest(false, true);  // Right motor, forward
    } else if (cmd == 'R') {
      runTest(false, false);  // Right motor, backward
    } else if (cmd != '\n' && cmd != '\r') {
      Serial.print("Unknown command: ");
      Serial.println(cmd);
      Serial.println("Commands: l=left fwd, L=left back, r=right fwd, R=right back");
    }
  }
}
