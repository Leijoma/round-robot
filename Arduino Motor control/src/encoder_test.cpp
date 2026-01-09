/**
 * Encoder Test Program - Direct USB Testing
 *
 * Simple program to test encoder functionality without ESP32/RobotLink
 * Connects directly via USB Serial
 *
 * Commands over Serial (9600 baud):
 * - 'f' = Forward at medium speed
 * - 'b' = Backward at medium speed
 * - 's' = Stop motors
 * - 'r' = Reset encoder counts to zero
 * - 't' = Toggle auto-print (every second)
 */

#include <Arduino.h>

// ============================================================
// Pin Definitions
// ============================================================
// Motor Pins (H-Bridge control - same as main.cpp)
// Motor 1 (Left)
const uint8_t PIN_M1_INA = 7;
const uint8_t PIN_M1_INB = 8;
const uint8_t PIN_M1_PWM = 5;

// Motor 2 (Right)
const uint8_t PIN_M2_INA = 4;
const uint8_t PIN_M2_INB = 9;
const uint8_t PIN_M2_PWM = 6;

// Encoder Pins (same as main.cpp)
const uint8_t PIN_ENC_L_A = 2;  // INT0
const uint8_t PIN_ENC_L_B = 10;
const uint8_t PIN_ENC_R_A = 3;  // INT1
const uint8_t PIN_ENC_R_B = 11;

// ============================================================
// Global Variables
// ============================================================
volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;

bool autoPrint = true;
unsigned long lastPrintTime = 0;

// ============================================================
// Encoder ISR - Simple and Reliable
// ============================================================
void isrEncoderLeft() {
  // 2X decoding: Handle RISING and FALLING edges differently
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
  // 2X decoding: Handle RISING and FALLING edges differently
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

// ============================================================
// Motor Control (H-Bridge - same as main.cpp)
// ============================================================
void setMotor(uint8_t pinINA, uint8_t pinINB, uint8_t pinPWM, int16_t pwm) {
  if (pwm > 0) {
    // Forward
    digitalWrite(pinINA, HIGH);
    digitalWrite(pinINB, LOW);
    analogWrite(pinPWM, constrain(pwm, 0, 255));
  } else if (pwm < 0) {
    // Backward
    digitalWrite(pinINA, LOW);
    digitalWrite(pinINB, HIGH);
    analogWrite(pinPWM, constrain(-pwm, 0, 255));
  } else {
    // Stop
    digitalWrite(pinINA, LOW);
    digitalWrite(pinINB, LOW);
    analogWrite(pinPWM, 0);
  }
}

void setMotorLeft(int pwm) {
  // Left motor is inverted in hardware wiring (same as main.cpp)
  setMotor(PIN_M1_INA, PIN_M1_INB, PIN_M1_PWM, -pwm);
}

void setMotorRight(int pwm) {
  // Right motor not inverted
  setMotor(PIN_M2_INA, PIN_M2_INB, PIN_M2_PWM, pwm);
}

void stopMotors() {
  setMotorLeft(0);
  setMotorRight(0);
}

// ============================================================
// Print Encoder Counts
// ============================================================
void printEncoders() {
  // Read atomically
  noInterrupts();
  long left = encoderLeftCount;
  long right = encoderRightCount;
  interrupts();

  Serial.print("Encoders: L=");
  Serial.print(left);
  Serial.print(" R=");
  Serial.print(right);
  Serial.print(" | Time=");
  Serial.print(millis() / 1000.0, 1);
  Serial.println("s");
}

// ============================================================
// Setup
// ============================================================
void setup() {
  // Initialize Serial
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  Serial.println("========================================");
  Serial.println("  Encoder Test Program v1.0");
  Serial.println("========================================");
  Serial.println();

  // Setup motor pins (H-Bridge)
  pinMode(PIN_M1_INA, OUTPUT);
  pinMode(PIN_M1_INB, OUTPUT);
  pinMode(PIN_M1_PWM, OUTPUT);
  pinMode(PIN_M2_INA, OUTPUT);
  pinMode(PIN_M2_INB, OUTPUT);
  pinMode(PIN_M2_PWM, OUTPUT);

  // Setup encoder pins with pullups
  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);

  // Attach interrupts - CHANGE mode for 2X decoding (double resolution)
  // Triggers on both rising and falling edges of channel A
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncoderRight, CHANGE);

  // Stop motors
  stopMotors();

  Serial.println("Hardware initialized");
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  f = Forward (PWM 150)");
  Serial.println("  b = Backward (PWM 150)");
  Serial.println("  s = Stop");
  Serial.println("  r = Reset encoder counts");
  Serial.println("  t = Toggle auto-print");
  Serial.println("  p = Print encoder counts now");
  Serial.println();
  Serial.println("Auto-print: ENABLED (every 1 second)");
  Serial.println("========================================");
  Serial.println();

  lastPrintTime = millis();
}

// ============================================================
// Main Loop
// ============================================================
void loop() {
  // Auto-print encoder counts every 100ms (10 Hz) for faster feedback
  if (autoPrint && (millis() - lastPrintTime >= 100)) {
    printEncoders();
    lastPrintTime = millis();
  }

  // Process serial commands
  if (Serial.available()) {
    char cmd = Serial.read();

    switch (cmd) {
      case 'f':
      case 'F':
        Serial.println(">>> FORWARD (PWM 150)");
        setMotorLeft(150);
        setMotorRight(150);
        break;

      case 'b':
      case 'B':
        Serial.println(">>> BACKWARD (PWM 150)");
        setMotorLeft(-150);
        setMotorRight(-150);
        break;

      case 's':
      case 'S':
        Serial.println(">>> STOP");
        stopMotors();
        break;

      case 'r':
      case 'R':
        Serial.println(">>> RESET ENCODERS");
        noInterrupts();
        encoderLeftCount = 0;
        encoderRightCount = 0;
        interrupts();
        printEncoders();
        break;

      case 't':
      case 'T':
        autoPrint = !autoPrint;
        Serial.print(">>> AUTO-PRINT: ");
        Serial.println(autoPrint ? "ENABLED" : "DISABLED");
        break;

      case 'p':
      case 'P':
        printEncoders();
        break;

      case '\n':
      case '\r':
        // Ignore newlines
        break;

      default:
        Serial.print("Unknown command: ");
        Serial.println(cmd);
        break;
    }
  }
}
