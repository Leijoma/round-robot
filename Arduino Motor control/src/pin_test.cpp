/**
 * Pin Test - Read raw encoder pin states while motors run
 *
 * This will show us if encoder pins 10/11 are receiving ANY signals
 */

#include <Arduino.h>

// Motor Pins
const uint8_t PIN_M1_INA = 7;
const uint8_t PIN_M1_INB = 8;
const uint8_t PIN_M1_PWM = 5;
const uint8_t PIN_M2_INA = 4;
const uint8_t PIN_M2_INB = 9;
const uint8_t PIN_M2_PWM = 6;

// Encoder Pins to test
const uint8_t PIN_ENC_L_A = 2;
const uint8_t PIN_ENC_L_B = 10;
const uint8_t PIN_ENC_R_A = 3;
const uint8_t PIN_ENC_R_B = 11;

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
  Serial.println("  Pin State Test");
  Serial.println("========================================");
  Serial.println();

  // Setup motor pins
  pinMode(PIN_M1_INA, OUTPUT);
  pinMode(PIN_M1_INB, OUTPUT);
  pinMode(PIN_M1_PWM, OUTPUT);
  pinMode(PIN_M2_INA, OUTPUT);
  pinMode(PIN_M2_INB, OUTPUT);
  pinMode(PIN_M2_PWM, OUTPUT);

  // Setup encoder pins as inputs with pullups
  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);

  // Stop motors
  setMotor(PIN_M1_INA, PIN_M1_INB, PIN_M1_PWM, 0);
  setMotor(PIN_M2_INA, PIN_M2_INB, PIN_M2_PWM, 0);

  Serial.println("Motors stopped. Reading pin states...");
  Serial.println("Send 'f' to start motors forward at PWM 200");
  Serial.println("Send 's' to stop motors");
  Serial.println();
  Serial.println("Pin states (LA=Left A, LB=Left B, RA=Right A, RB=Right B):");
  Serial.println("========================================");
}

void loop() {
  static unsigned long lastPrint = 0;
  static bool motorsRunning = false;

  // Print pin states every 100ms
  if (millis() - lastPrint >= 100) {
    int la = digitalRead(PIN_ENC_L_A);
    int lb = digitalRead(PIN_ENC_L_B);
    int ra = digitalRead(PIN_ENC_R_A);
    int rb = digitalRead(PIN_ENC_R_B);

    Serial.print("LA=");
    Serial.print(la);
    Serial.print(" LB=");
    Serial.print(lb);
    Serial.print(" RA=");
    Serial.print(ra);
    Serial.print(" RB=");
    Serial.print(rb);
    Serial.print(motorsRunning ? " [MOTORS ON]" : " [MOTORS OFF]");
    Serial.println();

    lastPrint = millis();
  }

  // Handle serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'f' || cmd == 'F') {
      Serial.println("\n>>> MOTORS FORWARD (PWM 200)");
      // Left motor inverted
      setMotor(PIN_M1_INA, PIN_M1_INB, PIN_M1_PWM, -200);
      setMotor(PIN_M2_INA, PIN_M2_INB, PIN_M2_PWM, 200);
      motorsRunning = true;
    } else if (cmd == 's' || cmd == 'S') {
      Serial.println("\n>>> MOTORS STOP");
      setMotor(PIN_M1_INA, PIN_M1_INB, PIN_M1_PWM, 0);
      setMotor(PIN_M2_INA, PIN_M2_INB, PIN_M2_PWM, 0);
      motorsRunning = false;
    }
  }
}
