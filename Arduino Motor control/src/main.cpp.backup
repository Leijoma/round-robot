/*
 * Motor Hardware Test - Direct PWM control to test if motors work
 * Upload this to test motors without PID/deadband
 */

#include <Arduino.h>

// Monster Moto Shield pins
const uint8_t PIN_M1_INA = 7;   // Left motor direction A
const uint8_t PIN_M1_INB = 8;   // Left motor direction B
const uint8_t PIN_M1_PWM = 5;   // Left motor PWM (Timer 0)
const uint8_t PIN_M2_INA = 4;   // Right motor direction A
const uint8_t PIN_M2_INB = 9;   // Right motor direction B
const uint8_t PIN_M2_PWM = 6;   // Right motor PWM (Timer 0)

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== MOTOR HARDWARE TEST ===");
  Serial.println("Direct PWM control to verify motors work");
  Serial.println("====================================\n");

  // Configure motor pins
  pinMode(PIN_M1_INA, OUTPUT);
  pinMode(PIN_M1_INB, OUTPUT);
  pinMode(PIN_M1_PWM, OUTPUT);
  pinMode(PIN_M2_INA, OUTPUT);
  pinMode(PIN_M2_INB, OUTPUT);
  pinMode(PIN_M2_PWM, OUTPUT);

  // Stop motors initially
  digitalWrite(PIN_M1_INA, LOW);
  digitalWrite(PIN_M1_INB, LOW);
  analogWrite(PIN_M1_PWM, 0);
  digitalWrite(PIN_M2_INA, LOW);
  digitalWrite(PIN_M2_INB, LOW);
  analogWrite(PIN_M2_PWM, 0);

  Serial.println("Motors initialized (stopped)");
  Serial.println("Starting test sequence...\n");
}

void setMotor(uint8_t pinA, uint8_t pinB, uint8_t pinPWM, int16_t pwm) {
  if (pwm >= 0) {
    // Forward
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
    analogWrite(pinPWM, pwm);
  } else {
    // Reverse
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
    analogWrite(pinPWM, -pwm);
  }
}

void testMotor(const char* name, uint8_t pinA, uint8_t pinB, uint8_t pinPWM) {
  Serial.print("Testing ");
  Serial.print(name);
  Serial.println(":");

  // Test PWM values: 50, 100, 150, 200, 255
  int16_t pwm_values[] = {50, 75, 100, 125, 150, 175, 200, 225, 255};

  for (int16_t pwm : pwm_values) {
    Serial.print("  PWM=");
    Serial.print(pwm);
    Serial.println(" (motor should spin)");

    setMotor(pinA, pinB, pinPWM, pwm);
    delay(2000);  // Run for 2 seconds

    // Stop
    setMotor(pinA, pinB, pinPWM, 0);
    delay(1000);  // Pause 1 second
  }

  Serial.println();
}

void loop() {
  // Test left motor
  Serial.println("\n=== LEFT MOTOR TEST ===");
  testMotor("LEFT MOTOR", PIN_M1_INA, PIN_M1_INB, PIN_M1_PWM);

  delay(2000);

  // Test right motor
  Serial.println("\n=== RIGHT MOTOR TEST ===");
  testMotor("RIGHT MOTOR", PIN_M2_INA, PIN_M2_INB, PIN_M2_PWM);

  delay(2000);

  // Test both motors together
  Serial.println("\n=== BOTH MOTORS TEST ===");
  Serial.println("Both motors at PWM=150");
  setMotor(PIN_M1_INA, PIN_M1_INB, PIN_M1_PWM, 150);
  setMotor(PIN_M2_INA, PIN_M2_INB, PIN_M2_PWM, 150);
  delay(3000);

  // Stop both
  setMotor(PIN_M1_INA, PIN_M1_INB, PIN_M1_PWM, 0);
  setMotor(PIN_M2_INA, PIN_M2_INB, PIN_M2_PWM, 0);

  Serial.println("\n=== TEST COMPLETE ===");
  Serial.println("Repeating in 5 seconds...\n");
  delay(5000);
}
