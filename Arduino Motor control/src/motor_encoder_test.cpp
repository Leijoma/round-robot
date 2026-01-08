/**
 * motor_encoder_test.cpp - Test Motor + Encoder Libraries
 *
 * Verifies that:
 * - Motor spins when commanded
 * - Encoder counts correctly
 * - Velocity calculation works
 * - Direction handling is correct
 *
 * Commands:
 *   pwm <value>  - Set motor PWM (-255 to +255)
 *   stop         - Stop motor
 *   reset        - Reset encoder count
 */

#include <Arduino.h>
#include <Encoder.h>
#include <MotorControl.h>

// Hardware Configuration - Left Motor
#define PIN_ENC_L_A 2
#define PIN_ENC_L_B 10
#define PIN_M1_INA 7   // Left motor direction A
#define PIN_M1_INB 8   // Left motor direction B
#define PIN_M1_PWM 5   // Left motor PWM

// Hardware Configuration - Right Motor
#define PIN_ENC_R_A 3
#define PIN_ENC_R_B 11
#define PIN_M2_INA 4   // Right motor direction A
#define PIN_M2_INB 9   // Right motor direction B
#define PIN_M2_PWM 6   // Right motor PWM

// Robot Parameters
const float WHEEL_DIAMETER = 0.0825f;  // 82.5mm
const float TICKS_PER_REV = 714.0f;   // 2X decoding
const float METERS_PER_TICK = (PI * WHEEL_DIAMETER) / TICKS_PER_REV;

// Update Rate
const unsigned long UPDATE_INTERVAL = 100;  // 100ms = 10 Hz

// Objects
// Motors correct, now fix encoder directions to match
Encoder encoderLeft(PIN_ENC_L_A, PIN_ENC_L_B, true);   // Reversed
Encoder encoderRight(PIN_ENC_R_A, PIN_ENC_R_B, true);  // Also Reversed
MotorControl motorLeft(PIN_M1_INA, PIN_M1_INB, PIN_M1_PWM, true);    // Reversed
MotorControl motorRight(PIN_M2_INA, PIN_M2_INB, PIN_M2_PWM, false);  // Normal

// State Variables
unsigned long lastUpdate = 0;
int32_t prevEncoderCountL = 0;
int32_t prevEncoderCountR = 0;

// ISRs
void isrEncoderLeft() {
    encoderLeft.handleInterrupt();
}

void isrEncoderRight() {
    encoderRight.handleInterrupt();
}

void setup() {
    Serial.begin(9600);
    while (!Serial && millis() < 3000);  // Wait up to 3s for serial

    Serial.println(F("\n========================================"));
    Serial.println(F("  Motor + Encoder Test"));
    Serial.println(F("========================================"));
    Serial.println();

    // Initialize encoders
    encoderLeft.begin();
    encoderRight.begin();
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncoderLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncoderRight, CHANGE);

    // Initialize motors
    motorLeft.begin();
    motorRight.begin();

    Serial.println(F("Commands:"));
    Serial.println(F("  pwm <L> <R>   Set PWM for both motors (-255 to +255)"));
    Serial.println(F("  stop          Stop both motors"));
    Serial.println(F("  reset         Reset encoders"));
    Serial.println();
    Serial.println(F("Output: Time | EncL | VelL | PWML | EncR | VelR | PWMR"));
    Serial.println(F("----------------------------------------------------------------"));

    lastUpdate = millis();
    prevEncoderCountL = 0;
    prevEncoderCountR = 0;
}

void loop() {
    unsigned long now = millis();

    // Periodic status output
    if (now - lastUpdate >= UPDATE_INTERVAL) {
        float dt = (now - lastUpdate) / 1000.0f;
        lastUpdate = now;

        // Read left encoder
        int32_t countL = encoderLeft.getCount();
        int32_t deltaL = countL - prevEncoderCountL;
        prevEncoderCountL = countL;
        float velocityL = (deltaL * METERS_PER_TICK) / dt;

        // Read right encoder
        int32_t countR = encoderRight.getCount();
        int32_t deltaR = countR - prevEncoderCountR;
        prevEncoderCountR = countR;
        float velocityR = (deltaR * METERS_PER_TICK) / dt;

        // Display both motors
        Serial.print(now);
        Serial.print(F(" | "));
        Serial.print(countL);
        Serial.print(F(" | "));
        Serial.print(velocityL, 4);
        Serial.print(F(" | "));
        Serial.print(motorLeft.getPWM());
        Serial.print(F(" | "));
        Serial.print(countR);
        Serial.print(F(" | "));
        Serial.print(velocityR, 4);
        Serial.print(F(" | "));
        Serial.println(motorRight.getPWM());
    }

    // Process serial commands
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toLowerCase();

        if (cmd.startsWith("pwm ")) {
            // Parse two PWM values: "pwm <L> <R>"
            int spaceIdx = cmd.indexOf(' ', 4);
            if (spaceIdx > 0) {
                int pwmL = cmd.substring(4, spaceIdx).toInt();
                int pwmR = cmd.substring(spaceIdx + 1).toInt();
                motorLeft.setPWM(pwmL);
                motorRight.setPWM(pwmR);
                Serial.print(F("→ PWM set to: L="));
                Serial.print(pwmL);
                Serial.print(F(" R="));
                Serial.println(pwmR);
            } else {
                Serial.println(F("→ Error: pwm requires two values"));
            }
        }
        else if (cmd == "stop") {
            motorLeft.stop();
            motorRight.stop();
            Serial.println(F("→ Motors stopped"));
        }
        else if (cmd == "reset") {
            encoderLeft.reset();
            encoderRight.reset();
            prevEncoderCountL = 0;
            prevEncoderCountR = 0;
            Serial.println(F("→ Encoders reset"));
        }
        else if (cmd.length() > 0) {
            Serial.print(F("→ Unknown command: "));
            Serial.println(cmd);
        }
    }
}
