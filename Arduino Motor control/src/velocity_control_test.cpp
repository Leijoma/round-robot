/**
 * velocity_control_test.cpp - PID Velocity Control Test
 *
 * Tests closed-loop velocity control using:
 * - Encoder library (verified working)
 * - MotorControl library (verified working)
 * - PIDController library (with deadband)
 *
 * Commands:
 *   vel <L> <R>       - Set target velocities (m/s)
 *   kp <value>        - Set Kp gain for both motors
 *   kp_l/kp_r <value> - Set Kp for left/right motor
 *   ki <value>        - Set Ki gain for both motors
 *   ki_l/ki_r <value> - Set Ki for left/right motor
 *   kd <value>        - Set Kd gain for both motors
 *   kd_l/kd_r <value> - Set Kd for left/right motor
 *   db <fwd> <rev>    - Set deadband for both motors
 *   db_l/db_r <f> <r> - Set deadband for left/right motor
 *   filter <alpha>    - Set velocity filter (0.0-1.0)
 *   stop              - Stop both motors
 *   reset             - Reset PID controllers
 *   status            - Show current settings
 */

#include <Arduino.h>
#include <Encoder.h>
#include <MotorControl.h>
#include <PIDController.h>

// Hardware Configuration - Left Motor
#define PIN_ENC_L_A 2
#define PIN_ENC_L_B 10
#define PIN_M1_INA 7
#define PIN_M1_INB 8
#define PIN_M1_PWM 5

// Hardware Configuration - Right Motor
#define PIN_ENC_R_A 3
#define PIN_ENC_R_B 11
#define PIN_M2_INA 4
#define PIN_M2_INB 9
#define PIN_M2_PWM 6

// Robot Parameters
const float WHEEL_DIAMETER = 0.0825f;  // 82.5mm
const float TICKS_PER_REV = 714.0f;   // 2X decoding
const float METERS_PER_TICK = (PI * WHEEL_DIAMETER) / TICKS_PER_REV;

// Control Loop Timing
const unsigned long CONTROL_INTERVAL = 50;  // 50ms = 20 Hz
const unsigned long OUTPUT_INTERVAL = 100;   // 100ms = 10 Hz for display

// Objects - Motors and Encoders (with verified direction settings)
Encoder encoderLeft(PIN_ENC_L_A, PIN_ENC_L_B, true);    // Reversed
Encoder encoderRight(PIN_ENC_R_A, PIN_ENC_R_B, true);   // Reversed
MotorControl motorLeft(PIN_M1_INA, PIN_M1_INB, PIN_M1_PWM, true);   // Reversed
MotorControl motorRight(PIN_M2_INA, PIN_M2_INB, PIN_M2_PWM, false); // Normal

// PID Controllers
PIDController pidLeft;
PIDController pidRight;

// State Variables
float targetVelLeft = 0.0f;
float targetVelRight = 0.0f;
float currentVelLeft = 0.0f;
float currentVelRight = 0.0f;

unsigned long lastControlUpdate = 0;
unsigned long lastOutput = 0;
int32_t prevEncoderLeft = 0;
int32_t prevEncoderRight = 0;

// ISRs
void isrEncoderLeft() {
    encoderLeft.handleInterrupt();
}

void isrEncoderRight() {
    encoderRight.handleInterrupt();
}

void printStatus() {
    Serial.println(F("\n========== CURRENT SETTINGS =========="));
    Serial.println(F("LEFT MOTOR:"));
    Serial.print(F("  PID: Kp=")); Serial.print(pidLeft.Kp, 2);
    Serial.print(F(", Ki=")); Serial.print(pidLeft.Ki, 2);
    Serial.print(F(", Kd=")); Serial.println(pidLeft.Kd, 2);
    Serial.print(F("  Deadband: Fwd=")); Serial.print(pidLeft.deadband_forward, 1);
    Serial.print(F(", Rev=")); Serial.println(pidLeft.deadband_reverse, 1);
    Serial.print(F("  Filter: ")); Serial.println(pidLeft.filter_alpha, 2);

    Serial.println(F("RIGHT MOTOR:"));
    Serial.print(F("  PID: Kp=")); Serial.print(pidRight.Kp, 2);
    Serial.print(F(", Ki=")); Serial.print(pidRight.Ki, 2);
    Serial.print(F(", Kd=")); Serial.println(pidRight.Kd, 2);
    Serial.print(F("  Deadband: Fwd=")); Serial.print(pidRight.deadband_forward, 1);
    Serial.print(F(", Rev=")); Serial.println(pidRight.deadband_reverse, 1);
    Serial.print(F("  Filter: ")); Serial.println(pidRight.filter_alpha, 2);

    Serial.print(F("Target Vel - L: ")); Serial.print(targetVelLeft, 3);
    Serial.print(F(", R: ")); Serial.println(targetVelRight, 3);
    Serial.println(F("======================================\n"));
}

void setup() {
    Serial.begin(9600);
    while (!Serial && millis() < 3000);

    Serial.println(F("\n========================================"));
    Serial.println(F("  PID Velocity Control Test"));
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

    // Initialize PID controllers with TUNED values
    // Left motor - needs higher Ki and DB due to higher static friction
    pidLeft.Kp = 50.0f;
    pidLeft.Ki = 60.0f;     // Tuned to reduce steady-state error
    pidLeft.Kd = 0.0f;
    pidLeft.deadband_forward = 50.0f;  // Balanced for startup vs tracking
    pidLeft.deadband_reverse = 50.0f;
    pidLeft.filter_alpha = 0.3f;

    // Right motor - works well with standard settings
    pidRight.Kp = 50.0f;
    pidRight.Ki = 20.0f;
    pidRight.Kd = 0.0f;
    pidRight.deadband_forward = 35.0f;
    pidRight.deadband_reverse = 35.0f;
    pidRight.filter_alpha = 0.3f;

    Serial.println(F("Commands:"));
    Serial.println(F("  vel <L> <R>       Set target velocities (m/s)"));
    Serial.println(F("  kp/ki/kd <val>    Set PID gains (both motors)"));
    Serial.println(F("  kp_l/kp_r <val>   Set Kp for left/right motor"));
    Serial.println(F("  ki_l/ki_r <val>   Set Ki for left/right motor"));
    Serial.println(F("  kd_l/kd_r <val>   Set Kd for left/right motor"));
    Serial.println(F("  db <fwd> <rev>    Set deadband (both motors)"));
    Serial.println(F("  db_l <fwd> <rev>  Set left motor deadband"));
    Serial.println(F("  db_r <fwd> <rev>  Set right motor deadband"));
    Serial.println(F("  filter <alpha>    Set filter (0.0-1.0)"));
    Serial.println(F("  stop              Stop motors"));
    Serial.println(F("  reset             Reset PID"));
    Serial.println(F("  status            Show settings"));
    Serial.println();
    Serial.println(F("Output: Time | TargetL | ActualL | PWML | TargetR | ActualR | PWMR"));
    Serial.println(F("----------------------------------------------------------------------"));

    lastControlUpdate = millis();
    lastOutput = millis();
}

void loop() {
    unsigned long now = millis();

    // Control loop - 20 Hz
    if (now - lastControlUpdate >= CONTROL_INTERVAL) {
        float dt = (now - lastControlUpdate) / 1000.0f;
        lastControlUpdate = now;

        // Read encoders
        int32_t encL = encoderLeft.getCount();
        int32_t encR = encoderRight.getCount();

        // Calculate velocities
        int32_t deltaL = encL - prevEncoderLeft;
        int32_t deltaR = encR - prevEncoderRight;
        prevEncoderLeft = encL;
        prevEncoderRight = encR;

        currentVelLeft = (deltaL * METERS_PER_TICK) / dt;
        currentVelRight = (deltaR * METERS_PER_TICK) / dt;

        // Update PID controllers
        float pwmLeft = pidLeft.update(currentVelLeft, targetVelLeft, dt);
        float pwmRight = pidRight.update(currentVelRight, targetVelRight, dt);

        // Apply to motors
        motorLeft.setPWM((int16_t)pwmLeft);
        motorRight.setPWM((int16_t)pwmRight);
    }

    // Output telemetry - 10 Hz
    if (now - lastOutput >= OUTPUT_INTERVAL) {
        lastOutput = now;

        Serial.print(now);
        Serial.print(F(" | "));
        Serial.print(targetVelLeft, 3);
        Serial.print(F(" | "));
        Serial.print(pidLeft.getFilteredValue(), 3);
        Serial.print(F(" | "));
        Serial.print(motorLeft.getPWM());
        Serial.print(F(" | "));
        Serial.print(targetVelRight, 3);
        Serial.print(F(" | "));
        Serial.print(pidRight.getFilteredValue(), 3);
        Serial.print(F(" | "));
        Serial.println(motorRight.getPWM());
    }

    // Process serial commands
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toLowerCase();

        if (cmd.startsWith("vel ")) {
            int spaceIdx = cmd.indexOf(' ', 4);
            if (spaceIdx > 0) {
                targetVelLeft = cmd.substring(4, spaceIdx).toFloat();
                targetVelRight = cmd.substring(spaceIdx + 1).toFloat();
                Serial.print(F("→ Target velocities: L="));
                Serial.print(targetVelLeft, 3);
                Serial.print(F(" R="));
                Serial.print(targetVelRight, 3);
                Serial.println(F(" m/s"));
            }
        }
        else if (cmd.startsWith("kp_l ")) {
            float value = cmd.substring(5).toFloat();
            pidLeft.Kp = value;
            Serial.print(F("→ Left Kp = "));
            Serial.println(value, 2);
        }
        else if (cmd.startsWith("kp_r ")) {
            float value = cmd.substring(5).toFloat();
            pidRight.Kp = value;
            Serial.print(F("→ Right Kp = "));
            Serial.println(value, 2);
        }
        else if (cmd.startsWith("kp ")) {
            float value = cmd.substring(3).toFloat();
            pidLeft.Kp = pidRight.Kp = value;
            Serial.print(F("→ Kp = "));
            Serial.println(value, 2);
        }
        else if (cmd.startsWith("ki_l ")) {
            float value = cmd.substring(5).toFloat();
            pidLeft.Ki = value;
            Serial.print(F("→ Left Ki = "));
            Serial.println(value, 2);
        }
        else if (cmd.startsWith("ki_r ")) {
            float value = cmd.substring(5).toFloat();
            pidRight.Ki = value;
            Serial.print(F("→ Right Ki = "));
            Serial.println(value, 2);
        }
        else if (cmd.startsWith("ki ")) {
            float value = cmd.substring(3).toFloat();
            pidLeft.Ki = pidRight.Ki = value;
            Serial.print(F("→ Ki = "));
            Serial.println(value, 2);
        }
        else if (cmd.startsWith("kd_l ")) {
            float value = cmd.substring(5).toFloat();
            pidLeft.Kd = value;
            Serial.print(F("→ Left Kd = "));
            Serial.println(value, 2);
        }
        else if (cmd.startsWith("kd_r ")) {
            float value = cmd.substring(5).toFloat();
            pidRight.Kd = value;
            Serial.print(F("→ Right Kd = "));
            Serial.println(value, 2);
        }
        else if (cmd.startsWith("kd ")) {
            float value = cmd.substring(3).toFloat();
            pidLeft.Kd = pidRight.Kd = value;
            Serial.print(F("→ Kd = "));
            Serial.println(value, 2);
        }
        else if (cmd.startsWith("db_l ")) {
            int spaceIdx = cmd.indexOf(' ', 5);
            if (spaceIdx > 0) {
                float fwd = cmd.substring(5, spaceIdx).toFloat();
                float rev = cmd.substring(spaceIdx + 1).toFloat();
                pidLeft.deadband_forward = fwd;
                pidLeft.deadband_reverse = rev;
                Serial.print(F("→ Left Deadband: Fwd="));
                Serial.print(fwd, 1);
                Serial.print(F(" Rev="));
                Serial.println(rev, 1);
            }
        }
        else if (cmd.startsWith("db_r ")) {
            int spaceIdx = cmd.indexOf(' ', 5);
            if (spaceIdx > 0) {
                float fwd = cmd.substring(5, spaceIdx).toFloat();
                float rev = cmd.substring(spaceIdx + 1).toFloat();
                pidRight.deadband_forward = fwd;
                pidRight.deadband_reverse = rev;
                Serial.print(F("→ Right Deadband: Fwd="));
                Serial.print(fwd, 1);
                Serial.print(F(" Rev="));
                Serial.println(rev, 1);
            }
        }
        else if (cmd.startsWith("db ")) {
            int spaceIdx = cmd.indexOf(' ', 3);
            if (spaceIdx > 0) {
                float fwd = cmd.substring(3, spaceIdx).toFloat();
                float rev = cmd.substring(spaceIdx + 1).toFloat();
                pidLeft.deadband_forward = pidRight.deadband_forward = fwd;
                pidLeft.deadband_reverse = pidRight.deadband_reverse = rev;
                Serial.print(F("→ Deadband: Fwd="));
                Serial.print(fwd, 1);
                Serial.print(F(" Rev="));
                Serial.println(rev, 1);
            }
        }
        else if (cmd.startsWith("filter ")) {
            float value = constrain(cmd.substring(7).toFloat(), 0.0f, 1.0f);
            pidLeft.filter_alpha = pidRight.filter_alpha = value;
            Serial.print(F("→ Filter alpha = "));
            Serial.println(value, 2);
        }
        else if (cmd == "stop") {
            targetVelLeft = targetVelRight = 0.0f;
            motorLeft.stop();
            motorRight.stop();
            Serial.println(F("→ Motors stopped"));
        }
        else if (cmd == "reset") {
            pidLeft.reset();
            pidRight.reset();
            prevEncoderLeft = encoderLeft.getCount();
            prevEncoderRight = encoderRight.getCount();
            Serial.println(F("→ PID controllers reset"));
        }
        else if (cmd == "status") {
            printStatus();
        }
        else if (cmd.length() > 0) {
            Serial.print(F("→ Unknown: "));
            Serial.println(cmd);
        }
    }
}
