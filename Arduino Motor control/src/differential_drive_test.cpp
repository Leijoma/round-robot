/**
 * differential_drive_test.cpp - Differential Drive Control Test
 *
 * Implements differential drive kinematics:
 *   v_left = v_linear - (omega * wheelbase / 2)
 *   v_right = v_linear + (omega * wheelbase / 2)
 *
 * Commands:
 *   drive <linear> <angular>  - Set velocities (m/s, rad/s)
 *   stop                      - Stop both motors
 *   reset                     - Reset PID controllers
 *   status                    - Show current settings
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
const float WHEELBASE = 0.244f;        // 244mm
const float TICKS_PER_REV = 714.0f;    // 2X decoding
const float METERS_PER_TICK = (PI * WHEEL_DIAMETER) / TICKS_PER_REV;

// Control Loop Timing
const unsigned long CONTROL_INTERVAL = 50;  // 50ms = 20 Hz
const unsigned long OUTPUT_INTERVAL = 100;   // 100ms = 10 Hz

// Objects
Encoder encoderLeft(PIN_ENC_L_A, PIN_ENC_L_B, true);
Encoder encoderRight(PIN_ENC_R_A, PIN_ENC_R_B, true);
MotorControl motorLeft(PIN_M1_INA, PIN_M1_INB, PIN_M1_PWM, true);
MotorControl motorRight(PIN_M2_INA, PIN_M2_INB, PIN_M2_PWM, false);
PIDController pidLeft;
PIDController pidRight;

// State Variables
float targetLinearVel = 0.0f;
float targetAngularVel = 0.0f;
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

void updateWheelVelocities() {
    // Differential drive kinematics
    // v_left = v_linear - (omega * wheelbase / 2)
    // v_right = v_linear + (omega * wheelbase / 2)
    float halfWheelbase = WHEELBASE / 2.0f;
    targetVelLeft = targetLinearVel - (targetAngularVel * halfWheelbase);
    targetVelRight = targetLinearVel + (targetAngularVel * halfWheelbase);
}

void printStatus() {
    Serial.println(F("\n========== DIFFERENTIAL DRIVE STATUS =========="));
    Serial.print(F("Command: Linear=")); Serial.print(targetLinearVel, 3);
    Serial.print(F(" m/s, Angular=")); Serial.print(targetAngularVel, 3);
    Serial.println(F(" rad/s"));

    Serial.print(F("Wheel Targets: Left=")); Serial.print(targetVelLeft, 3);
    Serial.print(F(" m/s, Right=")); Serial.print(targetVelRight, 3);
    Serial.println(F(" m/s"));

    Serial.println(F("\nLEFT MOTOR:"));
    Serial.print(F("  PID: Kp=")); Serial.print(pidLeft.Kp, 2);
    Serial.print(F(", Ki=")); Serial.print(pidLeft.Ki, 2);
    Serial.print(F(", Kd=")); Serial.println(pidLeft.Kd, 2);
    Serial.print(F("  Deadband: ")); Serial.println(pidLeft.deadband_forward, 1);

    Serial.println(F("RIGHT MOTOR:"));
    Serial.print(F("  PID: Kp=")); Serial.print(pidRight.Kp, 2);
    Serial.print(F(", Ki=")); Serial.print(pidRight.Ki, 2);
    Serial.print(F(", Kd=")); Serial.println(pidRight.Kd, 2);
    Serial.print(F("  Deadband: ")); Serial.println(pidRight.deadband_forward, 1);
    Serial.println(F("===============================================\n"));
}

void setup() {
    Serial.begin(9600);
    while (!Serial && millis() < 3000);

    Serial.println(F("\n========================================"));
    Serial.println(F("  Differential Drive Control Test"));
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

    // Initialize PID with TUNED values
    pidLeft.Kp = 50.0f;
    pidLeft.Ki = 60.0f;
    pidLeft.Kd = 0.0f;
    pidLeft.deadband_forward = 50.0f;
    pidLeft.deadband_reverse = 50.0f;
    pidLeft.filter_alpha = 0.3f;

    pidRight.Kp = 50.0f;
    pidRight.Ki = 20.0f;
    pidRight.Kd = 0.0f;
    pidRight.deadband_forward = 35.0f;
    pidRight.deadband_reverse = 35.0f;
    pidRight.filter_alpha = 0.3f;

    Serial.println(F("Robot Parameters:"));
    Serial.print(F("  Wheel Diameter: ")); Serial.print(WHEEL_DIAMETER * 1000); Serial.println(F(" mm"));
    Serial.print(F("  Wheelbase: ")); Serial.print(WHEELBASE * 1000); Serial.println(F(" mm"));
    Serial.println();

    Serial.println(F("Commands:"));
    Serial.println(F("  drive <linear> <angular>  Set velocities (m/s, rad/s)"));
    Serial.println(F("  stop                      Stop motors"));
    Serial.println(F("  reset                     Reset PID"));
    Serial.println(F("  status                    Show settings"));
    Serial.println();
    Serial.println(F("Examples:"));
    Serial.println(F("  drive 0.15 0        Drive straight at 0.15 m/s"));
    Serial.println(F("  drive 0 0.5         Rotate in place at 0.5 rad/s"));
    Serial.println(F("  drive 0.15 0.3      Drive forward while turning"));
    Serial.println();
    Serial.println(F("Output: Time | LinVel | AngVel | L_Tgt | L_Act | L_PWM | R_Tgt | R_Act | R_PWM"));
    Serial.println(F("--------------------------------------------------------------------------------"));

    lastControlUpdate = millis();
    lastOutput = millis();
}

void loop() {
    unsigned long now = millis();

    // Control loop - 20 Hz
    if (now - lastControlUpdate >= CONTROL_INTERVAL) {
        float dt = (now - lastControlUpdate) / 1000.0f;
        lastControlUpdate = now;

        // Read encoders and calculate velocities
        int32_t encL = encoderLeft.getCount();
        int32_t encR = encoderRight.getCount();
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

        // Calculate actual linear and angular velocities
        float actualLinear = (pidLeft.getFilteredValue() + pidRight.getFilteredValue()) / 2.0f;
        float actualAngular = (pidRight.getFilteredValue() - pidLeft.getFilteredValue()) / WHEELBASE;

        Serial.print(now);
        Serial.print(F(" | "));
        Serial.print(actualLinear, 3);
        Serial.print(F(" | "));
        Serial.print(actualAngular, 3);
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

        if (cmd.startsWith("drive ")) {
            int spaceIdx = cmd.indexOf(' ', 6);
            if (spaceIdx > 0) {
                targetLinearVel = cmd.substring(6, spaceIdx).toFloat();
                targetAngularVel = cmd.substring(spaceIdx + 1).toFloat();
                updateWheelVelocities();
                Serial.print(F("→ Drive: Linear="));
                Serial.print(targetLinearVel, 3);
                Serial.print(F(" m/s, Angular="));
                Serial.print(targetAngularVel, 3);
                Serial.println(F(" rad/s"));
                Serial.print(F("  Wheel velocities: L="));
                Serial.print(targetVelLeft, 3);
                Serial.print(F(" R="));
                Serial.print(targetVelRight, 3);
                Serial.println(F(" m/s"));
            }
        }
        else if (cmd == "stop") {
            targetLinearVel = targetAngularVel = 0.0f;
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
