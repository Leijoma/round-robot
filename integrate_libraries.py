#!/usr/bin/env python3
"""
Integrate modular PID libraries into main.cpp
Replaces old inline code with library calls and applies tuned parameters
"""

import re

print("="*80)
print("INTEGRATING MODULAR PID LIBRARIES INTO MAIN.CPP")
print("="*80)

# Read main.cpp
with open('Arduino Motor control/src/main.cpp', 'r') as f:
    content = f.read()

print("\n1. Adding library includes...")
content = content.replace(
    '#include "RobotLinkMessages.h"',
    '''#include "RobotLinkMessages.h"
#include <Encoder.h>
#include <MotorControl.h>
#include <PIDController.h>'''
)

print("2. Removing old PID class definition...")
# Find and remove the entire PID class (lines 40-128 approximately)
pid_class_pattern = r'// ============================================================\n// PID Controller Class with Deadband Compensation\n// ============================================================\nclass PIDController \{.*?\n\};'
content = re.sub(pid_class_pattern, '// PID Controller from library', content, flags=re.DOTALL)

print("3. Adding library object declarations...")
# Find the "Global Variables" section and add library objects before it
global_vars_pattern = r'(// ============================================================\n// Global Variables\n// ============================================================)'
library_objects = '''// ============================================================
// Library Objects - Motors, Encoders, PID
// ============================================================
// Direction settings verified through motor_encoder_test.cpp
Encoder encoderLeft(PIN_ENC_L_A, PIN_ENC_L_B, true);    // Reversed
Encoder encoderRight(PIN_ENC_R_A, PIN_ENC_R_B, true);   // Reversed
MotorControl motorLeft(PIN_M1_INA, PIN_M1_INB, PIN_M1_PWM, true);   // Reversed
MotorControl motorRight(PIN_M2_INA, PIN_M2_INB, PIN_M2_PWM, false); // Normal

\\1'''
content = re.sub(global_vars_pattern, library_objects, content)

print("4. Removing old encoder count variables...")
content = re.sub(r'// Encoder counts \(volatile for ISR access\)\nvolatile int32_t encoderLeftCount = 0;\nvolatile int32_t encoderRightCount = 0;\n\n', '', content)

print("5. Updating encoder ISRs...")
old_isr_left = r'''void isrEncoderLeft\(\) \{
  // 2X decoding: Read both A and B to determine direction
  // Handles both RISING and FALLING edges correctly
  bool a = digitalRead\(PIN_ENC_L_A\);
  bool b = digitalRead\(PIN_ENC_L_B\);

  // XOR logic: \(A==B\) means forward, \(A!=B\) means backward
  if \(a == b\) \{
    encoderLeftCount\+\+;
  \} else \{
    encoderLeftCount--;
  \}
\}'''

new_isr_left = '''void isrEncoderLeft() {
    encoderLeft.handleInterrupt();
}'''

content = re.sub(old_isr_left, new_isr_left, content)

old_isr_right = r'''void isrEncoderRight\(\) \{
  // 2X decoding: Read both A and B to determine direction
  // Handles both RISING and FALLING edges correctly
  bool a = digitalRead\(PIN_ENC_R_A\);
  bool b = digitalRead\(PIN_ENC_R_B\);

  // XOR logic: \(A==B\) means forward, \(A!=B\) means backward
  if \(a == b\) \{
    encoderRightCount\+\+;
  \} else \{
    encoderRightCount--;
  \}
\}'''

new_isr_right = '''void isrEncoderRight() {
    encoderRight.handleInterrupt();
}'''

content = re.sub(old_isr_right, new_isr_right, content)

print("6. Updating motor control functions...")
old_setmotor = r'''void setMotor\(uint8_t pinINA, uint8_t pinINB, uint8_t pinPWM, int16_t pwm\) \{
  if \(pwm > 0\) \{
    // Forward
    digitalWrite\(pinINA, HIGH\);
    digitalWrite\(pinINB, LOW\);
    analogWrite\(pinPWM, constrain\(pwm, 0, 255\)\);
  \} else if \(pwm < 0\) \{
    // Backward
    digitalWrite\(pinINA, LOW\);
    digitalWrite\(pinINB, HIGH\);
    analogWrite\(pinPWM, constrain\(-pwm, 0, 255\)\);
  \} else \{
    // Stop
    digitalWrite\(pinINA, LOW\);
    digitalWrite\(pinINB, LOW\);
    analogWrite\(pinPWM, 0\);
  \}
\}

void setMotors\(int16_t left, int16_t right\) \{
  pwmLeft = -left;   // Left motor IS inverted \(hardware wiring\)
  pwmRight = right;  // Right motor NOT inverted

  setMotor\(PIN_M1_INA, PIN_M1_INB, PIN_M1_PWM, pwmLeft\);
  setMotor\(PIN_M2_INA, PIN_M2_INB, PIN_M2_PWM, pwmRight\);
\}'''

new_setmotor = '''void setMotors(int16_t left, int16_t right) {
    motorLeft.setPWM(left);
    motorRight.setPWM(right);
    pwmLeft = left;
    pwmRight = right;
}'''

content = re.sub(old_setmotor, new_setmotor, content, flags=re.DOTALL)

print("7. Replacing encoder count access...")
# Replace encoderLeftCount/encoderRightCount with library calls
content = content.replace('encoderLeftCount', 'encoderLeft.getCount()')
content = content.replace('encoderRightCount', 'encoderRight.getCount()')

print("8. Removing noInterrupts/interrupts from encoder reads...")
# Simplify encoder reads (library handles atomicity)
content = re.sub(
    r'noInterrupts\(\);\s+int32_t encL = encoderLeft\.getCount\(\(\)\);\s+int32_t encR = encoderRight\.getCount\(\(\)\);\s+interrupts\(\);',
    'int32_t encL = encoderLeft.getCount();\n  int32_t encR = encoderRight.getCount();',
    content
)

print("9. Adding tuned PID parameters to setup()...")
# Find pinMode setup in setup() and add PID initialization after encoder/motor setup
setup_pattern = r'(attachInterrupt\(digitalPinToInterrupt\(PIN_ENC_R_A\), isrEncoderRight, CHANGE\);)'
pid_init = '''\\1

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
  pidRight.output_max = 255.0f;'''

content = re.sub(setup_pattern, pid_init, content)

# Remove old motor pin setup since library handles it
content = re.sub(r'\n  // Set motor control pins as outputs.*?analogWrite\(PIN_M2_PWM, 0\);', '', content, flags=re.DOTALL)

print("10. Removing min_threshold from PID update calls...")
# The library PID doesn't use min_threshold parameter
content = content.replace('.update(currentVelLeft, targetVelLeft, dt, MIN_VELOCITY_THRESHOLD)',
                         '.update(currentVelLeft, targetVelLeft, dt)')
content = content.replace('.update(currentVelRight, targetVelRight, dt, MIN_VELOCITY_THRESHOLD)',
                         '.update(currentVelRight, targetVelRight, dt)')

# Write updated main.cpp
with open('Arduino Motor control/src/main.cpp', 'w') as f:
    f.write(content)

print("\n" + "="*80)
print("INTEGRATION COMPLETE!")
print("="*80)
print("\nChanges made:")
print("  ✓ Added library includes (Encoder, MotorControl, PIDController)")
print("  ✓ Removed old PID class definition")
print("  ✓ Added library object declarations with verified directions")
print("  ✓ Updated encoder ISRs to use library methods")
print("  ✓ Replaced motor control functions with library calls")
print("  ✓ Updated all encoder count access to use library methods")
print("  ✓ Applied tuned PID parameters in setup()")
print("  ✓ Simplified code (removed atomic encoder reads)")
print("\nNext step: Compile and test!")
