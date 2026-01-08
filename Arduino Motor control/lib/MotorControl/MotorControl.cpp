/**
 * MotorControl.cpp - DC Motor H-Bridge Control Implementation
 */

#include "MotorControl.h"

MotorControl::MotorControl(uint8_t pinINA, uint8_t pinINB, uint8_t pinPWM, bool reverse)
    : _pinINA(pinINA), _pinINB(pinINB), _pinPWM(pinPWM), _reverse(reverse), _currentPWM(0) {
}

void MotorControl::begin() {
    pinMode(_pinINA, OUTPUT);
    pinMode(_pinINB, OUTPUT);
    pinMode(_pinPWM, OUTPUT);
    stop();
}

void MotorControl::setPWM(int16_t pwm) {
    // Constrain PWM to valid range
    pwm = constrain(pwm, -255, 255);

    // Apply reversal if configured
    if (_reverse) {
        pwm = -pwm;
    }

    _currentPWM = pwm;

    if (pwm > 0) {
        // Forward
        digitalWrite(_pinINA, HIGH);
        digitalWrite(_pinINB, LOW);
        analogWrite(_pinPWM, pwm);
    } else if (pwm < 0) {
        // Reverse
        digitalWrite(_pinINA, LOW);
        digitalWrite(_pinINB, HIGH);
        analogWrite(_pinPWM, -pwm);
    } else {
        // Stop (coast)
        stop();
    }
}

void MotorControl::stop() {
    digitalWrite(_pinINA, LOW);
    digitalWrite(_pinINB, LOW);
    analogWrite(_pinPWM, 0);
    _currentPWM = 0;
}

void MotorControl::brake() {
    digitalWrite(_pinINA, HIGH);
    digitalWrite(_pinINB, HIGH);
    analogWrite(_pinPWM, 255);
    _currentPWM = 0;
}
