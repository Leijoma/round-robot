/**
 * MotorControl.h - DC Motor H-Bridge Control Library
 *
 * Controls a single DC motor using an H-bridge driver.
 * Supports bidirectional control with PWM speed control.
 */

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>

class MotorControl {
public:
    /**
     * Constructor
     * @param pinINA H-bridge input A
     * @param pinINB H-bridge input B
     * @param pinPWM H-bridge PWM/enable pin
     * @param reverse Set true to reverse motor direction
     */
    MotorControl(uint8_t pinINA, uint8_t pinINB, uint8_t pinPWM, bool reverse = false);

    /**
     * Initialize motor control pins
     */
    void begin();

    /**
     * Set motor PWM (-255 to +255)
     * Negative values run motor backwards
     * @param pwm PWM value (-255 to +255)
     */
    void setPWM(int16_t pwm);

    /**
     * Stop motor (coast)
     */
    void stop();

    /**
     * Brake motor (short both terminals)
     */
    void brake();

    /**
     * Get current PWM value
     */
    int16_t getPWM() const { return _currentPWM; }

private:
    uint8_t _pinINA;
    uint8_t _pinINB;
    uint8_t _pinPWM;
    bool _reverse;
    int16_t _currentPWM;
};

#endif
