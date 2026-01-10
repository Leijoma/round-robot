/**
 * PIDController.cpp - PID Controller Implementation
 */

#include "PIDController.h"

PIDController::PIDController()
    : Kp(0.0f), Ki(0.0f), Kd(0.0f),
      Kff_v(0.0f), Kff_a(0.0f),
      deadband_forward(0.0f), deadband_reverse(0.0f),
      filter_alpha(0.3f),
      output_min(-255.0f), output_max(255.0f),
      _integral(0.0f), _prev_setpoint(0.0f), _prev_measurement(0.0f), _filtered_value(0.0f) {
}

void PIDController::reset() {
    _integral = 0.0f;
    _prev_setpoint = 0.0f;
    _prev_measurement = 0.0f;
    _filtered_value = 0.0f;
}

float PIDController::update(float measurement, float setpoint, float dt) {
    // 1. Low-pass filter the measurement
    _filtered_value = filter_alpha * measurement + (1.0f - filter_alpha) * _filtered_value;

    // 2. Calculate error using filtered value
    float error = setpoint - _filtered_value;

    // 3. Proportional term
    float p_term = Kp * error;

    // 4. Integral term
    _integral += error * dt;
    float i_term = Ki * _integral;

    // 5. Derivative term (usually 0 for DC motors)
    float d_term = 0.0f;
    if (Kd > 0.0f && dt > 0.0f) {
        float derivative = (_filtered_value - _prev_measurement) / dt;
        d_term = -Kd * derivative;  // Negative because we want to dampen change
    }

    // 6. Feed-forward terms
    float accel = 0.0f;
    if (dt > 0.0f) {
        accel = (setpoint - _prev_setpoint) / dt;
    }
    float ff_term = Kff_v * setpoint + Kff_a * accel;

    // 7. Combine all terms
    float output_raw = ff_term + p_term + i_term + d_term;

    // 8. Apply deadband compensation
    if (setpoint > 0.001f) {
        output_raw += deadband_forward;
    } else if (setpoint < -0.001f) {
        output_raw -= deadband_reverse;
    }
    // Note: When setpoint near zero, no deadband applied
    // Velocity ramping ensures gradual approach to zero, so PID can track smoothly

    // 9. Clamp to output limits
    float output = constrain(output_raw, output_min, output_max);

    // 10. Back-calculation anti-windup
    if (output != output_raw && Ki > 0.0001f) {
        float excess = output_raw - output;
        _integral -= excess / Ki * 0.5f;  // Factor 0.5 for stability
    }

    // 11. Store previous values
    _prev_setpoint = setpoint;
    _prev_measurement = _filtered_value;

    return output;
}
