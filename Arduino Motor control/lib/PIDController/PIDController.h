/**
 * PIDController.h - PID Controller with Feed-Forward and Anti-Windup
 *
 * Features:
 * - PI control (D-term typically not needed for DC motors)
 * - Velocity and acceleration feed-forward
 * - Back-calculation anti-windup
 * - Low-pass velocity filtering
 * - Deadband compensation
 */

#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
    // PID Gains
    float Kp;
    float Ki;
    float Kd;

    // Feed-Forward Gains
    float Kff_v;  // Velocity feed-forward
    float Kff_a;  // Acceleration feed-forward

    // Deadband Compensation
    float deadband_forward;
    float deadband_reverse;

    // Velocity Filter
    float filter_alpha;  // 0.0 = max filtering, 1.0 = no filtering

    // Output Limits
    float output_min;
    float output_max;

    /**
     * Constructor with default values
     */
    PIDController();

    /**
     * Reset controller state (integral, previous values)
     */
    void reset();

    /**
     * Update controller
     * @param measurement Current measured value
     * @param setpoint Target setpoint
     * @param dt Time step (seconds)
     * @return Control output
     */
    float update(float measurement, float setpoint, float dt);

    /**
     * Get filtered measurement (useful for logging)
     */
    float getFilteredValue() const { return _filtered_value; }

    /**
     * Get integral term (useful for diagnostics)
     */
    float getIntegral() const { return _integral; }

private:
    float _integral;
    float _prev_setpoint;
    float _prev_measurement;
    float _filtered_value;
};

#endif
