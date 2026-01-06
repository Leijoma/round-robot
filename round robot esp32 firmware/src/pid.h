#pragma once
#include <Arduino.h>

struct Pid {
  float kp = 0.0f;
  float ki = 0.0f;   // per second
  float kd = 0.0f;   // seconds

  float iTerm = 0.0f;
  float prevErr = 0.0f;
  float outMin = -255.0f;
  float outMax =  255.0f;
  float iMin = -150.0f;
  float iMax =  150.0f;

  void reset() {
    iTerm = 0.0f;
    prevErr = 0.0f;
  }

  float step(float setpoint, float measured, float dt_s) {
    if (dt_s <= 0) return 0;

    float err = setpoint - measured;

    // Integrator (anti-windup clamp)
    iTerm += err * ki * dt_s;
    if (iTerm > iMax) iTerm = iMax;
    if (iTerm < iMin) iTerm = iMin;

    float dErr = (err - prevErr) / dt_s;
    prevErr = err;

    float out = (kp * err) + iTerm + (kd * dErr);
    if (out > outMax) out = outMax;
    if (out < outMin) out = outMin;

    return out;
  }
};
