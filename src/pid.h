/*
  Simple PID controller for drone attitude stabilization.
  - Supports proportional, integral, derivative gains (Kp, Ki, Kd)
  - Includes anti-windup (integral clamping)
  - Reset function for when drone arms/disarms
*/
#pragma once

class PID {
public:
  PID(float kp, float ki, float kd, float outMin, float outMax);
  
  float compute(float setpoint, float measured, float dt);
  void reset();
  void setGains(float kp, float ki, float kd);
  void setLimits(float outMin, float outMax);
  
  float getKp() const { return Kp; }
  float getKi() const { return Ki; }
  float getKd() const { return Kd; }
  float getIntegral() const { return integral; }
  float getLastError() const { return lastError; }

private:
  float Kp, Ki, Kd;
  float outMin, outMax;
  float integral;
  float lastError;
};
