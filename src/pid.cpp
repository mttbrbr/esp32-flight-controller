#include "pid.h"
#include <Arduino.h>

PID::PID(float kp, float ki, float kd, float outMin, float outMax)
  : Kp(kp), Ki(ki), Kd(kd), outMin(outMin), outMax(outMax),
    integral(0), lastError(0) {}

float PID::compute(float setpoint, float measured, float dt) {
  if (dt <= 0) dt = 0.001f; // safety
  
  float error = setpoint - measured;
  
  // Proportional
  float P = Kp * error;
  
  // Integral (with anti-windup clamping)
  integral += error * dt;
  float maxIntegral = (outMax - outMin) / 2.0f;
  integral = constrain(integral, -maxIntegral, maxIntegral);
  float I = Ki * integral;
  
  // Derivative
  float derivative = (error - lastError) / dt;
  float D = Kd * derivative;
  
  lastError = error;
  
  float output = P + I + D;
  return constrain(output, outMin, outMax);
}

void PID::reset() {
  integral = 0;
  lastError = 0;
}

void PID::setGains(float kp, float ki, float kd) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
}

void PID::setLimits(float outMin, float outMax) {
  this->outMin = outMin;
  this->outMax = outMax;
}
