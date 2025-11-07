#pragma once
#include <Arduino.h>

// Safe motor interface: registers pins, supports arm/disarm and set throttle
// Uses ESP32 LEDC (PWM) when armed. By default motors are disarmed.

class Motors {
public:
  Motors();
  void initPins();
  void arm();
  void disarm();
  bool isArmed() const { return armed; }
  // throttle 0..1000
  void setMotorThrottle(int idx, int throttle);
  void setAllThrottle(int throttle);

private:
  bool armed = false;
  // default pins (change in code or wiring)
  const int pins[4] = {14, 12, 13, 27};
  const int freq = 400; // Hz
  const int resolution = 10; // bits
  const int channel[4] = {0,1,2,3};
};
