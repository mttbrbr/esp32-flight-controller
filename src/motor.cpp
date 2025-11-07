#include "motor.h"
#include <Arduino.h>

Motors::Motors() {}

void Motors::initPins() {
  // configure PWM channels but do not output until armed
  for (int i = 0; i < 4; ++i) {
    ledcSetup(channel[i], freq, resolution);
    ledcAttachPin(pins[i], channel[i]);
    // write minimum (0) to be safe
    ledcWrite(channel[i], 0);
  }
  armed = false;
}

void Motors::arm() {
  // In a real setup: check failsafes, delay, send ESC arming sequence
  armed = true;
}

void Motors::disarm() {
  // stop outputs immediately
  for (int i = 0; i < 4; ++i) ledcWrite(channel[i], 0);
  armed = false;
}

void Motors::setMotorThrottle(int idx, int throttle) {
  if (idx < 0 || idx >= 4) return;
  throttle = constrain(throttle, 0, 1000);
  if (!armed) return; // safety: do nothing unless armed
  // map 0..1000 to PWM range (0..(2^resolution-1))
  int max = (1 << resolution) - 1;
  int out = map(throttle, 0, 1000, 0, max);
  ledcWrite(channel[idx], out);
}

void Motors::setAllThrottle(int throttle) {
  for (int i = 0; i < 4; ++i) setMotorThrottle(i, throttle);
}
