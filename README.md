# Drone skeleton (ESP32) — IMU + safe motor stub

This project contains a minimal, safe starting point for reading an MPU6050 IMU and a motor interface stub for an ESP32-based flight controller.

Files added
- `src/imu.h`, `src/imu.cpp` — raw I2C reader for MPU6050 + complementary filter for roll/pitch.
- `src/motor.h`, `src/motor.cpp` — safe motor interface using ESP32 LEDC PWM; motors remain disabled until armed.
- `src/main.cpp` — example program to initialize IMU, print attitude, and accept basic serial commands.

Safety notes
- Motors are DISARMED by default. You must explicitly arm via the serial command `a` after verifying wiring and ESC safety.
- Throttle is 0..1000. The code maps that range to PWM using LEDC. Verify ESC expected PWM and calibrate/adjust before flight.
- Yaw uses gyro integration only and will drift. For reliable yaw, use a magnetometer or an AHRS algorithm with magnetometer.

Wiring (assumptions)
- MPU6050 SDA -> ESP32 SDA (default Wire pins)
- MPU6050 SCL -> ESP32 SCL
- MPU6050 VCC -> 3.3V
- MPU6050 GND -> GND
- Motors/ESCs -> pins defined in `Motors::pins` (default 14,12,13,27). Change these to match your board.

Build & flash
1. Install PlatformIO and select the board.
2. Build and upload using PlatformIO UI or:

```bash
platformio run --environment upesy_wroom
platformio run --target upload --environment upesy_wroom
```

Running
- Open serial monitor at 115200. The program prints roll, pitch, yaw ~10 Hz.
- Commands via serial (newline-terminated):
  - `a` : arm motors
  - `d` : disarm motors
  - `p` : print raw sensor values
  - `t###` : set throttle for all motors (0..1000)

Next steps
- Replace complementary filter with Madgwick or Mahony AHRS for better performance.
- Add PID controllers for attitude and mixing to motors.
- Add RC input (PWM/SBUS) and failsafe handling.
- Add logging/telemetry and parameter storage.
