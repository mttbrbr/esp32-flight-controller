/*
  Simple MPU6050 reader + complementary filter for roll/pitch.
  - Uses Wire.h to read MPU6050 registers directly (no external library dependency)
  - Provides roll/pitch/yaw (yaw is gyro-integrated and will drift unless magnetometer used)
  - Returns accel in m/s^2 and gyro in deg/s
*/
#pragma once
#include <Arduino.h>
#include <Wire.h>

struct Vec3 { float x, y, z; };

class IMU {
public:
  IMU();
  bool begin();
  void update();
  void setUseMadgwick(bool en) { useMadgwick = en; }
  float getRoll() const { return roll; }
  float getPitch() const { return pitch; }
  float getYaw() const { return yaw; }
  Vec3 getAccel() const { return accel; }
  Vec3 getGyro() const { return gyro; }

private:
  void readRaw();
  int16_t read16(uint8_t reg);

  // raw
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  // converted
  Vec3 accel; // m/s^2
  Vec3 gyro;  // deg/s

  // angle state
  float roll = 0, pitch = 0, yaw = 0;
  unsigned long lastMicros = 0;
  const float alpha = 0.98; // complementary filter weight
  // Madgwick AHRS state (quaternion)
  bool useMadgwick = false;
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
  float beta = 0.1f; // algorithm gain
  void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt);
};
