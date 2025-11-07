#include "imu.h"

// MPU6050 I2C address and registers
static const uint8_t MPU_ADDR = 0x68;
static const uint8_t REG_PWR = 0x6B;
static const uint8_t REG_ACCEL_X = 0x3B;
static const uint8_t REG_GYRO_X = 0x43;

IMU::IMU() {}

bool IMU::begin() {
  // wake up
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_PWR);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;
  // small delay
  delay(10);
  lastMicros = micros();
  return true;
}

int16_t IMU::read16(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 2);
  int16_t hi = Wire.read();
  int16_t lo = Wire.read();
  return (hi << 8) | lo;
}

void IMU::readRaw() {
  // read 14 bytes accel(6) temp(2) gyro(6)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_X);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 14);
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  int16_t tmp = (Wire.read() << 8) | Wire.read(); (void)tmp;
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

void IMU::update() {
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0f;
  if (dt <= 0) dt = 0.001f;
  lastMicros = now;

  readRaw();

  // convert raw to units
  // assuming default ±2g (16384 LSB/g) and ±250 dps (131 LSB/dps)
  const float ACC_SCALE = 9.80665f / 16384.0f; // to m/s^2
  const float GYRO_SCALE = 1.0f / 131.0f; // dps per LSB

  accel.x = ax * ACC_SCALE;
  accel.y = ay * ACC_SCALE;
  accel.z = az * ACC_SCALE;

  gyro.x = gx * GYRO_SCALE;
  gyro.y = gy * GYRO_SCALE;
  gyro.z = gz * GYRO_SCALE;

  // compute accel angles (degrees)
  float accRoll = atan2f(accel.y, accel.z) * 180.0f / PI;
  float accPitch = atan2f(-accel.x, sqrtf(accel.y*accel.y + accel.z*accel.z)) * 180.0f / PI;

  // integrate gyro to angles
  float gyroRollRate = gyro.x; // deg/s
  float gyroPitchRate = gyro.y;
  float gyroYawRate = gyro.z;

  if (useMadgwick) {
    // Madgwick expects gyro in rad/s and accel in g (or normalized)
    float gx = gyro.x * PI / 180.0f;
    float gy = gyro.y * PI / 180.0f;
    float gz = gyro.z * PI / 180.0f;
    // normalize accel to g
    float axn = accel.x / 9.80665f;
    float ayn = accel.y / 9.80665f;
    float azn = accel.z / 9.80665f;
    madgwickUpdate(gx, gy, gz, axn, ayn, azn, dt);
    // compute Euler angles from quaternion (degrees)
    roll = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 180.0f / PI;
    pitch = asinf(constrain(2.0f * (q0*q2 - q3*q1), -1.0f, 1.0f)) * 180.0f / PI;
    yaw = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * 180.0f / PI;
  } else {
    // complementary filter
    roll = alpha * (roll + gyroRollRate * dt) + (1 - alpha) * accRoll;
    pitch = alpha * (pitch + gyroPitchRate * dt) + (1 - alpha) * accPitch;
    yaw += gyroYawRate * dt; // no magnetometer; will drift
  }
}

// Madgwick AHRS (IMU-only update)
void IMU::madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q0 = 2.0f * q0;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _4q0 = 4.0f * q0;
  float _4q1 = 4.0f * q1;
  float _4q2 = 4.0f * q2;
  float _8q1 = 8.0f * q1;
  float _8q2 = 8.0f * q2;
  float q0q0 = q0 * q0;
  float q1q1 = q1 * q1;
  float q2q2 = q2 * q2;
  float q3q3 = q3 * q3;

  // Normalize accelerometer measurement
  float recipNorm = sqrtf(ax * ax + ay * ay + az * az);
  if (recipNorm == 0.0f) return; // invalid data
  recipNorm = 1.0f / recipNorm;
  ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

  // Gradient descent algorithm corrective step
  float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
  float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
  float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
  float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

  // Normalize step magnitude
  float sNorm = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
  if (sNorm == 0.0f) return;
  s0 /= sNorm; s1 /= sNorm; s2 /= sNorm; s3 /= sNorm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;

  // Integrate to yield quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  // Normalize quaternion
  float recipQNorm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  if (recipQNorm == 0.0f) return;
  recipQNorm = 1.0f / recipQNorm;
  q0 *= recipQNorm; q1 *= recipQNorm; q2 *= recipQNorm; q3 *= recipQNorm;
}
