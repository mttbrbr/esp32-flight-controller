#include <Arduino.h>
#include "imu.h"
#include "motor.h"
#include "pid.h"
#include <Wire.h>

IMU imu;
Motors motors;

// PID controllers for roll, pitch, yaw (attitude)
PID pidRoll(2.0, 0.5, 0.8, -400, 400);   // output: motor correction
PID pidPitch(2.0, 0.5, 0.8, -400, 400);
PID pidYaw(1.5, 0.1, 0.5, -200, 200);

// Flight mode and setpoints
bool stabilizationEnabled = false;
float setpointRoll = 0.0;   // target roll (degrees)
float setpointPitch = 0.0;  // target pitch
float setpointYaw = 0.0;    // target yaw
int baseThrottle = 0;       // base throttle (0-1000)

unsigned long lastPrint = 0;
unsigned long lastDetailedPrint = 0;
unsigned long lastControlLoop = 0;
bool verboseMode = true; // stampa dettagliata ogni 1s

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n========================================");
  Serial.println("   DRONE CONTROL - Numerical Test Mode");
  Serial.println("========================================");
  
  // Initialize I2C pins for ESP32 (SDA=21, SCL=22)
  Wire.begin(21, 22);
  Serial.println("[INIT] I2C initialized (SDA=21, SCL=22)");
  
  // enable Madgwick filter for better attitude estimation
  imu.setUseMadgwick(true);
  Serial.println("[INIT] Madgwick AHRS filter enabled");
  
  // Initialize IMU (MPU6050 over I2C)
  if (!imu.begin()) {
    Serial.println("[ERROR] IMU init failed - check wiring!");
  } else {
    Serial.println("[OK] IMU ready (MPU6050 @ 0x68)");
  }

  // Initialize motor pins (disarmed by default)
  motors.initPins();
  Serial.println("[OK] Motors initialized (DISARMED)");
  Serial.println("     Motor pins: 14(M1) 12(M2) 13(M3) 27(M4)");
  
  Serial.println("\n--- SERIAL COMMANDS ---");
  Serial.println("  a      = ARM motors");
  Serial.println("  d      = DISARM motors");
  Serial.println("  s      = Toggle STABILIZATION on/off");
  Serial.println("  t###   = Set base throttle 0-1000 (e.g., t500)");
  Serial.println("  r###   = Set roll setpoint -45..45 (e.g., r10)");
  Serial.println("  i###   = Set pitch setpoint -45..45 (e.g., i-5)");
  Serial.println("  y###   = Set yaw setpoint (e.g., y20)");
  Serial.println("  p      = Print RAW sensor values");
  Serial.println("  v      = Toggle verbose mode");
  Serial.println("  h      = Show this help");
  Serial.println("========================================\n");
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastControlLoop) / 1000000.0f;
  if (dt < 0.001f) dt = 0.001f;
  lastControlLoop = now;
  
  imu.update();

  // PID control loop (if stabilization enabled and armed)
  if (stabilizationEnabled && motors.isArmed()) {
    float roll = imu.getRoll();
    float pitch = imu.getPitch();
    float yaw = imu.getYaw();
    
    float corrRoll = pidRoll.compute(setpointRoll, roll, dt);
    float corrPitch = pidPitch.compute(setpointPitch, pitch, dt);
    float corrYaw = pidYaw.compute(setpointYaw, yaw, dt);
    
    // Motor mixing (quadcopter X configuration)
    // M1 (front-right) M2 (rear-right) M3 (rear-left) M4 (front-left)
    int m1 = baseThrottle - corrRoll + corrPitch - corrYaw;
    int m2 = baseThrottle - corrRoll - corrPitch + corrYaw;
    int m3 = baseThrottle + corrRoll - corrPitch - corrYaw;
    int m4 = baseThrottle + corrRoll + corrPitch + corrYaw;
    
    motors.setMotorThrottle(0, constrain(m1, 0, 1000));
    motors.setMotorThrottle(1, constrain(m2, 0, 1000));
    motors.setMotorThrottle(2, constrain(m3, 0, 1000));
    motors.setMotorThrottle(3, constrain(m4, 0, 1000));
  } else if (motors.isArmed() && !stabilizationEnabled) {
    // Manual mode: all motors same throttle
    motors.setAllThrottle(baseThrottle);
  }

  // simple serial commands
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s == "a") {
      motors.arm();
      pidRoll.reset();
      pidPitch.reset();
      pidYaw.reset();
      Serial.println("[CMD] ARMED - motors enabled, PIDs reset");
    } else if (s == "d") {
      motors.disarm();
      Serial.println("[CMD] DISARMED - motors disabled");
    } else if (s == "s") {
      stabilizationEnabled = !stabilizationEnabled;
      Serial.printf("[CMD] Stabilization: %s\n", stabilizationEnabled ? "ON 🎯" : "OFF");
      if (stabilizationEnabled) {
        pidRoll.reset();
        pidPitch.reset();
        pidYaw.reset();
      }
    } else if (s == "p") {
      auto acc = imu.getAccel();
      auto gyr = imu.getGyro();
      Serial.println("--- RAW SENSOR DATA ---");
      Serial.printf("  Accel: X=%.3f Y=%.3f Z=%.3f m/s²\n", acc.x, acc.y, acc.z);
      Serial.printf("  Gyro:  X=%.3f Y=%.3f Z=%.3f dps\n", gyr.x, gyr.y, gyr.z);
    } else if (s == "v") {
      verboseMode = !verboseMode;
      Serial.printf("[CMD] Verbose mode: %s\n", verboseMode ? "ON" : "OFF");
    } else if (s == "h") {
      Serial.println("\n--- COMMANDS ---");
      Serial.println("  a = ARM  |  d = DISARM  |  s = stabilization on/off");
      Serial.println("  t### = throttle  |  r### = roll  |  i### = pitch  |  y### = yaw");
      Serial.println("  p = raw  |  v = verbose  |  h = help\n");
    } else if (s.startsWith("t")) {
      long v = s.substring(1).toInt();
      baseThrottle = constrain(v, 0, 1000);
      Serial.printf("[CMD] Base throttle set to %d/1000\n", baseThrottle);
    } else if (s.startsWith("r")) {
      float v = s.substring(1).toFloat();
      setpointRoll = constrain(v, -45, 45);
      Serial.printf("[CMD] Roll setpoint: %.1f°\n", setpointRoll);
    } else if (s.startsWith("i")) {
      float v = s.substring(1).toFloat();
      setpointPitch = constrain(v, -45, 45);
      Serial.printf("[CMD] Pitch setpoint: %.1f°\n", setpointPitch);
    } else if (s.startsWith("y")) {
      float v = s.substring(1).toFloat();
      setpointYaw = v;
      Serial.printf("[CMD] Yaw setpoint: %.1f°\n", setpointYaw);
    } else if (s.length() > 0) {
      Serial.printf("[WARN] Unknown command: '%s' (type 'h' for help)\n", s.c_str());
    }
  }

  // print attitude periodically (fast mode)
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    float roll = imu.getRoll();
    float pitch = imu.getPitch();
    float yaw = imu.getYaw();
    if (!verboseMode) {
      Serial.printf("R:%6.2f P:%6.2f Y:%6.2f | Stab:%d Armed:%d Thr:%d\n", 
                    roll, pitch, yaw, stabilizationEnabled, motors.isArmed(), baseThrottle);
    }
  }

  // detailed print every 1 second (verbose mode)
  if (verboseMode && (millis() - lastDetailedPrint >= 1000)) {
    lastDetailedPrint = millis();
    float roll = imu.getRoll();
    float pitch = imu.getPitch();
    float yaw = imu.getYaw();
    auto acc = imu.getAccel();
    
    Serial.println("\n========== STATUS ==========");
    Serial.printf("  Roll:   %7.2f° (target: %.1f°)\n", roll, setpointRoll);
    Serial.printf("  Pitch:  %7.2f° (target: %.1f°)\n", pitch, setpointPitch);
    Serial.printf("  Yaw:    %7.2f° (target: %.1f°)\n", yaw, setpointYaw);
    Serial.printf("  Accel:  [%.2f, %.2f, %.2f] m/s²\n", acc.x, acc.y, acc.z);
    Serial.printf("  Stabilization: %s\n", stabilizationEnabled ? "ON 🎯" : "OFF");
    Serial.printf("  Motors: %s\n", motors.isArmed() ? "ARMED ⚠️" : "DISARMED ✓");
    Serial.printf("  Base Throttle: %d/1000\n", baseThrottle);
    Serial.printf("  Uptime: %.1f s\n", millis() / 1000.0);
    Serial.println("============================\n");
  }
}
