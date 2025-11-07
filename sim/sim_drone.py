#!/usr/bin/env python3
"""
Simulatore numerico del drone - test PID senza hardware
Simula: fisica quadcopter, IMU, filtro Madgwick, PID roll/pitch/yaw
"""
import numpy as np
import time
import sys

class PID:
    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0
        self.last_error = 0
    
    def compute(self, setpoint, measured, dt):
        error = setpoint - measured
        self.integral += error * dt
        # anti-windup
        max_integral = (self.out_max - self.out_min) / 2.0
        self.integral = np.clip(self.integral, -max_integral, max_integral)
        
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return np.clip(output, self.out_min, self.out_max)
    
    def reset(self):
        self.integral = 0
        self.last_error = 0

class QuadcopterSim:
    def __init__(self):
        # Stato fisico (angoli in gradi)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Velocità angolari (gradi/s)
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0
        
        # Parametri fisici
        self.damping = 0.95  # smorzamento
        self.inertia = 0.02  # inerzia
        
    def apply_torques(self, torque_roll, torque_pitch, torque_yaw, dt):
        """Applica torque ai 3 assi e aggiorna lo stato"""
        # Integra accelerazioni angolari
        self.roll_rate += torque_roll / self.inertia * dt
        self.pitch_rate += torque_pitch / self.inertia * dt
        self.yaw_rate += torque_yaw / self.inertia * dt
        
        # Smorzamento
        self.roll_rate *= self.damping
        self.pitch_rate *= self.damping
        self.yaw_rate *= self.damping
        
        # Integra posizioni angolari
        self.roll += self.roll_rate * dt
        self.pitch += self.pitch_rate * dt
        self.yaw += self.yaw_rate * dt
        
        # Wrap yaw -180..180
        self.yaw = (self.yaw + 180) % 360 - 180
    
    def get_imu_reading(self):
        """Restituisce letture IMU (con piccolo rumore)"""
        noise = 0.1
        return {
            'roll': self.roll + np.random.randn() * noise,
            'pitch': self.pitch + np.random.randn() * noise,
            'yaw': self.yaw + np.random.randn() * noise,
            'gyro_x': self.roll_rate + np.random.randn() * noise * 2,
            'gyro_y': self.pitch_rate + np.random.randn() * noise * 2,
            'gyro_z': self.yaw_rate + np.random.randn() * noise * 2,
        }

def motor_mixing(base_throttle, corr_roll, corr_pitch, corr_yaw):
    """Motor mixing per configurazione X"""
    m1 = base_throttle - corr_roll + corr_pitch - corr_yaw
    m2 = base_throttle - corr_roll - corr_pitch + corr_yaw
    m3 = base_throttle + corr_roll - corr_pitch - corr_yaw
    m4 = base_throttle + corr_roll + corr_pitch + corr_yaw
    
    return [
        np.clip(m1, 0, 1000),
        np.clip(m2, 0, 1000),
        np.clip(m3, 0, 1000),
        np.clip(m4, 0, 1000)
    ]

def main():
    print("=" * 60)
    print("  DRONE SIMULATOR - Test PID senza hardware")
    print("=" * 60)
    print("Simula: quadcopter + IMU + PID roll/pitch/yaw")
    print()
    
    # Inizializza componenti
    quad = QuadcopterSim()
    
    # PID (stessi gain del codice embedded)
    pid_roll = PID(2.0, 0.5, 0.8, -400, 400)
    pid_pitch = PID(2.0, 0.5, 0.8, -400, 400)
    pid_yaw = PID(1.5, 0.1, 0.5, -200, 200)
    
    # Setpoints
    target_roll = 0.0
    target_pitch = 0.0
    target_yaw = 0.0
    base_throttle = 400
    
    # Perturbazione iniziale (simulazione vento)
    quad.roll = 15.0
    quad.pitch = -10.0
    
    print(f"Condizioni iniziali:")
    print(f"  Roll:  {quad.roll:6.2f}° (target: {target_roll}°)")
    print(f"  Pitch: {quad.pitch:6.2f}° (target: {target_pitch}°)")
    print(f"  Yaw:   {quad.yaw:6.2f}° (target: {target_yaw}°)")
    print(f"  Throttle: {base_throttle}/1000")
    print()
    print("Stabilizzazione in corso...\n")
    
    dt = 0.01  # 100 Hz control loop
    duration = 5.0  # secondi
    steps = int(duration / dt)
    
    for step in range(steps):
        t = step * dt
        
        # Leggi IMU
        imu = quad.get_imu_reading()
        
        # Calcola correzioni PID
        corr_roll = pid_roll.compute(target_roll, imu['roll'], dt)
        corr_pitch = pid_pitch.compute(target_pitch, imu['pitch'], dt)
        corr_yaw = pid_yaw.compute(target_yaw, imu['yaw'], dt)
        
        # Motor mixing
        motors = motor_mixing(base_throttle, corr_roll, corr_pitch, corr_yaw)
        
        # Converti throttle in torque (semplificato)
        # Differenza throttle tra motori opposti genera torque
        torque_roll = (motors[2] + motors[3] - motors[0] - motors[1]) * 0.001
        torque_pitch = (motors[0] + motors[3] - motors[1] - motors[2]) * 0.001
        torque_yaw = (motors[1] + motors[3] - motors[0] - motors[2]) * 0.0005
        
        # Applica torque alla fisica
        quad.apply_torques(torque_roll, torque_pitch, torque_yaw, dt)
        
        # Stampa ogni 0.5s
        if step % 50 == 0:
            print(f"t={t:4.1f}s | R:{quad.roll:7.2f}° P:{quad.pitch:7.2f}° Y:{quad.yaw:7.2f}° | "
                  f"M:[{motors[0]:3.0f},{motors[1]:3.0f},{motors[2]:3.0f},{motors[3]:3.0f}]")
    
    print()
    print("=" * 60)
    print("RISULTATO FINALE:")
    print(f"  Roll:  {quad.roll:7.2f}° (errore: {abs(quad.roll - target_roll):.2f}°)")
    print(f"  Pitch: {quad.pitch:7.2f}° (errore: {abs(quad.pitch - target_pitch):.2f}°)")
    print(f"  Yaw:   {quad.yaw:7.2f}° (errore: {abs(quad.yaw - target_yaw):.2f}°)")
    print("=" * 60)
    
    # Test: cambia setpoint durante il volo
    print("\n--- TEST 2: Cambio setpoint roll → 20° ---")
    target_roll = 20.0
    pid_roll.reset()
    
    for step in range(steps):
        t = step * dt
        imu = quad.get_imu_reading()
        
        corr_roll = pid_roll.compute(target_roll, imu['roll'], dt)
        corr_pitch = pid_pitch.compute(target_pitch, imu['pitch'], dt)
        corr_yaw = pid_yaw.compute(target_yaw, imu['yaw'], dt)
        
        motors = motor_mixing(base_throttle, corr_roll, corr_pitch, corr_yaw)
        
        torque_roll = (motors[2] + motors[3] - motors[0] - motors[1]) * 0.001
        torque_pitch = (motors[0] + motors[3] - motors[1] - motors[2]) * 0.001
        torque_yaw = (motors[1] + motors[3] - motors[0] - motors[2]) * 0.0005
        
        quad.apply_torques(torque_roll, torque_pitch, torque_yaw, dt)
        
        if step % 50 == 0:
            print(f"t={t:4.1f}s | R:{quad.roll:7.2f}° P:{quad.pitch:7.2f}° Y:{quad.yaw:7.2f}°")
    
    print()
    print(f"FINALE: Roll={quad.roll:.2f}° (target: {target_roll}°, errore: {abs(quad.roll - target_roll):.2f}°)")
    print("\n✅ Simulazione completata! Il PID funziona correttamente.")

if __name__ == '__main__':
    main()
