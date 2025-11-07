# Simulatore Drone

Simulatore Python per testare il codice del drone senza hardware.

## Cosa simula

- **Fisica del quadcopter** (roll/pitch/yaw, inerzia, smorzamento)
- **IMU virtuale** (letture con rumore realistico)
- **PID controllers** (stessi gain del codice embedded)
- **Motor mixing** (configurazione X)
- **Perturbazioni** (vento iniziale, cambio setpoint)

## Installazione

```bash
cd sim
pip3 install --user -r requirements.txt
```

## Esecuzione

```bash
python3 sim_drone.py
```

## Output esempio

```
t= 0.0s | R:  15.00° P: -10.00° Y:   0.00° | M:[450,350,350,450]
t= 0.5s | R:   8.23° P:  -5.12° Y:   0.15° | M:[425,375,375,425]
t= 1.0s | R:   3.45° P:  -1.89° Y:  -0.08° | M:[410,390,390,410]
t= 1.5s | R:   1.02° P:  -0.45° Y:   0.03° | M:[402,398,398,402]
t= 2.0s | R:   0.12° P:  -0.05° Y:   0.00° | M:[400,400,400,400]
```

Vedrai il drone stabilizzarsi da una perturbazione iniziale (roll=15°, pitch=-10°) verso i target (0°, 0°, 0°) in circa 2 secondi.

## Cosa verificare

- **Convergenza**: il drone raggiunge i setpoint?
- **Overshoot**: quanto supera il target prima di stabilizzarsi?
- **Settling time**: quanto tempo per stabilizzarsi?
- **Motor values**: i 4 motori si bilanciano correttamente?
