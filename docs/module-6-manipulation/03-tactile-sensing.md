--- 
id: tactile-sensing
title: '6.3 Lab: Tactile Feedback (Slip Detection)'
sidebar_label: '6.3 Lab: Tactile'
description: 'Simulating force sensors and detecting slip events.'
---

# 6.3 Lab: Tactile Feedback

**"Don't drop the egg."**

Humans adjust grip force automatically. If an object slips, we squeeze harder (Spinal Reflex). We will implement this.

## 🎯 Lab Objectives
1.  **Read Force Sensor** in Simulation.
2.  **Detect High-Frequency Noise** (Slip).
3.  **Reflex Controller**: $\Delta F = K \cdot \text{Slip}$.

---

## 6.3.1 Simulation

We simulate a "Frictionless" object that becomes "Frictional" only when squeezed.

```python
# PyBullet
force = p.getJointState(gripperId, jointIndex)[2] # Reaction Force

# FFT for Slip
# Slip generates vibration > 50Hz.
```

## 6.3.2 The Controller

```python
target_force = 5.0 # Newtons

while True:
    current_force = read_sensor()
    slip_signal = detect_slip()
    
    if slip_signal > threshold:
        target_force += 1.0 # Squeeze harder! 
        
    error = target_force - current_force
    motor_cmd = pid(error)
```

---

## 6.3.3 GelSight Simulation

Simulating optical tactile sensors is hard. We use **Tactile Image Generators** (rendering a heightmap based on penetration depth).

## 6.3.4 Quiz

1.  **How do we detect slip using only a force sensor?**
    *   a) Look for high-frequency vibrations (stick-slip phenomenon).
    *   b) We can't.
    *   *Answer: a*

2.  **What is the "Spinal Reflex" in robotics?**
    *   a) Low-latency control loop running on the MCU (1kHz) that reacts to slip without waiting for the main CPU.
    *   b) A back brace.
    *   *Answer: a*
