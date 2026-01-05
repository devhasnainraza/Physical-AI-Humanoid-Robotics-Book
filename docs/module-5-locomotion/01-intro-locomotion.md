---
id: intro-locomotion
title: '5.1 Lab: Locomotion Simulation'
sidebar_label: '5.1 Lab: PyBullet'
description: 'Setting up PyBullet and loading a Quadruped URDF.'
---

# 5.1 Lab: Locomotion Simulation (PyBullet)

**"First we crawl, then we walk."**

For locomotion, we need a fast, rigid-body physics engine. **PyBullet** is the standard for research because it is easy to install and mathematically robust.

## 🎯 Lab Objectives
1.  **Install PyBullet**.
2.  **Load a Unitree A1** (Quadruped).
3.  **Control Joints** directly.

---

## 5.1.1 Setup

```bash
pip install pybullet numpy
```

## 5.1.2 The Simulation Loop

Create `sim_quadruped.py`:

```python
import pybullet as p
import pybullet_data
import time

# 1. Connect
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# 2. Load Ground and Robot
planeId = p.loadURDF("plane.urdf")
# We use a built-in quadruped (Laikago) as a proxy for Unitree A1
robotId = p.loadURDF("laikago/laikago.urdf", [0, 0, 0.5])

# 3. Simulation Loop
while True:
    p.stepSimulation()
    
    # Simple PD Control (Stand Up)
    # Set target position for hip/knee joints
    targetPos = 0.5
    for i in range(12): # 12 Joints
        p.setJointMotorControl2(robotId, i, p.POSITION_CONTROL, targetPosition=targetPos, force=50)
        
    time.sleep(1./240.) # Real-time sync

p.disconnect()
```

## 5.1.3 Understanding the URDF
The `laikago.urdf` has 12 joints (3 per leg).
*   **Hip Roll** (Abduction/Adduction).
*   **Hip Pitch** (Forward/Back).
*   **Knee Pitch** (Extension).

To make it walk, we need to coordinate these 12 motors.
