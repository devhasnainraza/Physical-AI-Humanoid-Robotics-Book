---
id: isaac-lab
title: '05. NVIDIA Isaac Lab'
sidebar_label: '05. NVIDIA Isaac Lab'
---

# 2.5 NVIDIA Isaac Lab (Omniverse)

## 2.5.1 The Future is USD

**Isaac Lab** (formerly Isaac Gym/Orbit) is built on **NVIDIA Omniverse**.
It uses **USD (Universal Scene Description)**, the file format created by Pixar for movies. USD allows for non-destructive editing, layering, and massive scale.

## 2.5.2 GPU-Accelerated Physics

In Gazebo/MuJoCo, the CPU calculates the physics.
In Isaac Lab, the **GPU** calculates everything.
*   **Parallelism**: You can simulate 4,096 robots on a single GPU.
*   **Rendering**: The visual data stays on the GPU memory, meaning you can train Vision-based RL agents without the "CPU-GPU bottleneck" of copying images.

## 2.5.3 Setting Up the Environment

Isaac Lab requires a powerful GPU (RTX 3070+ recommended).
1.  Install **Isaac Sim**.
2.  Clone **Isaac Lab**.
3.  Run the manager script.

## 2.5.4 Domain Randomization in Isaac

Isaac Lab makes randomization easy via the `Cfg` classes.

```python
@configclass
class MyRobotCfg:
    # Randomize friction
    friction_range = [0.5, 1.2]
    
    # Randomize mass
    mass_scale_range = [0.9, 1.1]
    
    # Add noise to observations
    obs_noise = 0.05
```

When you launch the training, Isaac generates 4,000 parallel environments, each with slightly different physics parameters sampled from these ranges. This creates a robust policy.
