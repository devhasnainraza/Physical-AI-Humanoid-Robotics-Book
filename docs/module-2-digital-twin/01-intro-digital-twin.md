---
id: intro-digital-twin
title: '2.1 The Digital Twin'
sidebar_label: '2.1 Intro'
description: 'Why we simulate: Safety, Speed, and Data.'
---

# 2.1 The Digital Twin

**"Simulation is the dojo."**

Before you risk a $15,000 robot, you crash the $0 simulation. A Digital Twin is a physics-accurate replica of your robot in a virtual world.

## 🎯 Lab Objectives
1.  **Understand the 3 Sims**: Gazebo vs MuJoCo vs Isaac.
2.  **Install the Tools**.

---

## 2.1.1 The Landscape

| Simulator | Engine | Use Case |
| :--- | :--- | :--- |
| **Gazebo (Classic/Ignition)** | ODE/Dart | General Robotics, ROS 2 Integration. Best for SLAM/Nav. |
| **MuJoCo** | MJCF | Contact Physics. Best for RL/Locomotion. Fast. |
| **Isaac Lab (Omniverse)** | PhysX 5 | Photorealism, Parallel Training. Best for Sim-to-Real RL & Vision. |

We will use **Gazebo** for standard ROS work and **Isaac Lab** for RL.

---

## 2.1.2 Lab: Installation

```bash
# Install Gazebo Harmonic
sudo apt-get update
sudo apt-get install ros-humble-ros-gz
```

Verify installation:
`ign gazebo shapes.sdf`