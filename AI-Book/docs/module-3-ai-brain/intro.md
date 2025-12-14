---
sidebar_position: 1
title: The AI-Robot Brain
---

# Module 3: The AI-Robot Brain (NVIDIA Isaac)

## From Simulation to Intelligence

In this module, we move from basic movement to intelligent navigation using the NVIDIA Isaac ecosystem.

### NVIDIA Isaac Sim & Synthetic Data

Training AI models requires massive amounts of data. **Isaac Sim** allows us to generate "Synthetic Data"—labeled training images created by the simulation itself.
- **Domain Randomization**: Changing lights, textures, and object positions to make the AI robust.

### Isaac ROS: VSLAM & Navigation

We use GPU-accelerated libraries for the robot's spatial awareness:

1.  **VSLAM (Visual Simultaneous Localization and Mapping)**: The robot looks around and builds a map while figuring out where it is.
2.  **Nav2 (Navigation 2)**: The standard stack for moving from Point A to Point B while avoiding dynamic obstacles.

### Nav2 for Humanoids

Humanoid navigation is harder than wheeled navigation. We must account for:
- **Footstep Planning**: Where to place feet.
- **Balance**: Not falling over while turning.

<HardwareOnly profile="Unitree_Go2">

**Unitree Note**: The Go2 has built-in VSLAM, but running Isaac ROS on an attached Jetson Orin allows for custom, high-level navigation logic beyond the manufacturer's defaults.

</HardwareOnly>
