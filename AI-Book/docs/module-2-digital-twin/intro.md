---
sidebar_position: 1
title: The Digital Twin
---

# Module 2: The Digital Twin

## Gazebo & Unity for Physical AI

Before we risk expensive hardware, we simulate. A **Digital Twin** is a high-fidelity virtual replica of your robot and its environment.

### Physics Simulation

We simulate physical laws to test stability and control:
- **Gravity**: ensuring the robot can stand.
- **Collisions**: verifying path planning doesn't hit walls.
- **Friction**: testing walking on different surfaces.

### Sensor Simulation

We simulate the inputs the AI brain will receive:

| Sensor | Purpose | Simulated Plugin |
| :--- | :--- | :--- |
| **LiDAR** | 360° obstacle detection | `libgazebo_ros_velodyne_laser.so` |
| **Depth Camera** | 3D perception (RGB-D) | `libgazebo_ros_openni_kinect.so` |
| **IMU** | Balance & orientation | `libgazebo_ros_imu.so` |

## High-Fidelity Visualization

While Gazebo provides the physics, **Unity** or **NVIDIA Isaac Sim** provides the photorealistic visuals needed to train Vision-Language Models (VLMs).

![Simulation Architecture Placeholder](/img/undraw_docusaurus_react.svg)
