---
id: perception-sensor-fusion
title: '1.5 Perception & Sensor Fusion'
sidebar_label: '1.5 Sensor Fusion'
description: 'Using robot_localization (EKF) to fuse IMU and Odometry.'
---

# 1.5 Perception & Sensor Fusion

**"One sensor lies. Two sensors negotiate."**

Your wheel encoders slip. Your IMU drifts. In this lab, we use an **Extended Kalman Filter (EKF)** to merge them into a robust `odom` estimate.

## 🎯 Lab Objectives
1.  **Install `robot_localization`**.
2.  **Configure the EKF** yaml.
3.  **Fuse** noisy data.

---

## 1.5.1 The Setup

*   **Input 1**: `/wheel/odom` (Twist). Good linear velocity, bad rotation.
*   **Input 2**: `/imu/data` (Angular Velocity + Acceleration). Good rotation, drift in position.

## 1.5.2 Lab: EKF Configuration (`ekf.yaml`)

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true # We are on the ground (x, y, yaw)
    publish_tf: true
    
    # Wheel Odometry
    odom0: /wheel/odom
    # x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  false, false,
                   false, false, true,
                   false, false, false]
                   
    # IMU
    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, true,  # Yaw
                  false, false, false,
                  false, false, true,  # Yaw Velocity
                  true,  false, false] # X Accel
```

## 1.5.3 Running the Filter

```bash
ros2 launch robot_localization ekf.launch.py
```

Now, plot `/odometry/filtered` vs `/wheel/odom` in **PlotJuggler**.
You will see that `/odometry/filtered` is much smoother and doesn't jump when the wheels slip.

---

## 1.5.4 Quiz

1.  **What does `two_d_mode: true` do?**
    *   a) Ignores Z position, Roll, and Pitch.
    *   b) Makes the robot 2D.
    *   *Answer: a*

2.  **Why do we fuse IMU with Wheel Odometry?**
    *   a) To get better Gyro (Rotation) estimates, which wheels are bad at.
    *   b) To look cool.
    *   *Answer: a*