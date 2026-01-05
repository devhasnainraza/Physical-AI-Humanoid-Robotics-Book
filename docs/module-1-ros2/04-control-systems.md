---
id: control-systems
title: '1.4 ROS 2 Control'
sidebar_label: '1.4 Control'
description: 'Implementing PID and ros2_control framework.'
---

# 1.4 ROS 2 Control Framework

**"Don't write your own PID loop. Use the standard."**

In this lab, we will use `ros2_control`, the industrial standard for controlling robots in ROS 2. It handles the timing, resource management, and controller switching for you.

## 🎯 Lab Objectives
1.  **Configure `ros2_control`** in URDF.
2.  **Spawn a Joint Trajectory Controller**.
3.  **Command the robot** to move.

---

## 1.4.1 The Architecture

1.  **Controller Manager**: The boss. Loads/Unloads controllers.
2.  **Hardware Interface**: The driver (reads from `read()`, writes to `write()`).
3.  **Controllers**:
    *   `joint_trajectory_controller`: Follows a path (smooth).
    *   `velocity_controller`: Spins wheels.
    *   `effort_controller`: Sends raw torque.

---

## 1.4.2 Lab: Add Control to URDF

Add this to your `my_robot.urdf`:

```xml
<ros2_control name="RealRobot" type="system">
  <hardware>
    <plugin>mock_components/GenericSystem</plugin> <!-- Simulates hardware -->
  </hardware>
  <joint name="base_to_arm">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

---

## 1.4.3 Lab: Configuration (`controllers.yaml`)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

joint_trajectory_controller:
  ros__parameters:
    joints:
      - base_to_arm
    interface_name: position
```

---

## 1.4.4 Sending a Command

Once launched, you can move the robot by publishing a trajectory:

```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{joint_names: ["base_to_arm"], points: [{positions: [1.0], time_from_start: {sec: 1, nanosec: 0}}]}' -1
```

The arm will smoothly interpolate to 1.0 radian over 1 second.

---

## 1.4.5 Quiz

1.  **What is the role of the `hardware_interface`?**
    *   a) To run the PID loop.
    *   b) To abstract the physical hardware (read/write) from the controllers.
    *   *Answer: b*

2.  **Why use `joint_trajectory_controller` instead of raw position commands?**
    *   a) It ensures smooth motion (velocity/acceleration limits).
    *   b) It is faster.
    *   *Answer: a*