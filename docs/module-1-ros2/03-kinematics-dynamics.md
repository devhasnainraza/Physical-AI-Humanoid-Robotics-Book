---
id: kinematics-dynamics
title: '1.3 URDF & TF2'
sidebar_label: '1.3 Kinematics (URDF)'
description: 'Building the robot model and managing coordinate frames.'
---

# 1.3 Kinematics in ROS 2: URDF & TF2

**"If you don't know where your hand is, you can't pick up the cup."**

In this lab, we define the robot's physical structure (URDF) and track its moving parts (TF2).

## 🎯 Lab Objectives
1.  **Write a URDF** for a 2-joint arm.
2.  **Visualize it** in RViz.
3.  **Broadcast TFs** using `robot_state_publisher`.

---

## 1.3.1 URDF (Unified Robot Description Format)

URDF is an XML file that describes:
*   **Links**: The rigid parts (bones).
*   **Joints**: The moving parts (hinges).

### Lab: Create `my_robot.urdf`

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Arm Link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Joint connecting Base and Arm -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.05"/>
    <axis xyz="0 1 0"/> <!-- Rotates around Y -->
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

</robot>
```

---

## 1.3.2 Robot State Publisher

How does ROS know where `arm_link` is relative to `base_link`?
*   It reads the URDF.
*   It subscribes to `/joint_states` (Current angle of the joint).
*   It performs Forward Kinematics.
*   It broadcasts the result to `/tf`.

### Launching the Visualization

Create a launch file `display.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    urdf_file = 'my_robot.urdf'
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
        ),
    ])
```

**Run it**: You will see a GUI slider. Move it, and the red arm rotates in 3D!

---

## 1.3.3 TF2: The Time Machine

TF2 is a library that tracks coordinate frames over time.
*   **Buffer**: Stores the last 10 seconds of transforms.
*   **Query**: "Where was the camera frame relative to the hand frame 2 seconds ago?"

**Why "Time"?**
Sensor data is old by the time you get it.
*   Camera sees a cup at $t=100ms$.
*   Robot moves.
*   At $t=150ms$, you try to grab it.
*   TF2 calculates exactly where the cup was at $t=100ms$ and transforms it to where the hand is *now*.

### Python TF Listener Example

```python
from tf2_ros import TransformListener, Buffer

# In your node
self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)

# Look up transform
try:
    t = self.tf_buffer.lookup_transform(
        'base_link',
        'camera_link',
        rclpy.time.Time()) # Get latest
    self.get_logger().info(f'Trans: {t.transform.translation}')
except Exception as e:
    self.get_logger().info(f'Could not find transform: {e}')
```

## 1.3.4 Quiz

1.  **What publishes to `/tf`?**
    *   a) `robot_state_publisher`.
    *   b) The motor.
    *   c) The camera.
    *   *Answer: a (mostly)*

2.  **Why do we need a `joint_state_publisher`?**
    *   a) To tell ROS the current angles of the joints so FK can be computed.
    *   b) To drive the robot.
    *   *Answer: a*