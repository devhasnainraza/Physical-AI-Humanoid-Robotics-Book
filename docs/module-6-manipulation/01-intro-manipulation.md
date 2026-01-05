---
id: intro-manipulation
title: '6.1 Lab: Manipulation (MoveIt 2)'
sidebar_label: '6.1 Lab: MoveIt'
description: 'Kinematics and Motion Planning for a 7-DOF arm.'
---

# 6.1 Lab: Manipulation (MoveIt 2)

**"Pick and Place."**

MoveIt 2 is the standard library for manipulation. It wraps IK (KDL/Trac-IK), Collision Checking (FCL), and Planning (OMPL) into a nice ROS 2 interface.

## 🎯 Lab Objectives
1.  **Launch MoveIt** for a Panda Arm.
2.  **Move to Pose** (Cartesian).
3.  **Move to Joint** (Configuration).

---

## 6.1.1 Installation

```bash
sudo apt install ros-humble-moveit ros-humble-moveit-planners
sudo apt install ros-humble-panda-moveit-config
```

## 6.1.2 The Python Interface

MoveIt provides a python wrapper `moveit_py`.

```python
import rclpy
from moveit.planning import MoveItPy

def main():
    rclpy.init()
    
    # 1. Load Robot
    panda = MoveItPy(node_name="moveit_py")
    arm = panda.planning_component("panda_arm")
    
    # 2. Plan to Goal (Pose)
    arm.set_goal_state(pose_stamped_msg)
    plan_result = arm.plan()
    
    # 3. Execute
    if plan_result:
        arm.execute()
        
    rclpy.shutdown()
```

---

## 6.1.3 Visualizing in RViz

Launch the demo:
`ros2 launch panda_moveit_config demo.launch.py`

You will see the "Motion Planning" plugin. Drag the interactive marker to move the ghost arm, then click "Plan & Execute".

---

## 6.1.4 Quiz

1.  **What solver does MoveIt use for IK by default?**
    *   a) KDL (Kinematics and Dynamics Library).
    *   b) Magic.
    *   *Answer: a*

2.  **What is "OMPL"?**
    *   a) Open Motion Planning Library (contains RRT, PRM).
    *   b) A sensor.
    *   *Answer: a*
