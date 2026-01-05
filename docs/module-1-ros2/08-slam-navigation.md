---
id: slam-navigation
title: '1.8 SLAM & Navigation Lab'
sidebar_label: '1.8 SLAM & Nav Lab'
description: 'Mapping with SLAM Toolbox and Navigating with Nav2.'
---

# 1.8 SLAM & Navigation Lab

**"Mapping the unknown."**

## 🎯 Lab Objectives
1.  **Run SLAM Toolbox** to map a room.
2.  **Save the Map**.
3.  **Run Nav2** to click-to-move.

---

## 1.8.1 Mapping (SLAM Toolbox)

1.  **Launch**:
    `ros2 launch slam_toolbox online_async_launch.py`
2.  **Drive**: Use keyboard teleop.
    `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
3.  **Visualize**: Open RViz. Add "Map" display. Topic `/map`.

You will see the map grow as you drive!

4.  **Save**:
    Open the "SLAMToolbox" panel in RViz and click "Save Map".
    Or: `ros2 run nav2_map_server map_saver_cli -f my_map`

---

## 1.8.2 Navigation (Nav2)

1.  **Launch**:
    `ros2 launch nav2_bringup bringup_launch.py map:=my_map.yaml`
2.  **Localize**:
    In RViz, use "2D Pose Estimate" to tell the robot where it is initially.
3.  **Go**:
    Use "2D Goal Pose" to click a destination.
    The robot will plan a path (Global Planner) and drive it (Local Planner), avoiding obstacles.

---

## 1.8.3 Quiz

1.  **Which package is used for modern 2D SLAM in ROS 2?**
    *   a) Gmapping.
    *   b) SLAM Toolbox.
    *   *Answer: b*

2.  **What does "2D Pose Estimate" do?**
    *   a) Sets the goal.
    *   b) Initializes the AMCL filter (tells the robot where it is).
    *   *Answer: b*