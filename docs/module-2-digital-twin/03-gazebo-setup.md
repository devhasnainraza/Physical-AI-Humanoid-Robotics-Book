---
id: gazebo-setup
title: '2.3 Gazebo Simulation'
sidebar_label: '2.3 Gazebo Setup'
description: 'Worlds, Plugins, and the ROS 2 Bridge.'
---

# 2.3 Gazebo Simulation

**"Let there be light (and physics)."**

## 🎯 Lab Objectives
1.  **Create a World** file.
2.  **Add Sensors** (Lidar/Camera) via Plugins.
3.  **Bridge** Gazebo to ROS 2.

---

## 2.3.1 The World File (`my_world.sdf`)

```xml
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane</uri>
    </include>
    
    <!-- Physics Settings -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
  </world>
</sdf>
```

---

## 2.3.2 Adding a Lidar Plugin

In your URDF/SDF:

```xml
<plugin
    filename="libignition-gazebo-sensors-system.so"
    name="ignition::gazebo::systems::Sensors">
    <render_engine>ogre2</render_engine>
</plugin>

<sensor name='lidar' type='gpu_lidar'>
    <topic>scan</topic>
    <update_rate>10</update_rate>
    <ray>
        <scan>
            <horizontal>
                <samples>640</samples>
                <resolution>1</resolution>
                <min_angle>-1.57</min_angle>
                <max_angle>1.57</max_angle>
            </horizontal>
        </scan>
        <range>
            <min>0.08</min>
            <max>10.0</max>
        </range>
    </ray>
</sensor>
```

---

## 2.3.3 The Bridge (`ros_gz_bridge`)

Gazebo talks "Ignition Transport". ROS talks "DDS". We need a translator.

```bash
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan
```
This maps the Gazebo `/scan` to ROS 2 `/scan`.