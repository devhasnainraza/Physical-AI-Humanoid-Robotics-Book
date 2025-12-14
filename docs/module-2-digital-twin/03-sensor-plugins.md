---
sidebar_position: 3
title: Simulating Sensors (LiDAR & Depth)
---

# Simulating the Robot's Senses

A robot without sensors is blind. In Gazebo, we attach **Plugins** to the robot model to generate synthetic sensor data.

## 1. The Sensor Plugin Architecture

Sensors are defined inside a `<link>` in your URDF/SDF.

### 1.1 Adding a 2D LiDAR

The standard for navigation is a 2D LiDAR (like the RPLidar). It scans a slice of the world.

Add this to your robot's URDF/SDF (inside the `lidar_link`):

```xml
<sensor name="lidar" type="gpu_lidar">
  <pose>0 0 0 0 0 0</pose>
  <topic>/scan</topic>
  <update_rate>10</update_rate> <!-- 10 Hz -->
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples> <!-- 1 degree resolution -->
        <resolution>1</resolution>
        <min_angle>-3.14</min_angle>
        <max_angle>3.14</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.15</min>
      <max>10.0</max>
    </range>
  </ray>
  <always_on>true</always_on>
  <visualize>true</visualize> <!-- Draws blue rays in GUI -->
</sensor>
```

---

## 2. Simulating a Depth Camera (RealSense)

Humanoids need 3D vision. We simulate an RGB-D camera which produces two streams:
1.  **RGB Image**: `/camera/image_raw`
2.  **Depth Map**: `/camera/depth/image_raw` (Distance to each pixel)

```xml
<sensor name="camera" type="depth_camera">
  <pose>0.1 0 0.5 0 0 0</pose>
  <topic>/camera</topic>
  <camera>
    <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

### 2.1 The `ros_gz_bridge`

Gazebo produces messages in its own format (Ignition Transport). ROS 2 expects DDS messages. We need a bridge node to convert them.

**Command:**
```bash
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```

---

## 3. Sensor Noise: The Mathematics

Perfect sensors make for lazy AI. Real sensors are noisy.
We model noise using a **Gaussian Distribution**.

For a true distance D_true, the sensor reports D_measured:

```
D_measured = D_true + epsilon
epsilon ~ Gaussian(mean, variance)
```

Where:
-   mean = 0.0 (Unbiased error)
-   $\sigma$ (Standard Deviation) = 0.01 (1cm error)

In SDF:
```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.01</stddev> 
</noise>
```

### 3.1 Bias vs Noise
-   **Noise**: Random jitter (can be filtered by averaging).
-   **Bias**: Constant offset (e.g., sensor always reads +5cm).
-   **Drift**: Bias that changes over time (IMU drift).

Simulating **Drift** is crucial for testing VSLAM loop closure capabilities.