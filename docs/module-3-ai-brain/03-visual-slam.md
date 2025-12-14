---
sidebar_position: 3
title: Visual SLAM
---

# Visual SLAM: Seeing and Mapping

## 1. The Localization Problem

Before a robot can navigate, it must answer two questions:
1.  **Mapping**: What does the world look like?
2.  **Localization**: Where am I in that world?

**SLAM (Simultaneous Localization and Mapping)** solves both at once.

---

## 2. Visual vs LiDAR SLAM

| Feature | LiDAR SLAM (e.g., Slam Toolbox) | Visual SLAM (e.g., ORB-SLAM3) |
| :--- | :--- | :--- |
| **Input** | Laser Scan (2D/3D points) | Camera Images (Pixels) |
| **Features** | Corners, Walls | Textures, Edges, Objects |
| **Failure Mode** | Long corridors (geometric similarity) | Low light, motion blur, featureless white walls |
| **Cost** | High (LiDARs are expensive) | Low (Cameras are cheap) |

---

## 3. Epipolar Geometry: The Math of Stereo Vision

How do we get 3D depth ($Z$) from two 2D images?

Given two cameras with focal length $f$, separated by a baseline $b$, looking at a point $P$.
The disparity d is the difference in the x-coordinate of the point in the Left (xL) and Right (xR) images.

`d = xL - xR`

The depth Z is calculated as:

`Z = (f * b) / d`

**Implication**:
-   If d is small (point is far away), depth estimation is noisy.
-   If $b$ (baseline) is wider, depth accuracy improves at range, but close-up performance suffers.

---

## 4. Using `isaac_ros_visual_slam`

NVIDIA provides a highly optimized VSLAM package for the Jetson.

### 4.1 Launching VSLAM

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'enable_imu_fusion': True, # Use gyroscope for better stability
            'enable_rectified_pose': True
        }],
        remappings=[
            ('stereo_camera/left/image_rect', '/camera/left/image_rect'),
            ('stereo_camera/right/image_rect', '/camera/right/image_rect'),
            ('visual_slam/imu', '/camera/imu')
        ]
    )

    return LaunchDescription([
        ComposableNodeContainer(
            name='visual_slam_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[visual_slam_node],
            output='screen'
        )
    ])
```

### 4.2 Loop Closure

When the robot revisits a known location, accumulated drift error (Dead Reckoning) is corrected.
Mathematically, this is a **Pose Graph Optimization** problem.
We minimize the error:

`E = sum( (z_i - h(x_i))^2 )`

Where z_i is the sensor measurement and x_i is the estimated pose.