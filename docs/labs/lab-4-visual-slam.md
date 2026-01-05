--- 
id: lab-4-visual-slam
title: 'Lab 4: Visual SLAM for Environment Mapping'
sidebar_label: 'Lab 4: Visual SLAM'
---

# Lab 4: Visual SLAM for Environment Mapping

## Objective
This lab focuses on **Visual SLAM (Simultaneous Localization and Mapping)**. You will launch a simulated robot with a stereo camera in Gazebo, use the NVIDIA Isaac ROS Visual SLAM package to estimate the robot's pose and build a sparse map of the environment, and visualize the results in RViz.

## Theoretical Background
**Visual SLAM** solves the problem of simultaneously building a map of an unknown environment and localizing the robot within that map using only camera imagery. For a detailed theoretical understanding, refer to **Chapter 8: SLAM and Navigation**.

*   **Stereo Camera**: Provides two image streams (left and right) that allow for triangulation and direct depth estimation, improving robustness over monocular VSLAM.
*   **Feature Matching**: Identifying and tracking unique points across multiple camera frames to estimate ego-motion.
*   **Pose Graph Optimization**: A backend optimization process that reduces accumulated errors, especially during loop closures.

## Prerequisites
*   **ROS 2 Humble Development Environment**: Set up as in Lab 1.
*   **NVIDIA Isaac ROS**: Requires a Jetson Orin device or an NVIDIA GPU on your workstation with a compatible Isaac ROS Docker container setup. For this lab, we will use a pre-configured Docker image (or assume a workstation with GPU and Isaac ROS installed).
*   **Gazebo Harmonic (Ignition)**: Installed with ROS 2.
    ```bash
    sudo apt install ros-humble-ros-gz
    ```

## Step-by-Step Instructions

### Step 1: Prepare Your Workspace for Isaac ROS
Due to the dependencies and GPU requirements of Isaac ROS, it's often easiest to work within a dedicated Docker container.
1.  **Pull Isaac ROS Container**:
    ```bash
    docker pull nvcr.io/nvidia/isaac-ros-dev:humble
    ```
2.  **Launch Container (if on native Ubuntu/Jetson)**:
    If you are on an NVIDIA Jetson or a native Ubuntu machine with an NVIDIA GPU, you can launch the container directly. Skip the DevContainer and launch it from your host machine.
    ```bash
    docker run -it --rm --network host --runtime nvidia \
        -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
        -v /path/to/this/book/project:/workspaces/cortex-h1 \
        nvcr.io/nvidia/isaac-ros-dev:humble bash
    # Replace /path/to/this/book/project with your actual path
    ```
    *Note: If you are already in a Dev Container based on `ros-humble`, you might need to ensure it has GPU access and the necessary Isaac ROS packages. For simplicity, we will assume you are either in a suitable Isaac ROS container or a native setup.*

### Step 2: Create a Gazebo World with a Stereo Camera Robot
We will launch a simple robot with a stereo camera.
1.  Create a new ROS 2 package:
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_cmake my_slam_sim
    cd my_slam_sim
    ```
2.  Create a URDF for a simple robot with a stereo camera (`urdf/stereo_robot.urdf`):
    ```xml
    <?xml version="1.0"?>
    <robot name="stereo_robot">
      <link name="base_link">
        <visual><geometry><box size="0.2 0.2 0.1"/></geometry></visual>
        <collision><geometry><box size="0.2 0.2 0.1"/></geometry></collision>
        <inertial><mass value="1.0"/><inertia ixx="0.01" iyy="0.01" izz="0.01"/></inertial>
      </link>

      <joint name="camera_joint" type="fixed">
        <parent link="base_link"/>
        <child link="camera_link"/>
        <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
      </joint>

      <link name="camera_link">
        <visual><geometry><box size="0.05 0.1 0.05"/></geometry></visual>
        <collision><geometry><box size="0.05 0.1 0.05"/></geometry></collision>
        <inertial><mass value="0.01"/><inertia ixx="0.0001" iyy="0.0001" izz="0.0001"/></inertial>
        <sensor name="stereo_camera" type="multicamera">
          <always_on>true</always_on>
          <update_rate>10.0</update_rate>
          <camera name="left_camera">
            <horizontal_fov>1.047</horizontal_fov>
            <image><width>640</width><height>480</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>100</far></clip>
            <pose>0 -0.05 0 0 0 0</pose> <!-- Left camera slightly to the left -->
          </camera>
          <camera name="right_camera">
            <horizontal_fov>1.047</horizontal_fov>
            <image><width>640</width><height>480</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>100</far></clip>
            <pose>0 0.05 0 0 0 0</pose> <!-- Right camera slightly to the right -->
          </camera>
          <plugin name="camera_controller" filename="libgazebo_ros_multicamera.so">
            <cameraName>stereo_camera</cameraName>
            <alwaysOn>true</alwaysOn>
            <updateRate>10.0</updateRate>
            <imageTopicName>image_raw</imageTopicName>
            <cameraInfoTopicName>camera_info</cameraInfoTopicName>
            <frameName>camera_link</frameName>
            <hack_baseline>0.1</hack_baseline>
            <tf_prefix>stereo_camera</tf_prefix>
          </plugin>
        </sensor>
      </link>
    </robot>
    ```
3.  Create a launch file to spawn the robot in Gazebo (`launch/stereo_sim.launch.py`):
    ```python
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node
    from launch.substitutions import LaunchConfiguration

    def generate_launch_description():
        pkg_name = 'my_slam_sim'
        pkg_share_dir = get_package_share_directory(pkg_name)
        
        use_sim_time = LaunchConfiguration('use_sim_time', default='true')

        robot_description_path = os.path.join(pkg_share_dir, 'urdf', 'stereo_robot.urdf')
        
        # Gazebo launch
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-r empty.sdf'}.items() # -r starts paused
        )

        # Robot State Publisher (publishes TF from URDF)
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': open(robot_description_path).read()}],
            arguments=[robot_description_path] # Pass URDF directly
        )

        # Spawn robot in Gazebo
        spawn_entity_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'stereo_robot', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.5'],
            output='screen'
        )

        # Bridge Gazebo images to ROS 2 topics
        # Note: Gazebo plugin should already publish to ROS 2. This is a fallback/additional bridge.
        bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/stereo_camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/stereo_camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ],
            output='screen'
        )

        return LaunchDescription([
            DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
            gazebo_launch,
            robot_state_publisher_node,
            spawn_entity_node,
            bridge_node
        ])
    ```
4.  Update `my_slam_sim/CMakeLists.txt` and `my_slam_sim/package.xml`:
    *   **CMakeLists.txt**: Add `install(DIRECTORY urdf launch DESTINATION share/${PROJECT_NAME})`
    *   **package.xml**: Add `ros_gz_sim`, `robot_state_publisher`, `ros_gz_bridge`.

### Step 3: Build and Launch Simulation
```bash
cd ~/ros2_ws
colcon build --packages-select my_slam_sim
. install/setup.bash
ros2 launch my_slam_sim stereo_sim.launch.py
```
This should open Gazebo with your robot.

### Step 4: Launch Isaac ROS Visual SLAM
In a new terminal (within your Isaac ROS environment/container):
```bash
. install/setup.bash # Source your workspace
ros2 launch isaac_ros_visual_slam visual_slam.launch.py \
    # Map topics from our simulation to VSLAM
    left_image_topic:=/stereo_camera/left/image_raw \
    left_camera_info_topic:=/stereo_camera/left/camera_info \
    right_image_topic:=/stereo_camera/right/image_raw \
    right_camera_info_topic:=/stereo_camera/right/camera_info \
    # Set to true for a simple empty world mapping
    enable_imu_fusion:=false \
    # Isaac ROS will publish its output here
    publish_tf:=true \
    use_sim_time:=true
```

### Step 5: Visualize SLAM Output in RViz
Open another terminal, source your workspace, and launch RViz:
```bash
. install/setup.bash
rviz2 -d $(ros2 pkg prefix my_slam_sim)/share/my_slam_sim/rviz/slam.rviz # You'll create this file
```
*   **Create `rviz/slam.rviz`**: Save the current RViz configuration (`File -> Save Config As`) after setting up the following displays:
    *   `TF`: To see the robot's pose.
    *   `Image`: For `/stereo_camera/left/image_raw`.
    *   `PointCloud2`: For `/visual_slam/map` (or similar map topic from Isaac ROS).
    *   `Path`: For `/visual_slam/tracking/odometry` to see the estimated path.

### Step 6: Drive the Robot and Build Map
1.  In Gazebo, make sure the simulation is running (`play` button if paused).
2.  Use the Gazebo GUI controls to drive your stereo robot around the empty world.
3.  Observe the RViz window. You should see:
    *   The camera image stream.
    *   The robot's pose moving according to your control.
    *   A sparse point cloud map building up in real-time.
    *   The estimated path of the robot.

## Verification
*   Gazebo window with stereo robot appears.
*   Isaac ROS Visual SLAM launches without errors.
*   RViz displays robot pose, camera images, and a point cloud map being built as the robot moves.
*   The `/visual_slam/tracking/odometry` topic should show changes in pose.

## Challenge Questions
1.  **Map Density**: Add some simple textured boxes or cylinders to your Gazebo world (`.sdf` file). How does this affect the density and accuracy of the generated point cloud map?
2.  **IMU Fusion**: Modify the `visual_slam.launch.py` to enable IMU fusion (`enable_imu_fusion:=true`) and ensure your robot's URDF includes an IMU sensor with a corresponding topic. How does IMU data help stabilize the map and pose estimation, especially during fast movements?
3.  **Loop Closure**: Drive the robot in a loop (e.g., a square path) and return to its starting point. Does the map "snap" together? If not, what parameters in VSLAM might you tune to improve loop closure detection?
