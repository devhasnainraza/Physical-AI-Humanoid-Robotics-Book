---
id: lab-2-urdf-modeling
title: 'Lab 2: URDF Modeling and Visualization'
sidebar_label: 'Lab 2: URDF Modeling'
---

# Lab 2: URDF Modeling and Visualization

## Objective
This lab aims to provide hands-on experience with **URDF (Unified Robot Description Format)** and **Xacro (XML Macros)**. You will construct a multi-joint robotic arm, define its kinematics and visualization properties, and then visualize it in RViz, the ROS 2 visualization tool.

## Theoretical Background
**URDF** is an XML format for describing the kinematic and dynamic properties of a robot. It defines the robot's links (rigid bodies) and joints (connections between links).
**Xacro** extends URDF by allowing you to use macros, properties, and basic programming constructs to create more concise and reusable robot descriptions. This is crucial for complex robots like humanoids.

*   **Link**: A rigid body with physical properties (mass, inertia) and visual/collision geometries.
*   **Joint**: Defines the kinematic relationship between two links (parent and child). Types include `revolute` (rotating), `prismatic` (sliding), and `fixed`.
*   **Xacro Properties**: Variables that can be defined and reused throughout the Xacro file.
*   **Xacro Macros**: Reusable blocks of XML code, similar to functions, which can take parameters.

## Prerequisites
*   **ROS 2 Humble Development Environment**: Set up as in Lab 1.
*   **`urdf_tutorial` package**: Installed.
    ```bash
    sudo apt install ros-humble-urdf-tutorial
    ```

## Step-by-Step Instructions

### Step 1: Create a New Package for Your Robot Description
Navigate to your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_robot_description
```
We use `ament_cmake` here because robot description packages typically involve `.xacro` files that need to be processed by CMake.

### Step 2: Create the Xacro File
Navigate into your new package's directory:
```bash
cd ~/ros2_ws/src/my_robot_description
mkdir urdf rviz
```
Create a file named `urdf/simple_arm.xacro` and add the following content. This Xacro file will describe a simple 2-DOF robotic arm.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_arm">

  <!-- ======================= -->
  <!-- Xacro Properties -->
  <!-- ======================= -->
  <xacro:property name="PI" value="3.1415926535897931" />
  <xacro:property name="arm_width" value="0.05" />
  <xacro:property name="link1_length" value="0.3" />
  <xacro:property name="link2_length" value="0.25" />
  <xacro:property name="joint_limit" value="1.57" /> <!-- 90 degrees in radians -->

  <!-- ======================= -->
  <!-- Materials -->
  <!-- ======================= -->
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>
  <material name="black">
    <color rgba="0 0 0 1"/>
  </material>

  <!-- ======================= -->
  <!-- BASE LINK -->
  <!-- ======================= -->
  <link name="base_link">
    <visual>
      <geometry><cylinder length="0.05" radius="0.08"/></geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry><cylinder length="0.05" radius="0.08"/></geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- ======================= -->
  <!-- JOINT 1: Revolute -->
  <!-- ======================= -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/>
    <axis xyz="0 0 1"/> <!-- Rotation around Z-axis -->
    <limit lower="-${joint_limit}" upper="${joint_limit}" effort="100" velocity="10"/>
  </joint>

  <!-- ======================= -->
  <!-- LINK 1 -->
  <!-- ======================= -->
  <link name="link1">
    <visual>
      <geometry><box size="${arm_width} ${arm_width} ${link1_length}"/></geometry>
      <origin xyz="0 0 ${link1_length/2}" rpy="0 0 0"/>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry><box size="${arm_width} ${arm_width} ${link1_length}"/></geometry>
      <origin xyz="0 0 ${link1_length/2}" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- ======================= -->
  <!-- JOINT 2: Revolute -->
  <!-- ======================= -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 ${link1_length}" rpy="0 0 0"/> <!-- At the end of Link 1 -->
    <axis xyz="0 1 0"/> <!-- Rotation around Y-axis -->
    <limit lower="-${joint_limit}" upper="${joint_limit}" effort="100" velocity="10"/>
  </joint>

  <!-- ======================= -->
  <!-- LINK 2 (End-effector) -->
  <!-- ======================= -->
  <link name="link2">
    <visual>
      <geometry><box size="${arm_width} ${arm_width} ${link2_length}"/></geometry>
      <origin xyz="0 0 ${link2_length/2}" rpy="0 0 0"/>
      <material name="white"/>
    </visual>
    <collision>
      <geometry><box size="${arm_width} ${arm_width} ${link2_length}"/></geometry>
      <origin xyz="0 0 ${link2_length/2}" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- ======================= -->
  <!-- GRIPPER LINK (Fixed) -->
  <!-- ======================= -->
  <joint name="gripper_joint" type="fixed">
    <parent link="link2"/>
    <child link="gripper"/>
    <origin xyz="0 0 ${link2_length}" rpy="0 0 0"/>
  </joint>

  <link name="gripper">
    <visual>
      <geometry><box size="0.02 0.08 0.02"/></geometry>
      <origin xyz="0 0 0.01" rpy="0 0 0"/>
      <material name="black"/>
    </visual>
  </link>

</robot>
```

### Step 3: Configure CMakeLists.txt and Package.xml
**Update `~/ros2_ws/src/my_robot_description/CMakeLists.txt`**:
Add the following lines to ensure your `xacro` file is installed and available.

```cmake
# Add these lines at the end of the file, after find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

# For Xacro processing during visualization (optional, but good practice)
find_package(xacro REQUIRED)
```

**Update `~/ros2_ws/src/my_robot_description/package.xml`**:
Add these dependencies:
```xml
  <depend>xacro</depend>
  <depend>urdf_tutorial</depend>
```

### Step 4: Build Your Package
Navigate back to your workspace root and build your package:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_description
```

### Step 5: Source and Visualize in RViz
Source your workspace to make the package contents available:
```bash
. install/setup.bash
```
Now, launch the RViz visualization:
```bash
ros2 launch urdf_tutorial display.launch.py model:=$(ros2 pkg prefix my_robot_description)/share/my_robot_description/urdf/simple_arm.xacro
```
This command processes your `simple_arm.xacro` file into a URDF and then uses `urdf_tutorial`'s launch file to display it in RViz. A GUI window with sliders for `joint1` and `joint2` will appear. Move the sliders to articulate your robotic arm!

## Verification
*   RViz window opens and displays a 2-DOF robotic arm.
*   Sliders in the Joint State Publisher GUI control the `joint1` and `joint2` angles.
*   The gripper (fixed link) moves along with `link2`.

## Challenge Questions
1.  Add a third revolute joint and link to the arm (e.g., a wrist joint). How do the kinematics change?
2.  Modify the `inertial` properties of `link1` and `link2`. How do these affect simulation in a physics engine (though not visible in RViz directly)?
3.  Implement a Xacro macro for a generic arm segment that takes length and mass as parameters, then use it to define your arm.
