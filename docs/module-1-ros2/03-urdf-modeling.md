---
sidebar_position: 3
title: Modeling Robots with URDF
---

# URDF: The Body Schema of a Robot

## 1. What is URDF?

**URDF (Unified Robot Description Format)** is an XML file format used in ROS to describe all elements of a robot:
-   **Links**: The rigid parts (bones) like a shin or thigh.
-   **Joints**: The moving parts (connectors) like a knee or hip.
-   **Visuals**: What it looks like (meshes).
-   **Collisions**: Simple shapes used for physics calculations.

## 2. The Physics of Inertia

To simulate a robot accurately, Gazebo needs to know how hard it is to rotate a limb. This is the **Moment of Inertia (Tensor)**.

### 2.1 Calculating Inertia for a Box

For a solid cuboid (width w, depth d, height h) of mass m:

```
Ixx = 1/12 * m * (h^2 + d^2)
Iyy = 1/12 * m * (w^2 + h^2)
Izz = 1/12 * m * (w^2 + d^2)
```

If you put `ixx="1.0"` for a 100g finger, the simulation will explode because the physics engine sees a "black hole density" object.

---

## 3. Anatomy of a Leg

Let's model a simple 2-DOF (Degree of Freedom) leg. It has a **Hip** joint and a **Knee** joint.

### 3.1 The XML Structure

Create a file named `simple_leg.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_leg">

  <!-- MATERIALS -->
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>

  <!-- BASE LINK (The Hip Bone) -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <!-- UPPER LEG LINK (Thigh) -->
  <link name="upper_leg">
    <visual>
      <geometry>
        <box size="0.1 0.05 0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.05 0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <!-- Ixx = 1/12 * 1 * (0.4^2 + 0.05^2) = 0.0135 -->
      <inertia ixx="0.0135" ixy="0.0" ixz="0.0" iyy="0.0135" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- HIP JOINT (Connecting Base to Thigh) -->
  <joint name="hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_leg"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/> <!-- Rotates around Y axis -->
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- LOWER LEG LINK (Shin) -->
  <link name="lower_leg">
    <visual>
      <geometry>
        <box size="0.08 0.04 0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <material name="blue"/>
    </visual>
  </link>

  <!-- KNEE JOINT -->
  <joint name="knee_joint" type="revolute">
    <parent link="upper_leg"/>
    <child link="lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/> <!-- Located at bottom of thigh -->
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="2.0" effort="10.0" velocity="1.0"/>
  </joint>

</robot>
```

---

## 4. Visualizing with `urdf_tutorial`

To see your robot, you don't need a physical machine. ROS 2 provides tools to render the XML.

1.  Install the tools:
    ```bash
    sudo apt install ros-humble-urdf-tutorial
    ```
2.  Launch the display:
    ```bash
    ros2 launch urdf_tutorial display.launch.py model:=simple_leg.urdf
    ```

**What you will see:**
A window (RViz) will open showing your leg. A GUI slider window (Joint State Publisher) allows you to move the `hip_joint` and `knee_joint` sliders and watch the leg kick in real-time.

---

## 5. Xacro: Macros for Humanoids

Writing XML for a robot with 2 arms and 2 legs (like the Unitree G1) involves copy-pasting code 4 times. This is bad practice.

**Xacro (XML Macros)** allows you to write a function (macro) for a leg, and call it twice.

```xml
<xacro:macro name="leg" params="prefix side">
  <link name="${prefix}_upper_leg"> ... </link>
  <joint name="${prefix}_hip" ...> ... </joint>
</xacro:macro>

<!-- Instantiation -->
<xacro:leg prefix="left" side="1" />
<xacro:leg prefix="right" side="-1" />
```

This generates the full URDF automatically at runtime.