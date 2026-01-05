---
id: urdf-creation
title: '2.2 Advanced URDF (Xacro)'
sidebar_label: '2.2 Xacro & Inertia'
description: 'Using Macros and calculating Inertia tensors.'
---

# 2.2 Advanced URDF (Xacro)

**"Don't copy-paste XML. Use Xacro."**

URDF is verbose. Xacro is URDF with Macros.

## 🎯 Lab Objectives
1.  **Write a Xacro macro** for a wheel.
2.  **Calculate Inertia** correctly (Crucial for Sim!).

---

## 2.2.1 Xacro Macros

Instead of writing the `<link>` and `<joint>` for 4 wheels, write a macro and call it 4 times.

```xml
<xacro:macro name="wheel" params="prefix x_reflect y_reflect">
    <link name="${prefix}_wheel">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.05"/>
            </geometry>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.1" length="0.05"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="0.5"/>
            <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
        </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="${prefix}_wheel"/>
        <origin xyz="${x_reflect*0.2} ${y_reflect*0.3} 0"/>
        <axis xyz="0 1 0"/>
    </joint>
</xacro:macro>

<!-- Instantiate -->
<xacro:wheel prefix="front_left" x_reflect="1" y_reflect="1"/>
<xacro:wheel prefix="front_right" x_reflect="1" y_reflect="-1"/>
```

---

## 2.2.2 The Inertia Matrix (I)

Simulation needs Mass ($m$) and Inertia ($I$).
If $I$ is wrong, the physics engine explodes.

**Box Formula**:
$$ I_{xx} = \frac{1}{12}m(h^2 + d^2) $$

**Sphere Formula**:
$$ I_{xx} = \frac{2}{5}mr^2 $$

Use MeshLab or SolidWorks to get accurate values. Do not guess!
