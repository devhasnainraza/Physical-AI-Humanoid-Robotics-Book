---
sidebar_position: 2
title: Building a Realistic World (SDF)
---

# Creating a Realistic Test Environment with SDF

## 1. The SDF Format

While URDF describes the *robot*, **SDF (Simulation Description Format)** describes the *world* and the robot's interaction with it. SDF is more powerful than URDF as it allows for closed-loop chains (loops in robot limbs) and detailed environment settings.

---

## 2. Writing `test_world.sdf`

We will create a world with:
1.  **Sunlight**: Directional light source.
2.  **Ground Plane**: With friction.
3.  **Physics**: Configuring the step size (1ms).

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="humanoid_gym">
    
    <!-- Physics: 1000 Hz (1ms step) -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Global Light Source -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static> <!-- Gravity doesn't affect it -->
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <!-- Friction Friction Friction -->
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu> <!-- Static Friction -->
                <mu2>1.0</mu2> <!-- Dynamic Friction -->
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <!-- PBR Material -->
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
            <pbr>
                <metal>
                  <albedo_map>materials/textures/concrete_albedo.png</albedo_map>
                  <normal_map>materials/textures/concrete_normal.png</normal_map>
                  <roughness_map>materials/textures/concrete_roughness.png</roughness_map>
                </metal>
            </pbr>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

### 2.1 PBR Materials (Physically Based Rendering)

For VSLAM to work, walls cannot be plain white. They need **Texture**.
Gazebo supports PBR, which uses multiple maps (Albedo, Normal, Roughness) to simulate how light bounces off surfaces. This creates realistic reflections and shadows that are critical for training vision models.

---

## 3. Adding Obstacles

To test navigation, we need obstacles. You can add simple primitives directly in the SDF:

```xml
<model name="box_obstacle">
  <pose>2 0 0.5 0 0 0</pose> <!-- 2m in front of robot -->
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <material>
        <ambient>1 0 0 1</ambient> <!-- Red -->
      </material>
    </visual>
  </link>
</model>
```