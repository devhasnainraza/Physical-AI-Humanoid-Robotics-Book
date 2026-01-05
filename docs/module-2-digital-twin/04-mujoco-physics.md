---
id: mujoco-physics
title: '04. MuJoCo Physics'
sidebar_label: '04. MuJoCo Physics'
---

# 2.4 MuJoCo Physics for RL

## 2.4.1 Why MuJoCo?

**MuJoCo (Multi-Joint dynamics with Contact)** is different from Gazebo.
*   **Speed**: It can run thousands of steps per second.
*   **Stability**: It uses a specialized solver that allows for soft contacts and stable simulations even with large time steps.
*   **Differentiability**: (In newer versions) You can calculate gradients through the physics engine, which is huge for optimization.

## 2.4.2 The MJCF Format

MuJoCo uses its own XML format called **MJCF**. It is more concise than URDF.

```xml
<mujoco>
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="1 1 0.1" rgba=".9 .9 .9 1"/>
    <body pos="0 0 1">
      <joint type="free"/>
      <geom type="sphere" size=".1" rgba="1 0 0 1"/>
    </body>
  </worldbody>
</mujoco>
```

## 2.4.3 Python Bindings

We interact with MuJoCo via Python.

```python
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("model.xml")
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # Step the physics
        mujoco.mj_step(model, data)
        
        # Access data
        print(f"Position: {data.qpos}")
        
        # Update viewer
        viewer.sync()
```

## 2.4.4 Key Concepts for RL
1.  **qpos (Position)**: Joint angles and positions.
2.  **qvel (Velocity)**: Joint velocities.
3.  **ctrl (Control)**: The input vector (motor actions).
4.  **sensordata**: Simulated IMU/Force sensors.

In an RL loop, you read `qpos/qvel` (Observation) and write to `ctrl` (Action).
