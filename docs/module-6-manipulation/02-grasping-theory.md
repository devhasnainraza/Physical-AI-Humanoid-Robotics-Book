---
id: grasping-theory
title: '6.2 Lab: Grasp Detection'
sidebar_label: '6.2 Lab: Grasping'
description: 'Finding grasp poses from Point Clouds.'
---

# 6.2 Lab: Grasp Detection

**"Where do I put my fingers?"**

We have a depth camera. We see a mug. How do we find a valid `[x, y, z, roll, pitch, yaw]` for the gripper?

## 🎯 Lab Objectives
1.  **Process Point Cloud** (Downsample).
2.  **Estimate Normals**.
3.  **Find Anti-Podal Points** (Parallel surfaces).

---

## 6.2.1 The Algorithm (GPD - Grasp Pose Detection)

1.  **Input**: Point Cloud $P$.
2.  **Voxelize**: Shrink $P$ to 1000 points.
3.  **Sample**: Pick a random point $p_1$.
4.  **Search**: Find points $p_2$ within gripper width ($8cm$).
5.  **Check Normals**: Are the normals at $p_1$ and $p_2$ pointing at each other? ($n_1 \cdot n_2 \approx -1$).
    *   Yes: Good grasp candidate!
    *   No: Slippery.

## 6.2.2 Code (Open3D)

```python
import open3d as o3d
import numpy as np

# 1. Load Cloud
pcd = o3d.io.read_point_cloud("mug.ply")
pcd.estimate_normals()

# 2. Heuristic Search
points = np.asarray(pcd.points)
normals = np.asarray(pcd.normals)

for i in range(len(points)):
    p1 = points[i]
    n1 = normals[i]
    
    # ... search for p2 ...
    # if dot(n1, n2) < -0.9: Found Grasp!
```

---

## 6.2.3 Deep Learning Approach (GraspNet)

Instead of geometry, use a CNN.
*   **Input**: Depth Image.
*   **Output**: 6D Grasp Pose + Quality Score.
*   **Lab**: Run `inference_graspnet.py`.

## 6.2.4 Quiz

1.  **What is an "Anti-Podal" grasp?**
    *   a) Grasping opposite sides of an object (forces cancel out).
    *   b) Grasping with feet.
    *   *Answer: a*

2.  **Why do we check normals?**
    *   a) To ensure the gripper fingers align with the surface (flat contact).
    *   b) To detect color.
    *   *Answer: a*
