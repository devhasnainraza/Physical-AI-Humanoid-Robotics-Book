---
id: social-navigation
title: '7.2 Lab: Social Navigation'
sidebar_label: '7.2 Lab: Social Nav'
description: 'Configuring Nav2 Costmaps to respect personal space.'
---

# 7.2 Lab: Social Navigation

**"Excuse me."**

A robot should not drive through people. We will add a **Social Layer** to the Nav2 Costmap.

## 🎯 Lab Objectives
1.  **Install `nav2_social_costmap_plugin`**.
2.  **Configure `nav2_params.yaml`**.
3.  **Test** by putting a "Human" obstacle in Gazebo.

---

## 7.2.1 The Plugin

The plugin takes `/human/bounding_box` (from Lab 7.1) or `/tracked_persons` and injects a Gaussian cost.

## 7.2.2 Configuration

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "inflation_layer", "social_layer"]
      social_layer:
        plugin: "nav2_social_costmap_plugin::SocialLayer"
        enabled: true
        topic: "/human/poses"
        amplitude: 255.0 # Max cost
        cutoff: 10.0
        sigma: 0.5 # Width of bubble (0.5m)
```

## 7.2.3 Behavior

When the Global Planner sees the Social Cost, it will plan a path **around** the human bubble, even if the straight line is shorter.

---

## 7.2.4 Asymmetric Cost

If the human is moving, we stretch the Gaussian in front of them.
*   **Result**: The robot avoids crossing the human's path.

## 7.2.5 Quiz

1.  **What happens if the `sigma` is too large?**
    *   a) The robot will be too shy and might get stuck (unable to find a path).
    *   b) The robot will hit people.
    *   *Answer: a*

2.  **Does the Social Layer affect the Global or Local planner?**
    *   a) Both (usually). Global avoids planning through people; Local brakes if someone jumps in.
    *   b) Only Local.
    *   *Answer: a*
