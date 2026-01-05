---
id: bimanual-manipulation
title: '6.4 Lab: Dual Arm Coordination'
sidebar_label: '6.4 Lab: Dual Arm'
description: 'Synchronizing two arms for a handover task.'
---

# 6.4 Lab: Dual Arm Coordination

**"Two hands are better than one."**

## 🎯 Lab Objectives
1.  **Configure MoveIt** for two arms (`left_arm`, `right_arm`).
2.  **Plan a Handover**: Left holds object $\to$ Right grabs object $\to$ Left releases.

---

## 6.4.1 The SRDF (Semantic Robot Description)

We define a **Group** called `both_arms`.

```xml
<group name="both_arms">
    <group name="left_arm"/>
    <group name="right_arm"/>
</group>
```

## 6.4.2 Handover Logic

This is a **State Machine**.

1.  **Approach**: Move both arms to rendezvous point.
2.  **Contact**: Right arm closes gripper.
3.  **Load Transfer**:
    *   Right arm increases force.
    *   Left arm decreases force.
    *   *Critical*: If Left lets go too early, object drops.
4.  **Retreat**: Left arm moves away.

## 6.4.3 Code

```python
# MoveIt script
left.set_pose_target(rendezvous_left)
right.set_pose_target(rendezvous_right)

# Plan together to avoid self-collision
both.plan()
both.execute()
```

---

## 6.4.4 Closed Chain Kinematics

When both hold the object, they form a **Closed Chain**.
DOF drops from $7+7=14$ to $14 - 6 = 8$.
MoveIt doesn't handle closed chains well. We usually switch to **Impedance Control** (Master/Slave) for this phase.

## 6.4.5 Quiz

1.  **What is the biggest risk in dual-arm planning?**
    *   a) Self-collision (Arms hitting each other).
    *   b) Boredom.
    *   *Answer: a*

2.  **Why do we switch to Impedance Control during the hold?**
    *   a) To avoid fighting each other (internal forces) due to small positioning errors.
    *   b) To save power.
    *   *Answer: a*
