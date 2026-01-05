---
id: whole-body-control
title: '5.3 Lab: Whole-Body Control (Inverse Dynamics)'
sidebar_label: '5.3 Lab: WBC'
description: 'Implementing a QP-based inverse dynamics controller for task prioritization.'
---

# 5.3 Lab: Whole-Body Control (Inverse Dynamics)

**"Making all the parts work together in harmony."**

WBC is the lowest level of control that deals with the full complexity of the robot. It takes high-level commands (e.g., "move hand here") and calculates the precise motor torques for *every* joint.

## 🎯 Lab Objectives
1.  **Understand Inverse Dynamics** for floating base robots.
2.  **Formulate a Hierarchical QP** for task prioritization.
3.  **Implement a simple WBC** in Python.

---

## 5.3.1 Floating Base Dynamics

The equation of motion for a robot with a floating base (like our quadruped):
$$ M(q) \ddot{q} + C(q, \dot{q})\dot{q} + G(q) = S^T \tau + J_c^T f_c $$
*   $M$: Mass matrix.
*   $\tau$: Joint torques (what we command).
*   $f_c$: Contact forces (from the ground).
*   $J_c$: Contact Jacobian.

## 5.3.2 Task Prioritization (Hierarchical QP)

We define multiple tasks (e.g., maintain balance, swing leg, keep torso flat) and solve them in order of importance.

### Example Tasks:
1.  **Primary (Balance)**: Keep the Center of Mass (CoM) stable.
    *   Cost: $\min \| \ddot{p}_{CoM} - \ddot{p}_{CoM,des} \|^2$
2.  **Secondary (Swing Leg)**: Move the foot to a desired trajectory.
    *   Cost: $\min \| \ddot{p}_{foot} - \ddot{p}_{foot,des} \|^2$
3.  **Tertiary (Posture)**: Keep joints within comfortable limits.

---

## 5.3.3 The Optimization Problem (QP)

We solve a series of QPs. For Task 1:

$$ \min_{\ddot{q}, f_c, \tau} \| J_{CoM} \ddot{q} - \ddot{p}_{CoM,des} \|^2 $$

Subject to:
1.  **Dynamics**: $M\ddot{q} = \tau_{gravity} + J_c^T f_c - S^T \tau_{joint}$ (Re-arranged)
2.  **Friction Cone**: Limits on $f_c$ (Feet don't slip).
3.  **Torque Limits**: $\tau_{min} \leq \tau \leq \tau_{max}$.

Once $\ddot{q}^*$ is found for Task 1, we use its null-space to solve Task 2.

## 5.3.4 Lab: Python Implementation Sketch

```python
import numpy as np
import cvxpy as cp # CVXPY is a powerful convex optimization library

# Robot model (simplified)
num_joints = 12
num_contacts = 4
M = np.eye(num_joints + 6) # Mass matrix (joints + floating base)
C = np.zeros(num_joints + 6) # Coriolis
G = np.zeros(num_joints + 6) # Gravity
J_com = np.random.rand(3, num_joints + 6) # CoM Jacobian

# Variables to optimize
ddq = cp.Variable(num_joints + 6) # Joint accelerations + base acceleration
fc = cp.Variable(num_contacts * 3) # Contact forces (fx, fy, fz per contact)
tau_joints = cp.Variable(num_joints) # Joint torques

# Task 1: CoM Tracking
ddp_com_des = np.array([0,0,0]) # Keep CoM stationary
objective1 = cp.Minimize(cp.sum_squares(J_com @ ddq - ddp_com_des))

# Dynamics Constraint: M ddq + C + G = S_tau @ tau_joints + J_c.T @ fc
# Simplified: M @ ddq == J_c.T @ fc + S_tau @ tau_joints - G - C
S_tau = np.eye(num_joints + 6)[:, 6:] # Selects joint torques
J_c_full = np.random.rand(num_contacts * 3, num_joints + 6) # Full contact Jacobian

constraints = [
    M @ ddq == J_c_full.T @ fc + S_tau @ tau_joints - G - C,
    fc >= 0 # Normal forces are positive (simplified)
    # ... more constraints like friction cone, torque limits, joint limits
]

# Problem
prob1 = cp.Problem(objective1, constraints)
prob1.solve()

print("Optimal CoM acceleration: ", ddq.value)
# From ddq.value, calculate torques for Task 1.
# Then, define Task 2 in the null-space of J_com (for ddq).
```

---

## 5.3.5 Quiz

1.  **What is the highest priority task for a legged robot?**
    *   a) Waving its hand.
    *   b) Maintaining balance.
    *   *Answer: b*

2.  **Why do we use QP (Quadratic Programming) for WBC?**
    *   a) To solve linear equations.
    *   b) To minimize a quadratic cost function subject to linear constraints, which is fast and robust.
    *   *Answer: b*
