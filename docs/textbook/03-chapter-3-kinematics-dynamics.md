---
id: chapter-3-kinematics-dynamics
title: 'Chapter 3: Kinematics & Dynamics'
sidebar_label: '3. Kinematics & Dynamics'
description: 'Forward/Inverse Kinematics (DH Parameters), Jacobians, and Lagrangian Dynamics.'
---

# Chapter 3: Kinematics & Dynamics

**"Geometry tells you where you are. Physics tells you where you are going."**

Kinematics treats the robot as a purely geometric object—a skeleton of lines and points. Dynamics gives that skeleton mass and forces. This chapter covers the rigorous modeling required to control multi-jointed robots.

## 3.1 Forward Kinematics (FK)

The Forward Kinematics problem: Given joint angles $q = [\theta_1, ..., \theta_n]$, compute the pose of the end-effector $T_{ee}$.

### 3.1.1 Denavit-Hartenberg (DH) Parameters
A standardized method to attach coordinate frames to links.
Instead of 6 parameters per link, we use 4:
1.  **Link Length ($a_i$)**: Distance along $x_i$.
2.  **Link Twist ($\alpha_i$)**: Angle around $x_i$.
3.  **Link Offset ($d_i$)**: Distance along $z_{i-1}$.
4.  **Joint Angle ($\theta_i$)**: Rotation around $z_{i-1}$.

The transformation from frame $i-1$ to $i$ is:
$$ T_{i-1}^i = \text{Rot}_z(\theta_i) \cdot \text{Trans}_z(d_i) \cdot \text{Trans}_x(a_i) \cdot \text{Rot}_x(\alpha_i) $$

$$ T_{i-1}^i = \begin{bmatrix} \cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \\ \sin\theta_i & \cos\theta_i \cos\alpha_i & -\cos\theta_i \sin\alpha_i & a_i \sin\theta_i \\ 0 & \sin\alpha_i & \cos\alpha_i & d_i \\ 0 & 0 & 0 & 1 \end{bmatrix} $$

### 3.1.2 Product of Exponentials (PoE)
Modern alternative to DH. Uses Screw Theory.
$$ T_{ee}(q) = e^{\hat{S}_1 \theta_1} e^{\hat{S}_2 \theta_2} ... e^{\hat{S}_n \theta_n} M $$
*   $S_i$: Screw axis in base frame.
*   $M$: Home configuration ($q=0$).
*   **Pros**: No singularity issues with parameterization, easier to calibrate.

## 3.2 Inverse Kinematics (IK)

Given $T_{des}$, find $q$.
This is an ill-posed problem:
1.  **Redundancy**: Infinite solutions (7-DOF arm).
2.  **Singularity**: No solution in certain directions.
3.  **Workspace**: Target outside reach.

### 3.2.1 Numerical IK (Newton-Raphson)
We treat IK as a root-finding problem.
Define error $e(q) = x_{des} - FK(q)$.
We want $e(q) = 0$.
Taylor expansion: $e(q + \Delta q) \approx e(q) + J(q)\Delta q = 0$.
$$ \Delta q = J^{-1}(q) e(q) $$
Iterative update:
$$ q_{k+1} = q_k + \alpha J^\dagger (x_{des} - FK(q_k)) $$
*   $J^\dagger$: Moore-Penrose Pseudoinverse (handles non-square J).

**Python Example**:
```python
import numpy as np

def inverse_kinematics_step(target_pos, current_joints, robot_model):
    # 1. Forward Kinematics
    current_pos = robot_model.forward(current_joints)
    
    # 2. Error
    error = target_pos - current_pos
    
    # 3. Jacobian
    J = robot_model.jacobian(current_joints)
    
    # 4. Pseudoinverse (Damped Least Squares)
    lambda_dls = 0.01
    J_pinv = J.T @ np.linalg.inv(J @ J.T + lambda_dls**2 * np.eye(6))
    
    # 5. Update
    delta_q = J_pinv @ error
    return current_joints + delta_q
```

## 3.3 The Jacobian ($J$)

The Jacobian relates velocities, but also forces (Statics).

### 3.3.1 Velocity Mapping
$$ v_{ee} = J(q) \dot{q} $$

### 3.3.2 Force-Torque Duality
$$ \tau = J^T(q) F_{ext} $$
This is derived from the **Principle of Virtual Work**.
Power in Joint Space = Power in Task Space.
$$ \tau^T \dot{q} = F^T v $$
$$ \tau^T \dot{q} = F^T (J \dot{q}) $$
$$ \tau^T = F^T J \implies \tau = J^T F $$

**Significance**:
*   To apply 10N force at the hand, we compute required joint torques via $J^T$.
*   To detect collisions, we monitor motor currents, estimate $\tau$, and compute $F_{ext}$.

## 3.4 Dynamics: The Equations of Motion

To accelerate a robot, we must overcome Inertia, Friction, Gravity, and Coriolis forces.

### 3.4.1 Lagrangian Mechanics
$$ L(q, \dot{q}) = K(q, \dot{q}) - P(q) $$
*   $K$: Kinetic Energy ($\frac{1}{2} \dot{q}^T M(q) \dot{q}$). 
*   $P$: Potential Energy (Gravity).

Euler-Lagrange Equation:
$$ \frac{d}{dt} \frac{\partial L}{\partial \dot{q}} - \frac{\partial L}{\partial q} = \tau $$

### 3.4.2 The Standard Form
$$ M(q) \ddot{q} + C(q, \dot{q}) \dot{q} + G(q) = \tau $$

1.  **Mass Matrix ($M$)**: Symmetric, Positive Definite. Represents Inertial coupling. "If I move joint 1, joint 2 feels a kick."
2.  **Coriolis/Centrifugal ($C$)**: "Fictitious" forces in rotating frames.
    *   *Centrifugal*: Swing a hammer (pulls outward).
    *   *Coriolis*: Walking on a carousel (sideways push).
3.  **Gravity ($G$)**: Vector of torques needed to hold position against gravity.

### 3.4.3 Forward vs Inverse Dynamics

*   **Inverse Dynamics (RNEA)**:
    *   Input: $q, \dot{q}, \ddot{q}$.
    *   Output: $\tau$.
    *   Use: **Control**. "I want to accelerate at $5 m/s^2$, how much torque do I need?"
    *   Algorithm: Recursive Newton-Euler Algorithm ($O(N)$).

*   **Forward Dynamics (ABA)**:
    *   Input: $q, \dot{q}, \tau$.
    *   Output: $\ddot{q}$.
    *   Use: **Simulation**. "I applied 5Nm, where does the robot go?"
    *   Algorithm: Articulated Body Algorithm ($O(N)$).

## 3.5 Operational Space Dynamics

Sometimes we want to control the robot in Cartesian space directly ($F = ma$ at the hand).
We project the dynamics into task space:

$$ \Lambda(q) \ddot{x} + \mu(q, \dot{q}) + p(q) = F_{ext} $$

*   $\Lambda = (J M^{-1} J^T)^{-1}$: **Task Space Inertia Matrix**. "How heavy does the hand feel?"
    *   *Note*: The "apparent mass" changes with direction. Pushing along the arm is "heavy" (moving base). Pushing perpendicular is "light".

## 3.6 Summary

*   **FK/IK**: Where am I? (Geometry).
*   **Jacobian**: How fast am I moving? How strong am I? (Velocity/Force).
*   **Dynamics**: How do I accelerate? (Physics).

Understanding these matrices is the difference between a robot that moves like a rigid machine and one that moves like a living creature.
