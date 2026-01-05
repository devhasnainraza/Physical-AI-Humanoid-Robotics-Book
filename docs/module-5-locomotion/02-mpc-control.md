---
id: mpc-control
title: '5.2 Lab: Convex MPC for Locomotion'
sidebar_label: '5.2 Lab: MPC'
description: 'Formulating and solving a Quadratic Program (QP) for legged robot control.'
---

# 5.2 Lab: Convex MPC for Locomotion

**"Control the future, one step at a time."**

MPC (Model Predictive Control) is how agile legged robots stay balanced. In this lab, we will formulate a simple Linear MPC problem for a quadruped's Center of Mass (CoM) and solve it using OSQP (a Quadratic Program solver).

## 🎯 Lab Objectives
1.  **Understand the Linear Inverted Pendulum Model (LIPM)** states.
2.  **Formulate the QP** cost and constraints.
3.  **Solve the QP** using OSQP.

---

## 5.2.1 The LIPM State

We predict the robot's CoM horizontal position and velocity:
$$ x = [p_x, p_y, v_x, v_y]^T $$
The dynamics are:
$$ \dot{x} = A x + B u $$
Where $u = [p_{zmp,x}, p_{zmp,y}]^T$ (the Zero Moment Point).

## 5.2.2 Prediction Over Horizon ($N$)

We want to predict $x$ for the next $N$ time steps.
$$ X = \begin{bmatrix} x_0 \ x_1 \ \vdots \ x_N \end{bmatrix}, \quad U = \begin{bmatrix} u_0 \ u_1 \ \vdots \ u_N \end{bmatrix} $$

The predicted states are:
$$ x_{k+1} = A_d x_k + B_d u_k $$
This leads to:
$$ X = \mathcal{A} x_0 + \mathcal{B} U $$
Where $\mathcal{A}$ and $\mathcal{B}$ are block matrices derived from $A_d, B_d$.

---

## 5.2.3 The Optimization Problem (QP)

Minimize a cost function:
$$ \min_{U} \frac{1}{2} U^T H U + f^T U $$
Subject to linear constraints:
$$ l \leq A_{con} U \leq u $$

### Cost Function
*   **Reference Tracking**: Minimize $(X - X_{ref})^T Q (X - X_{ref})$. (Stay near desired trajectory).
*   **Control Effort**: Minimize $U^T R U$. (Don't use too much force/ZMP change).

### Constraints
1.  **ZMP within Support Polygon**: $u_k \in \text{SupportPolygon}_k$. (Feet can only push where they are).
2.  **Actuator Limits**: $\tau_{min} \leq \tau \leq \tau_{max}$. (Limits on joint torques).

---

## 5.2.4 Lab: Python Implementation with OSQP

```python
import numpy as np
import scipy.sparse as sparse
import osqp

# 1. System Dynamics (LIPM)
# Assume dt = 0.1s, z_com = 1.0m, g = 9.81
omega = np.sqrt(9.81 / 1.0)
Ad = np.array([
    [1, 0, np.sinh(omega*0.1)/omega, 0],
    [0, 1, 0, np.sinh(omega*0.1)/omega],
    [0, 0, np.cosh(omega*0.1), 0],
    [0, 0, 0, np.cosh(omega*0.1)]
])
Bd = np.array([
    [(1 - np.cosh(omega*0.1))/omega**2, 0],
    [0, (1 - np.cosh(omega*0.1))/omega**2],
    [-np.sinh(omega*0.1)/omega, 0],
    [0, -np.sinh(omega*0.1)/omega]
])

# 2. Prediction Matrices (A_bar, B_bar)
# Build A_bar and B_bar for N=10 steps (omitted for brevity)
# ... this expands x_{k+1} = Ad x_k + Bd u_k into X = A_bar x_0 + B_bar U

# 3. Cost Function (Example)
Q_cost = sparse.diags([10, 10, 1, 1]) # Weight for position error
R_cost = sparse.diags([0.1, 0.1])    # Weight for ZMP control effort

# ... Build H and f for QP from Q_cost, R_cost, A_bar, B_bar ...

# 4. Constraints (Example: ZMP within a box)
# Define A_con, l, u (omitted for brevity)
# e.g., for zmp_x in [-0.1, 0.1], zmp_y in [-0.05, 0.05]
# A_con = sparse.vstack([sparse.eye(2*N), -sparse.eye(2*N)])
# l = np.tile([-0.1, -0.05], N)
# u = np.tile([0.1, 0.05], N)


# 5. Solve QP
prob = osqp.OSQP()
prob.setup(P=H_qp, q=f_qp, A=A_con_qp, l=l_qp, u=u_qp, verbose=False)
res = prob.solve()

# Extract first ZMP command
zmp_command = res.x[:2] 
```

---

## 5.2.5 Quiz

1.  **What does `omega` in the LIPM equations represent?**
    *   a) Angular velocity.
    *   b) The natural frequency of the pendulum.
    *   *Answer: b*

2.  **Why do we use sparse matrices in OSQP?**
    *   a) Because our matrices are usually very large with many zeros, saving memory and computation.
    *   b) They look nicer.
    *   *Answer: a*