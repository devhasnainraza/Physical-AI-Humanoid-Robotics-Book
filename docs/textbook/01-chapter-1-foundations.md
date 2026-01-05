---
id: chapter-1-foundations
title: 'Chapter 1: Mathematical Foundations'
sidebar_label: '1. Foundations'
description: 'Linear Algebra, Lie Theory, Optimization, and Probability for Robotics.'
---

# Chapter 1: Mathematical Foundations

**"Mathematics is the language in which God has written the universe." — Galileo Galilei**

In robotics, mathematics is not just theory; it is the engine of implementation. Every motion a robot makes is a matrix multiplication. Every decision it takes is a probabilistic inference. This chapter establishes the rigorous mathematical pillars required for Physical AI: Linear Algebra, Lie Theory, Calculus, Optimization, and Probability.

---

## 1.1 Linear Algebra: The Space of Robots

Robots exist in Euclidean space ($\mathbb{R}^3$), but their configurations (joints, orientations) exist in curved high-dimensional manifolds.

### 1.1.1 Vectors and Norms
A vector $x \in \mathbb{R}^n$ represents a point or a direction.
$$ x = [x_1, x_2, ..., x_n]^T $$

The **magnitude** of error is measured by Norms.
*   **$L_2$ Norm (Euclidean)**: "As the crow flies."
    $$ \|x\|_2 = \sqrt{\sum x_i^2} $$
*   **$L_1$ Norm (Manhattan)**: "City blocks." Robust to outliers.
    $$ \|x\|_1 = \sum |x_i| $$

**Robotics Context**: In Trajectory Optimization, we minimize $\| x_{goal} - x_{curr} \|^2$.

### 1.1.2 The Cross Product (Skew-Symmetric Matrix)
The cross product $a \times b$ produces a vector perpendicular to both.
Crucially, we can represent $a \times b$ as a matrix multiplication $[a]_\times b$.

$$ [a]_\times = \begin{bmatrix} 0 & -a_z & a_y \\ a_z & 0 & -a_x \\ -a_y & a_x & 0 \end{bmatrix} $$

This matrix $[a]_\times$ is **Skew-Symmetric** ($A^T = -A$). This is the foundation of rotation physics ($\omega \times r$).

### 1.1.3 Eigenvalues and Eigenvectors
For a matrix $A$, an eigenvector $v$ is a vector that only scales (doesn't rotate) when $A$ is applied.
$$ Av = \lambda v $$
*   **Context**: If the eigenvalues of a control system matrix have Real Part < 0, the system is **Stable**. If > 0, it explodes.

---

## 1.2 Lie Theory: The Manifold of Rotation

Rotations are not vectors. You cannot just add them ($R_1 + R_2 \neq R_{total}$). They form a **Group**, specifically a Lie Group.

### 1.2.1 The Lie Group $SO(3)$
The space of 3D rotations.
$$ SO(3) = \{ R \in \mathbb{R}^{3 \times 3} \mid R^T R = I, \det(R) = 1 \} $$
This is a smooth, curved surface (manifold).

### 1.2.2 The Lie Algebra $\mathfrak{so}(3)$
The tangent space to the group at the Identity.
This is a flat vector space! We can add/subtract here.
Elements of $\mathfrak{so}(3)$ are skew-symmetric matrices $[\omega]_\times$.

### 1.2.3 The Exponential Map ($e^{\hat{\omega}\theta}$)
How do we go from the flat Tangent Space (Angular Velocity $\omega$) to the curved Manifold (Rotation $R$)?
**Integration**. For rotations, this is the Matrix Exponential.

$$ R = \exp([\omega]_\times \theta) = I + \sin\theta [\omega]_\times + (1-\cos\theta) [\omega]_\times^2 $$
*(Rodrigues' Rotation Formula)*.

**Intuition**: "Lie Algebra is velocity. Lie Group is position. The Exponential Map is 'Move at this velocity for 1 second'."

---

## 1.3 Homogeneous Transformations ($SE(3)$)

We combine Rotation ($R$) and Translation ($p$) into a single $4 \times 4$ matrix.
$$ T = \begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix} \in SE(3) $$

**Chaining**: $T^A_C = T^A_B \cdot T^B_C$.
**Inverting**: $T^{-1} = \begin{bmatrix} R^T & -R^T p \\ 0 & 1 \end{bmatrix}$.

---

## 1.4 Vector Calculus & Optimization

Robots maximize reward (RL) or minimize error (IK). Both require **Optimization**.

### 1.4.1 The Gradient ($\nabla f$)
Direction of steepest ascent.
$$ \nabla f(x) = \left[ \frac{\partial f}{\partial x_1}, \dots \right]^T $$

### 1.4.2 The Jacobian ($J$)
Maps velocities from Joint Space to Task Space.
$$ \dot{x} = J(q) \dot{q} $$
It is also the derivative of a vector-valued function.

### 1.4.3 Optimization Algorithms
We want to find $x^*$ that minimizes $f(x)$.

1.  **Gradient Descent**:
    $$ x_{k+1} = x_k - \alpha \nabla f(x_k) $$
    *   *Pros*: Simple.
    *   *Cons*: Slow (zig-zags).

2.  **Newton's Method**:
    Uses curvature (Hessian $H$) to jump straight to the minimum (if quadratic).
    $$ x_{k+1} = x_k - H^{-1} \nabla f(x_k) $$
    *   *Pros*: Fast.
    *   *Cons*: Calculating $H$ is expensive.

3.  **Gauss-Newton**:
    Approximates $H \approx J^T J$. Used for "Least Squares" problems like Inverse Kinematics.
    $$ \Delta x = -(J^T J)^{-1} J^T e $$

---

## 1.5 Probability: Dealing with Uncertainty

### 1.5.1 The Gaussian ($N(\mu, \Sigma)$)
Defined by Mean $\mu$ (Best Guess) and Covariance $\Sigma$ (Uncertainty Ellipse).
*   **Properties**: Linear transformation of a Gaussian is still a Gaussian. This is why Kalman Filters are fast.

### 1.5.2 Bayes Rule
$$ P(x|z) = \frac{P(z|x) P(x)}{P(z)} $$
*   **Prior $P(x)$**: "I think I am at $x=10$."
*   **Likelihood $P(z|x)$**: "The GPS says $x=12$."
*   **Posterior $P(x|z)$**: "I am likely at $x=11$."

---

## 1.6 Graph Theory (Basics)

Used in SLAM and Planning.
*   **Graph $G = (V, E)$**: Vertices (Nodes) and Edges (Links).
*   **Adjacency Matrix**: A grid representing connections.
*   **Sparsity**: In Mapping, the matrix is sparse (mostly zeros). We exploit this to solve systems with 100,000 variables efficiently.

---

## 🛠️ Practical Implementation (Python)

**Lie Algebra in Python**:

```python
import numpy as np
from scipy.linalg import expm

def skew(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])

# 1. Define Axis-Angle (Lie Algebra)
axis = np.array([0, 0, 1]) # Z-axis
angle = np.pi / 2          # 90 degrees
algebra = skew(axis * angle)

# 2. Exponentiate to Group (Rotation Matrix)
R = expm(algebra)

print("Rotation Matrix (90 deg about Z):\n", R.round(2))
# Output: [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
```

## 1.7 Quiz

1.  **Is $SO(3)$ a Vector Space?**
    *   a) Yes.
    *   b) No, it's a curved manifold. You cannot simply add two rotation matrices.
    *   *Answer: b*

2.  **What does the Jacobian Matrix tell us?**
    *   a) How fast the hand moves for a given joint speed.
    *   b) The color of the robot.
    *   *Answer: a*

3.  **Why do we use Gauss-Newton instead of standard Gradient Descent for IK?**
    *   a) Because it uses curvature information ($J^T J$) to converge much faster.
    *   b) Because it is easier to code.
    *   *Answer: a*
