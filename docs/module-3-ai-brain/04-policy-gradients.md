---
id: policy-gradients
title: '3.4 Lab: PPO for Continuous Control'
sidebar_label: '3.4 Lab: PPO'
description: 'Using PPO to balance an inverted pendulum (Continuous Action).'
---

# 3.4 Lab: PPO for Continuous Control

**"Torque is continuous."**

DQN only works for discrete buttons (Left/Right). Robots have motors that accept continuous values (e.g., -5.0 Nm to +5.0 Nm). For this, we need **PPO (Proximal Policy Optimization)**.

## 🎯 Lab Objectives
1.  **Understand Continuous Action Spaces**.
2.  **Train PPO** on `Pendulum-v1`.
3.  **Tune Hyperparameters** (Learning Rate).

---

## 3.4.1 The Environment: Pendulum-v1

*   **Goal**: Swing up a pendulum and keep it upright.
*   **Action**: Torque $	au \in [-2.0, 2.0]$.
*   **Observation**: [cos$\theta$, sin$\theta$, $\dot{\theta}$].

## 3.4.2 The Code

```python
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

# 1. Vectorized Environment (Run 4 sims in parallel!)
env = make_vec_env("Pendulum-v1", n_envs=4)

# 2. Define PPO
model = PPO(
    "MlpPolicy", 
    env, 
    verbose=1, 
    learning_rate=0.0003,
    n_steps=2048,
    batch_size=64,
    n_epochs=10
)

# 3. Train
model.learn(total_timesteps=100000)
model.save("ppo_pendulum")
```

---

## 3.4.3 Parallel Training (`make_vec_env`)

PPO is "On-Policy". It needs fresh data constantly.
Running 4 environments in parallel speeds up data collection by 4x. This is crucial for robotics.

---

## 3.4.4 Quiz

1.  **Can DQN solve Pendulum-v1?**
    *   a) Yes, easily.
    *   b) No, because DQN requires discrete actions. You would have to discretize the torque.
    *   *Answer: b*

2.  **Why use `make_vec_env`?**
    *   a) To make the code look complex.
    *   b) To collect experience from multiple simulations simultaneously, stabilizing the gradient.
    *   *Answer: b*
