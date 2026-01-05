---
id: learning-to-walk
title: '5.4 Lab: Learning to Walk (RL)'
sidebar_label: '5.4 Lab: RL Walk'
description: 'Training a neural network gait policy using PPO.'
---

# 5.4 Lab: Learning to Walk (RL)

**"No math, just rewards."**

Instead of deriving Jacobians and MPC matrices, we will let a Neural Network figure it out.

## 🎯 Lab Objectives
1.  **Define Observation**: `[Base Lin Vel, Base Ang Vel, Gravity Vec, Joint Pos, Joint Vel]`.
2.  **Define Action**: Target Joint Angles ($P_{des}$ for PD controller).
3.  **Define Reward**: $R = v_x - 0.01 · ||tau||^2 - 5.0 · 	ext{falling}$.

---

## 5.4.1 The Policy Network

We map 48 inputs $→$ 12 outputs.
`Obs (48) -> MLP (256, 128) -> Action (12)`

## 5.4.2 The PD Actuator Wrapper

The Network outputs `action` in $[-1, 1]$.
We scale this to joint angles: $q_{des} = q_{nominal} + 	ext{scale} · 	ext{action}$.
Then we apply torque:
$$ tau = K_p (q_{des} - q) - K_d q $$ 

**Why?** Because learning torques directly is too jittery. Learning "Offsets to a standing pose" is much easier.

## 5.4.3 Lab Code (Concept)

```python
import gymnasium as gym
from stable_baselines3 import PPO

# Load a pre-made Quadruped env (like A1Gym)
env = gym.make("Ant-v4") # Built-in Gym quadruped

model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./ppo_ant/")

# Train for 5 million steps (takes ~2 hours on GPU)
model.learn(total_timesteps=5000000)
```

## 5.4.4 Quiz

1.  **Why do we penalize torque in the reward?**
    *   a) To save battery and prevent "spastic" movements.
    *   b) To make the robot stronger.
    *   *Answer: a*

2.  **What is the "Reference Pose"?**
    *   a) The robot's default standing configuration. The NN learns small deviations from this.
    *   b) A book.
    *   *Answer: a*
