---
id: q-learning
title: '3.3 Lab: Training with DQN'
sidebar_label: '3.3 Lab: DQN'
description: 'Using Stable Baselines 3 to solve CartPole.'
---

# 3.3 Lab: Training with DQN

**"From Randomness to Mastery."**

We have the environment. Now we attach the Brain. We will use **Deep Q-Network (DQN)**, the algorithm that started the Deep RL revolution (Atari).

## 🎯 Lab Objectives
1.  **Train a DQN Agent** using SB3.
2.  **Monitor Training** with Tensorboard.
3.  **Save and Load** the trained model.

---

## 3.3.1 The Training Script

```python
import gymnasium as gym
from stable_baselines3 import DQN
import os

# 1. Create Environment
env = gym.make("CartPole-v1", render_mode="rgb_array")

# 2. Define Model
# MlpPolicy: Multi-Layer Perceptron (Simple Neural Net)
model = DQN("MlpPolicy", env, verbose=1, tensorboard_log="./dqn_cartpole_logs/")

# 3. Train
print("Training started...")
model.learn(total_timesteps=50000)
print("Training finished.")

# 4. Save
model.save("dqn_cartpole")
```

---

## 3.3.2 Monitoring with Tensorboard

While training runs, open a new terminal:
```bash
tensorboard --logdir ./dqn_cartpole_logs/
```
Go to `http://localhost:6006`.
*   **Watch the `rollout/ep_rew_mean` graph.**
*   It should start at ~20 and rise to 500 (Max score) within 50k steps.

---

## 3.3.3 Evaluation (Enjoy Mode)

Now let's watch the trained agent.

```python
# Load Model
model = DQN.load("dqn_cartpole")

# Create Env with Render
env = gym.make("CartPole-v1", render_mode="human")

obs, _ = env.reset()
for _ in range(1000):
    env.render()
    
    # Predict Action (Deterministic)
    action, _states = model.predict(obs, deterministic=True)
    
    obs, reward, done, truncated, info = env.step(action)
    if done or truncated:
        obs, _ = env.reset()
```

## 3.3.4 Quiz

1.  **Why do we set `deterministic=True` during evaluation?**
    *   a) To stop the agent from exploring (Epsilon-greedy) and make it perform its best.
    *   b) To save memory.
    *   *Answer: a*

2.  **If the reward graph stays flat at 20, what happened?**
    *   a) The agent failed to learn (Hyperparameters might be wrong).
    *   b) The task is impossible.
    *   *Answer: a*
