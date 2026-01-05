---
id: intro-rl
title: '3.1 Lab: The RL Stack'
sidebar_label: '3.1 Lab: Setup'
description: 'Setting up PyTorch, Gym, and Stable Baselines 3.'
---

# 3.1 Lab: The RL Stack

**"Before we learn, we must install."**

In this lab, we prepare the deep learning environment required to train intelligent agents. We will use **Stable Baselines 3 (SB3)**, the industry standard for reliable RL implementations.

## 🎯 Lab Objectives
1.  **Install the "God Stack"**: PyTorch + Gym + SB3.
2.  **Verify GPU Acceleration** (CUDA).
3.  **Run a Baseline**: Random Agent.

---

## 3.1.1 Installation

Create a fresh Conda environment to avoid dependency hell.

```bash
conda create -n ai_brain python=3.10
conda activate ai_brain

# 1. PyTorch (Check pytorch.org for your CUDA version)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# 2. RL Libraries
pip install gymnasium[all] stable-baselines3 shimmy
pip install tensorboard
```

---

## 3.1.2 The "Hello World" of RL

Let's run the classic **CartPole** environment.
*   **Goal**: Balance a pole on a cart.
*   **Input**: `[Cart Pos, Cart Vel, Pole Angle, Pole Vel]`
*   **Action**: `Left` or `Right`.

### Code: The Random Agent

```python
import gymnasium as gym

# 1. Create Environment
env = gym.make("CartPole-v1", render_mode="human")

# 2. Reset
obs, info = env.reset()

# 3. Loop
for _ in range(100):
    env.render()
    
    # Random Action
    action = env.action_space.sample()
    
    # Step Physics
    obs, reward, terminated, truncated, info = env.step(action)
    
    if terminated or truncated:
        obs, info = env.reset()

env.close()
```

**Run it**: You should see a window pop up with a cart failing miserably to balance the pole.

---

## 3.1.3 Understanding the Observation

Add this print statement inside the loop:
```python
print(f"Obs: {obs} | Reward: {reward}")
```
*   **Obs**: A numpy array of 4 floats.
*   **Reward**: `1.0` for every frame the pole stays up.

In the next lab, we will replace the `random` action with a `learned` one.