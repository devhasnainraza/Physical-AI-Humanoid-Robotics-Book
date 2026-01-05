---
id: mdp
title: '3.2 Lab: Custom Gym Environment'
sidebar_label: '3.2 Lab: Custom Env'
description: 'Building a custom "GridWorld" MDP from scratch.'
---

# 3.2 Lab: Custom Gym Environment

**"If you can simulate it, you can solve it."**

To apply RL to a new robot, you must wrap your robot code in a **Gym Interface**. This standardizes the communication between the Agent and the World.

## 🎯 Lab Objectives
1.  **Inherit from `gym.Env`**.
2.  **Define Spaces**: `Box` vs `Discrete`.
3.  **Implement `step()` and `reset()`**.

---

## 3.2.1 The Scenario: GridWorld

A simple 5x5 grid.
*   **Start**: Top-Left (0,0).
*   **Goal**: Bottom-Right (4,4).
*   **Obstacle**: (2,2).
*   **Action**: Up, Down, Left, Right.

---

## 3.2.2 The Code (`grid_world.py`)

```python
import gymnasium as gym
from gymnasium import spaces
import numpy as np

class GridWorldEnv(gym.Env):
    def __init__(self):
        super().__init__()
        # 5x5 Grid
        self.grid_size = 5
        
        # Action Space: 0=Up, 1=Down, 2=Left, 3=Right
        self.action_space = spaces.Discrete(4)
        
        # Observation Space: [x, y] coordinates
        self.observation_space = spaces.Box(low=0, high=4, shape=(2,), dtype=np.int32)
        
        # State
        self.agent_pos = np.array([0, 0])
        self.goal_pos = np.array([4, 4])

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.agent_pos = np.array([0, 0])
        return self.agent_pos, {}

    def step(self, action):
        # Physics Logic
        if action == 0: self.agent_pos[1] = max(0, self.agent_pos[1] - 1) # Up
        if action == 1: self.agent_pos[1] = min(4, self.agent_pos[1] + 1) # Down
        if action == 2: self.agent_pos[0] = max(0, self.agent_pos[0] - 1) # Left
        if action == 3: self.agent_pos[0] = min(4, self.agent_pos[0] + 1) # Right
        
        # Reward Function
        terminated = np.array_equal(self.agent_pos, self.goal_pos)
        reward = 10.0 if terminated else -0.1 # Time penalty
        
        return self.agent_pos, reward, terminated, False, {}

# Test it
env = GridWorldEnv()
obs, _ = env.reset()
print("Start:", obs)
obs, reward, done, _, _ = env.step(1) # Down
print("Step Down:", obs, "Reward:", reward)
```

## 3.2.3 Quiz

1.  **Why do we give a negative reward (-0.1) for each step?**
    *   a) To punish the robot.
    *   b) To encourage the robot to find the *shortest* path (accumulate less penalty).
    *   *Answer: b*

2.  **What happens if we don't define `observation_space`?**
    *   a) Nothing.
    *   b) SB3 will crash because it needs to know the input shape for the Neural Network.
    *   *Answer: b*
