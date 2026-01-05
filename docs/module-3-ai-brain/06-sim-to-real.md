---
id: sim-to-real
title: '3.6 Lab: Sim-to-Real (Domain Randomization)'
sidebar_label: '3.6 Lab: Sim-to-Real'
description: 'Robustifying policies by randomizing physics.'
---

# 3.6 Lab: Sim-to-Real (Domain Randomization)

**"If you can walk on ice, you can walk on concrete."**

A policy trained in a perfect simulation will fail in reality. To fix this, we confuse the agent during training by constantly changing the physics.

## 🎯 Lab Objectives
1.  **Create a Gym Wrapper**.
2.  **Randomize Mass and Length**.
3.  **Train a Robust Policy**.

---

## 3.6.1 The Randomization Wrapper

We will wrap `Pendulum-v1` and change the pole mass every reset.

```python
import gymnasium as gym
import numpy as np

class DomainRandomizationWrapper(gym.Wrapper):
    def __init__(self, env):
        super().__init__(env)
        
    def reset(self, **kwargs):
        # 1. Randomize Mass (Default is 1.0)
        new_mass = np.random.uniform(0.5, 2.0)
        self.env.unwrapped.m = new_mass
        
        # 2. Randomize Length (Default is 1.0)
        new_length = np.random.uniform(0.8, 1.2)
        self.env.unwrapped.l = new_length
        
        return self.env.reset(**kwargs)

# Usage
env = gym.make("Pendulum-v1")
env = DomainRandomizationWrapper(env)

# Now train PPO on this 'env'.
# The agent will learn a policy that works for ALL masses between 0.5 and 2.0.
```

---

## 3.6.2 Sim-to-Real Gap Checklist

When deploying to a real robot, randomize:
1.  **Latency**: Add `time.sleep(random)` in the step function.
2.  **Noise**: Add `obs += np.random.normal()` to sensor data.
3.  **Friction**: (If using MuJoCo/PyBullet).

## 3.6.3 Quiz

1.  **Why do we randomize mass?**
    *   a) We don't know the exact mass of the real robot (cables, tape).
    *   b) To make the robot heavier.
    *   *Answer: a*

2.  **What happens if we don't randomize?**
    *   a) The policy overfits to the simulation physics and fails in reality.
    *   b) Nothing.
    *   *Answer: a*
