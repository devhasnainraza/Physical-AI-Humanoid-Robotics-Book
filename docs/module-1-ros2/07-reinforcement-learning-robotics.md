---
id: reinforcement-learning-robotics
title: '1.7 RL in ROS 2'
sidebar_label: '1.7 RL Bridge'
description: 'Connecting OpenAI Gym to ROS 2 Nodes.'
---

# 1.7 Reinforcement Learning in ROS 2

**"Train in Python, Deploy in ROS."**

RL libraries (PyTorch, Stable Baselines 3) live in Python/Gym. Robots live in ROS. We need a bridge.

## 🎯 Lab Objectives
1.  **Create a Gym Environment** that talks to ROS.
2.  **Train a PPO Agent** to move a turtle.

---

## 1.7.1 The Gym-ROS Bridge

We create a class `RosGymEnv(gym.Env)`.

1.  **`reset()`**: Calls `/reset_simulation` service.
2.  **`step(action)`**:
    *   Publishes `action` to `/cmd_vel`.
    *   Waits for sensor data (Lidar/Camera).
    *   Calculates Reward.
    *   Returns `obs, reward, done`.

### Code Skeleton

```python
import gym
import rclpy
from geometry_msgs.msg import Twist

class TurtleBotEnv(gym.Env):
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('gym_node')
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
    def step(self, action):
        # 1. Action
        msg = Twist()
        msg.linear.x = action[0]
        self.pub.publish(msg)
        
        # 2. Wait (Step Physics)
        # ... logic to wait 0.1s ...
        
        # 3. Reward
        reward = 1.0 # Placeholder
        
        return obs, reward, False, {}
```

## 1.7.2 Training Loop

```python
from stable_baselines3 import PPO

env = TurtleBotEnv()
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=10000)
```

---

## 1.7.3 Quiz

1.  **Why do we need a custom Gym Environment?**
    *   a) Because PyTorch doesn't speak ROS.
    *   b) To visualize the robot.
    *   *Answer: a*

2.  **What happens in the `step()` function?**
    *   a) We publish the action and collect the new observation.
    *   b) We install ROS.
    *   *Answer: a*