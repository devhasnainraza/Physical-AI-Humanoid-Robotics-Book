---
id: model-based-rl
title: '3.5 Model-Based RL'
sidebar_label: '3.5 Model-Based RL'
description: 'Learning the physics of the world to plan: World Models and Dreamer.'
---

# 3.5 Model-Based RL

Model-Free RL (PPO, DQN) requires millions of samples because it learns by trial-and-error without understanding physics.
**Model-Based RL (MBRL)** learns a **World Model** first, then plans using that model.

## 3.5.1 The World Model

We learn a function $f$ that mimics the environment:
$$ s_{t+1} = f(s_t, a_t) $$

*   **Pros**: Sample efficient. If we know the model, we can plan without moving the real robot (Safety).
*   **Cons**: **Model Bias**. If the model is wrong, the plan will be wrong. "Garbage in, garbage out."

## 3.5.2 MPC with Learned Models (PETS)

Instead of using a predefined physics model (like in Ch 5.2), we use a Neural Network ensemble to predict dynamics.

1.  Sample $K$ random action sequences.
2.  Predict outcomes using the Neural Network model.
3.  Pick the best sequence.
4.  Execute first action.

**Uncertainty Awareness**: By using an *ensemble* of NNs, if they disagree, we know the model is uncertain. We can be cautious.

## 3.5.3 Dreamer (Learning inside Imagination)

State-of-the-art algorithms like **DreamerV3** learn a latent world model.
1.  **Encode** images into a compact latent state $z$.
2.  **Predict** future states in latent space (Recurrent Neural Network).
3.  **Train** the Actor-Critic purely inside this "dream" (latent imagination).

This allows robots to learn tasks in minutes of real time, as most training happens in the "dream" (GPU).

## 3.5.4 Quiz

1.  **What is the main advantage of Model-Based RL?**
    *   a) It uses less compute.
    *   b) It is much more sample efficient (needs less data).
    *   c) It is model-free.
    *   *Answer: b*

2.  **What is "Model Bias"?**
    *   a) When the model prefers one color.
    *   b) Exploiting errors in the learned model, leading to failure in reality.
    *   c) A feature of fashion models.
    *   *Answer: b*

3.  **How does "Dreamer" train?**
    *   a) While the robot is sleeping.
    *   b) By interacting with a learned latent dynamics model.
    *   c) Using real-world data only.
    *   *Answer: b*