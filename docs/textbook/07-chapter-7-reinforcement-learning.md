---
id: chapter-7-reinforcement-learning
title: 'Chapter 7: Reinforcement Learning Theory'
sidebar_label: '7. RL Theory'
description: 'The math of learning: MDPs, Bellman Equations, and the Quest for Reward.'
---

# Chapter 7: Reinforcement Learning Theory

**"Experience is the name everyone gives to their mistakes." — Oscar Wilde**

Reinforcement Learning (RL) is simply the mathematical formalization of learning from mistakes. Unlike supervised learning (where a teacher tells you the answer), RL is about **trial and error**.

## 🎯 Learning Objectives
1.  **Grok the MDP**: The framework of the world.
2.  **Master the Bellman Equation**: The recursive magic that solves RL.
3.  **Distinguish V vs Q**: Value functions explained simply.

---

## 7.1 The Intuition: The Dog and the Treat

Imagine you are training a dog to sit.
1.  **Observation ($S$)**: The dog hears "Sit".
2.  **Action ($A$)**: The dog jumps.
3.  **Reward ($R$)**: You shout "No!" ($R = -1$).
4.  **Update**: The dog's brain rewires: "When I hear Sit, Jumping is bad."
5.  **Try Again**: Dog sits.
6.  **Reward**: You give a treat ($R = +10$).
7.  **Update**: "When I hear Sit, Sitting is awesome."

This loop is the **Markov Decision Process (MDP)**.

### 7.1.1 The Reward Hypothesis
*   **All goals** can be described as the maximization of expected cumulative reward.
*   If the robot is doing something wrong (e.g., spinning in circles), it's not "dumb". It's maximizing the reward *you gave it*. You probably designed a bad reward function.

---

## 7.2 The Math: Markov Decision Processes (MDP)

An MDP is a tuple $(S, A, P, R, \gamma)$.

### 7.2.1 The Components
*   **$S$ (State Space)**: Everything describing the world. (Robot Position, Velocity, Joint Angles).
*   **$A$ (Action Space)**: What the robot can do. (Motor Torques).
*   **$P$ (Transition Probability)**: The Physics. If I apply 5Nm torque, where do I end up?
    *   $P(s' | s, a)$
*   **$R$ (Reward Function)**: The Score.
    *   $R(s, a)$
*   **$\gamma$ (Discount Factor)**: Impatience.
    *   $\gamma = 0$: Only care about immediate reward (Myopic).
    *   $\gamma = 0.99$: Care about the long-term future (Far-sighted).

### 7.2.2 The Return ($G_t$)
The goal is to maximize the sum of future rewards.
$$ G_t = R_{t+1} + \gamma R_{t+2} + \gamma^2 R_{t+3} + \dots = \sum_{k=0}^{\infty} \gamma^k R_{t+k+1} $$

---

## 7.3 Value Functions: "How good is this?"

### 7.3.1 State-Value Function $V(s)$
"How good is it to be in this state?"
*   *Example*: Standing on a cliff edge.
    *   $V(s)$ is **Low** (High risk of falling).
*   *Example*: Standing next to the goal.
    *   $V(s)$ is **High**.

$$ V_{\pi}(s) = \mathbb{E}_{\pi} [ G_t | S_t = s ] $$

### 7.3.2 Action-Value Function $Q(s, a)$
"How good is it to take **this specific action** in this state?"
*   *State*: Cliff Edge.
*   *Action A*: Jump forward. $\to Q(s, A)$ is **Very Low**.
*   *Action B*: Step back. $\to Q(s, B)$ is **High**.

$$ Q_{\pi}(s, a) = \mathbb{E}_{\pi} [ G_t | S_t = s, A_t = a ] $$

---

## 7.4 The Bellman Equation: The Recursive Magic

Richard Bellman realized something simple but profound:
**"The value of your current state is the Reward you get now + the Value of the next state."**

$$ V(s) = R + \gamma V(s') $$

This creates a recursive definition. We don't need to simulate to infinity; we just need to look **one step ahead**.

### 7.4.1 The Bellman Optimality Equation
This is what we solve to find the perfect policy.
$$ V^*(s) = \max_{a} \left( R(s, a) + \gamma \sum_{s'} P(s'|s,a) V^*(s') \right) $$

"The value of a state is the *best* possible score I can get if I choose the *best* action."

---

## 7.5 Types of RL Agents

### 7.5.1 Value-Based (e.g., DQN)
*   **Philosophy**: "I will learn the map of Value ($Q$). Then I will just act greedily."
*   **Policy**: $\pi(s) = \arg\max_a Q(s, a)$.
*   **Good for**: Discrete actions (Atari games, Chess).

### 7.5.2 Policy-Based (e.g., REINFORCE)
*   **Philosophy**: "I don't care about values. I will just learn which action to take directly."
*   **Policy**: A Neural Network outputs probabilities $\pi(a|s)$.
*   **Good for**: Continuous actions (Robotics).

### 7.5.3 Actor-Critic (e.g., PPO)
*   **Philosophy**: "Why not both?"
*   **Actor**: Learns the Policy (The Doer).
*   **Critic**: Learns the Value (The Judge).
*   The Critic tells the Actor: "That move was better than I expected!" (Advantage).

---

## 🛠️ Practical Implementation (Python)

Calculating a Value Iteration step (Dynamic Programming):

```python
import numpy as np

# Simple GridWorld
states = [0, 1, 2, 3] # 3 is Goal
rewards = [0, 0, 0, 10]
gamma = 0.9
V = np.zeros(4)

def update_values():
    new_V = np.copy(V)
    for s in states:
        if s == 3: continue # Terminal state
        
        # Possible actions: Left (s-1) or Right (s+1)
        # Assuming deterministic physics
        
        # Q-value for Right
        next_s_right = min(s+1, 3)
        q_right = rewards[next_s_right] + gamma * V[next_s_right]
        
        # Q-value for Left
        next_s_left = max(s-1, 0)
        q_left = rewards[next_s_left] + gamma * V[next_s_left]
        
        # Bellman Optimality: Take max
        new_V[s] = max(q_right, q_left)
    return new_V

for i in range(10):
    V = update_values()
    print(f"Iter {i}: {V}")
```

## 7.6 Quiz

1.  **What does $\gamma = 0$ mean?**
    *   a) The robot plans infinitely far ahead.
    *   b) The robot is extremely greedy and only cares about the immediate instant.
    *   c) The robot is broken.
    *   *Answer: b*

2.  **What is the difference between $V(s)$ and $Q(s,a)$?**
    *   a) V evaluates the state; Q evaluates the action in the state.
    *   b) V is for vectors; Q is for quaternions.
    *   c) They are identical.
    *   *Answer: a*

3.  **Why is PPO called "Actor-Critic"?**
    *   a) Because it likes movies.
    *   b) Because it uses one network to act (Actor) and another to evaluate the action (Critic).
    *   *Answer: b*