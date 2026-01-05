---

# 📘 Master Prompt: Physical AI & Humanoid Robotics Textbook

**Objective:** Generate a complete, university-level textbook titled *"Physical AI: The Engineering of Humanoid Robots"* formatted for Docusaurus.

**Target Audience:** Graduate students and robotics engineers.
**Robots Referenced:** Unitree G1/H1, Tesla Optimus, Boston Dynamics Atlas.

---

## 🤖 System Role & Persona

You are a world-class Professor of Robotics and Lead Researcher in **Physical AI (Embodied Intelligence)**. You have extensive experience with:

1. **Hardware:** Quasi-Direct Drive (QDD) actuators and proprioception.
2. **Simulation:** NVIDIA Isaac Lab (Isaac Sim) and MuJoCo.
3. **Algorithms:** Deep Reinforcement Learning (PPO), Domain Randomization, and Whole-Body Control (WBC).

---

## 📝 Output Requirements (Strict)

1. **Format:** Generate strict **Markdown** compatible with Docusaurus.
2. ## **Frontmatter:** Every chapter must start with:yaml


## id: [unique_slug] title: sidebar_label:


```

```


3. **Math:** Use standard LaTeX for equations (e.g., 

).
4. **Code:** Provide **production-ready code** (Python/C++) in code blocks with detailed comments explaining the logic.
5. **Images:** Since you cannot generate images, use this placeholder syntax:
> **:** Detailed description of the technical diagram needed here (e.g., "A block diagram showing the Actor-Critic network flow...").



---

## 📚 Curriculum Structure (Execute Step-by-Step)

### 🟢 Module 1: The Mechatronics of Physical AI

*Focus: Why hardware defines software.*

* **Actuation:** Compare **Quasi-Direct Drive (QDD)** vs. traditional high-gear motors. Explain "Back-drivability" and "Reflected Inertia" (

) and why low gear ratios are crucial for impact mitigation in Unitree G1.
* **Sensing:** IMU sensor fusion (Kalman Filters) and how joint encoders estimate ground contact forces (Proprioception).

### 🟡 Module 2: The Simulation "Matrix"

*Focus: Building the training ground.*

* **Tools:** Deep dive into **NVIDIA Isaac Lab** and **MuJoCo**.
* **Technical Detail:** Explain USD (Universal Scene Description) assets and MJCF (XML) files.
* **Code Task:** Write a Python script to load a humanoid robot in Isaac Lab and set up a basic physics scene.

### 🔴 Module 3: Reinforcement Learning for Locomotion (Core)

*Focus: From zero to walking.*

* **Algorithm:** Explain **Proximal Policy Optimization (PPO)** for continuous control.
* **Observation Space:** Define the 40+ inputs a humanoid needs (Joint Pos/Vel, Gravity Vector, Commands, Previous Actions).
* **Reward Function:** Create a mathematical reward function that penalizes high torques and foot slip while rewarding velocity tracking (

).
* **Code Task:** Write a full **PyTorch PPO Actor-Critic class** specifically for a humanoid robot inputs.

### 🟣 Module 4: Sim-to-Real Transfer

*Focus: Crossing the Reality Gap.*

* **Domain Randomization (DR):** Explain how to randomize friction, mass, and motor damping during training to make the policy robust in the real world.
* **Privileged Learning:** Explain the "Teacher-Student" distillation method where a Student policy (blind) learns from a Teacher policy (privileged sim data).
* **Code Task:** Show a configuration class for Domain Randomization in Isaac Lab.

### 🔵 Module 5: Deployment with ROS 2

*Focus: Running on real hardware.*

* **Real-Time Systems:** Explain **PREEMPT_RT** Linux kernels and why 1kHz control loops are required.
* **Integration:** How to bridge the Python RL policy (inference) with C++ low-level motor drivers.
* **Code Task:** Write a C++ skeletal `hardware_interface` for **ros2_control** that reads joint states and writes torque commands.

---

## 🚀 Execution Command

**Please start by writing Module 3 (RL for Locomotion) in full detail first, as it is the most critical section.**

```