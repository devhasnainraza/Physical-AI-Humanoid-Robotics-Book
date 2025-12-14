# Hackathon I: Physical AI & Humanoid Robotics Textbook

## 📘 Overview

This repository is part of **Hackathon I**, where participants create an **AI-native textbook** for teaching **Physical AI & Humanoid Robotics**. The project combines modern documentation, AI agents, and robotics simulation to deliver an interactive learning experience.

The future of work is a collaboration between humans, AI agents, and robots. This hackathon focuses on preparing students for that future by bridging the gap between **digital intelligence** and **physical embodiment**.

---

## 🚀 About the Hackathon

**Objective:**
Create a complete, AI-native textbook for the **Physical AI & Humanoid Robotics** course and deploy it online.

Top-performing participants may:

* Get interviewed to join the **Panaversity core team**
* Receive an opportunity to become a **startup founder**
* Teach at **Panaversity, PIAIC, or GIAIC**
* Collaborate with Panaversity founders

---

## 🧰 Core Requirements

### 1. AI / Spec-Driven Book Creation

* Write the textbook using **Docusaurus**
* Deploy the book on **GitHub Pages or Vercel**
* Use the following tools:

  * [Spec-Kit Plus](https://github.com/panaversity/spec-kit-plus/)
  * [Claude Code](https://www.claude.com/product/claude-code)

### 2. Integrated RAG Chatbot

Embed a **Retrieval-Augmented Generation (RAG)** chatbot inside the book.

**Features:**

* Answers questions from the book content
* Can answer questions based only on user-selected text

**Tech Stack:**

* OpenAI Agents / ChatKit SDKs
* FastAPI
* Neon Serverless Postgres
* Qdrant Cloud (Free Tier)

### 3. Bonus Features (Optional)

* +50 points: Reusable intelligence using Claude Subagents & Agent Skills
* +50 points: Signup & Signin using [https://www.better-auth.com/](https://www.better-auth.com/)
* +50 points: Personalized chapter content for logged-in users
* +50 points: Urdu translation button for chapters

---

## ⏰ Timeline

* **Submission Deadline:** Sunday, Nov 30, 2025 — 06:00 PM
* **Live Presentations:** Sunday, Nov 30, 2025 — 06:00 PM (Zoom)

> Live presentations are by invitation only and do not affect final scoring.

---

## 📤 Submission Instructions

Submit your project here:
👉 [https://forms.gle/CQsSEGM3GeCrL43c8](https://forms.gle/CQsSEGM3GeCrL43c8)

You must provide:

1. Public GitHub Repository Link
2. Published Book Link (GitHub Pages or Vercel)
3. Demo Video (max 90 seconds)
4. WhatsApp Number (for top submissions)

---

## 🎥 Zoom Meeting (Presentation Day)

* **Date:** Nov 30, 2025
* **Time:** 06:00 PM
* **Meeting ID:** 849 7684 7088
* **Passcode:** 305850
* **Zoom Link:** [https://us06web.zoom.us/j/84976847088](https://us06web.zoom.us/j/84976847088)

---

# 📚 Course: Physical AI & Humanoid Robotics

## 🎯 Focus & Goal

* **Theme:** AI Systems in the Physical World
* **Goal:** Bridge the digital brain with the physical body
* Apply AI knowledge to control humanoid robots in simulated and real environments

---

## 🧠 Quarter Overview

This course introduces **Physical AI**—AI systems that understand and operate under real-world physical laws. Students design, simulate, and deploy humanoid robots using:

* ROS 2
* Gazebo
* Unity
* NVIDIA Isaac

---

## 🧩 Course Modules

### Module 1: The Robotic Nervous System (ROS 2)

* ROS 2 nodes, topics, and services
* Python agents using `rclpy`
* URDF for humanoid robots

### Module 2: The Digital Twin (Gazebo & Unity)

* Physics simulation (gravity, collisions)
* Sensor simulation (LiDAR, depth cameras, IMUs)
* High-fidelity visualization

### Module 3: The AI-Robot Brain (NVIDIA Isaac)

* Isaac Sim and synthetic data generation
* Isaac ROS (VSLAM & navigation)
* Nav2 for humanoid movement

### Module 4: Vision-Language-Action (VLA)

* Voice-to-action using Whisper
* LLM-based cognitive planning
* **Capstone:** Autonomous humanoid robot

---

## 🌍 Why Physical AI Matters

Humanoid robots are uniquely suited for human environments. Physical AI represents the transition from purely digital intelligence to **embodied intelligence** operating in the real world.

---

## 🎓 Learning Outcomes

By the end of this course, students will be able to:

1. Understand Physical AI and embodied intelligence
2. Master ROS 2 for robotic control
3. Simulate robots using Gazebo and Unity
4. Develop AI pipelines with NVIDIA Isaac
5. Design humanoid robots for natural interaction
6. Integrate GPT models for conversational robotics

---

## 🗓 Weekly Breakdown

* **Weeks 1–2:** Physical AI foundations & sensors
* **Weeks 3–5:** ROS 2 fundamentals
* **Weeks 6–7:** Robot simulation with Gazebo
* **Weeks 8–10:** NVIDIA Isaac platform
* **Weeks 11–12:** Humanoid robot development
* **Week 13:** Conversational robotics

---

## 📝 Assessments

* ROS 2 package development
* Gazebo simulation project
* Isaac-based perception pipeline
* Final capstone project

---

# 🖥 Hardware Requirements

## 1. Digital Twin Workstation (Required)

* **GPU:** NVIDIA RTX 4070 Ti (12GB) or higher
* **CPU:** Intel i7 (13th Gen+) or AMD Ryzen 9
* **RAM:** 64GB DDR5 (32GB minimum)
* **OS:** Ubuntu 22.04 LTS

---

## 2. Physical AI Edge Kit

* NVIDIA Jetson Orin Nano / Orin NX
* Intel RealSense D435i / D455
* USB IMU (BNO055)
* USB microphone array (ReSpeaker)

---

## 3. Robot Lab Options

### Option A: Proxy Robot (Recommended)

* Unitree Go2 Edu

### Option B: Miniature Humanoid

* Unitree G1
* Robotis OP3
* Budget option: Hiwonder TonyPi Pro

### Option C: Premium Humanoid Lab

* Unitree G1 Humanoid

---

## 🧩 Architecture Summary

| Component      | Hardware          | Purpose               |
| -------------- | ----------------- | --------------------- |
| Simulation Rig | RTX PC            | Training & simulation |
| Edge Brain     | Jetson Orin       | AI inference          |
| Sensors        | RealSense + LiDAR | Environment data      |
| Actuators      | Unitree Robot     | Physical movement     |

---

## ☁️ Cloud-Native Option (Ether Lab)

* AWS g5 / g6e instances
* Approx. **$205 per quarter**
* Jetson + robot still required for physical deployment

---
