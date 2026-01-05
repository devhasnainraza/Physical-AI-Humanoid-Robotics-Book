---
id: chapter-10-ethics
title: 'Chapter 10: Ethics & Safety'
sidebar_label: '10. Ethics'
description: 'The Trolley Problem, Bias, and the ISO Standards governing robotic life.'
---

# Chapter 10: Ethics & Safety

**"With great power comes great responsibility." — Uncle Ben**

A chatbot can say mean things. A robot can break your arm. The stakes in Physical AI are life and death.

## 🎯 Learning Objectives
1.  **Safety First**: ISO 13482 and the E-Stop.
2.  **Bias in Hardware**: Why sensors aren't neutral.
3.  **The Trolley Problem**: Engineering ethics vs Philosophy.

---

## 10.1 Safety Standards: The Robot Constitution

You cannot just build a robot and sell it. You must follow **ISO Standards**.

### 10.1.1 ISO 13482 (Personal Care Robots)
This standard governs robots that touch humans (Exoskeletons, Servants).
1.  **Safety-Rated Stop**: If the robot detects a failure, it must cut power to motors immediately (Category 0 Stop) or brake and then cut power (Category 1 Stop).
2.  **Speed & Separation Monitoring (SSM)**:
    *   *Green Zone*: Human far away $\to$ Robot moves fast.
    *   *Yellow Zone*: Human closer $\to$ Robot slows down.
    *   *Red Zone*: Human touching $\to$ Robot stops.

### 10.1.2 Power and Force Limiting (PFL)
The robot is designed so that *even if it hits you*, it cannot hurt you.
*   Low inertia arms.
*   Soft actuators.
*   Rounded edges.

---

## 10.2 Algorithmic Bias: The Racist Soap Dispenser

In 2017, a video went viral of an automatic soap dispenser that worked for white hands but not black hands.
**Why?** It used an IR sensor calibrated for light skin reflection.
**Lesson**: Bias isn't just in data; it's in physics and hardware choice.

*   **LIDAR**: Works on geometry (unbiased).
*   **Vision**: Trained on datasets (ImageNet) that are often Western-centric. A robot might recognize a "Fork" but not chop-sticks.

---

## 10.3 The Trolley Problem: Engineering Reality

**Philosophy**: "A trolley is heading for 5 people. You can switch tracks to hit 1 person. Do you do it?"
**Robotics**: "A self-driving car loses brakes. It sees a wall (kills passenger) and a pedestrian (kills pedestrian)."

In reality, robots don't make moral choices. They make **Risk Minimization** choices.
*   Calculate Kinetic Energy of impact.
*   Calculate probability of survival.
*   Choose the path of least energy transfer.

We don't code `if (pedestrian) then kill(passenger)`. We code `minimize(collision_force)`.

---

## 10.4 The Future of Work

Will robots take our jobs?
*   **Yes**: Dangerous, Dull, Dirty jobs (Mining, Sewer inspection).
*   **Maybe**: Logistics, Truck driving.
*   **No**: Empathy jobs (Nurse, Therapist, Artist).

**Cobots (Collaborative Robots)**: The future is not *Robot replacing Human*, but *Robot + Human* working together (Exoskeletons).

---

## 10.5 Quiz

1.  **What is an E-Stop?**
    *   a) An Electric Stop.
    *   b) An Emergency Stop button that physically cuts power to motors.
    *   c) A pause button.
    *   *Answer: b*

2.  **Why did the soap dispenser fail?**
    *   a) The sensor physics relied on light reflection, which varies by skin tone.
    *   b) The code was malicious.
    *   *Answer: a*

3.  **What is the goal of "Power and Force Limiting"?**
    *   a) To save electricity.
    *   b) To ensure that a collision is physically incapable of causing serious injury.
    *   *Answer: b*