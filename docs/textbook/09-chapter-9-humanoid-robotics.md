---
id: chapter-9-humanoid-robotics
title: 'Chapter 9: Humanoid Robotics'
sidebar_label: '9. Humanoids'
description: 'Bipedal walking, ZMP, and why robots fall down.'
---

# Chapter 9: Humanoid Robotics

**"Two legs are harder than four wheels, but they can go where wheels cannot."**

Humanoid robotics is the Holy Grail. It combines the hardest problems of Perception, Planning, and Control into an unstable package that fights gravity every millisecond.

## 🎯 Learning Objectives
1.  **Understand Stability**: Why you don't fall over.
2.  **The ZMP Criterion**: The Golden Rule of Walking.
3.  **LIPM**: The simplified physics model used by Honda ASIMO.

---

## 9.1 The Intuition: Don't Tip the Table

Imagine a table with one leg. It falls.
Imagine a table with four legs. It is stable.
*   The area between the legs is the **Support Polygon**.
*   If the Center of Mass (CoM) is inside the polygon, it stands.

**Now, imagine the table is accelerating (on a bus).**
Even if the CoM is inside, if the bus brakes hard, the table tips over.
*   **Static Stability**: Only cares about CoM. (Good for standing).
*   **Dynamic Stability**: Cares about CoM + Acceleration. (Good for walking).

---

## 9.2 The Zero Moment Point (ZMP)

The ZMP is the point on the ground where the robot is "pushing" effectively.
Think of balancing a broom on your palm.
*   To keep the broom upright, you must move your hand (ZMP) to stay under the broom's center.
*   If the ZMP reaches the edge of your hand (Support Polygon), you can't push anymore, and the broom falls.

**The Rule**: To not fall, the ZMP must remain strictly *inside* the Support Polygon (the footprint).

$$ p_{zmp} = x_{com} - \frac{z_{com}}{g} \ddot{x}_{com} $$

---

## 9.3 The Linear Inverted Pendulum (LIPM)

A full humanoid has 50 motors and complex mass distribution. Calculating the ZMP for the full body is slow.
We cheat. We model the robot as a **Stick Figure** (Inverted Pendulum).

**Assumptions**:
1.  The robot keeps its hips at a constant height ($z = const$).
2.  The legs are massless.

**Equation of Motion**:
$$ \ddot{x} = \omega^2 (x - p_{zmp}) $$
where $\omega = \sqrt{g/z}$.

This equation says: "My acceleration depends on how far my CoM is from my Foot."
*   If CoM is ahead of Foot ($x > p_{zmp}$), I accelerate forward.
*   If CoM is behind Foot, I accelerate backward.
*   **Walking is just throwing your CoM forward and catching it with the next step.**

---

## 9.4 Walking Pattern Generation

How do we actually make it walk?

1.  **Footstep Planner**: "I want to step 30cm forward."
2.  **Preview Control**: Solve the LIPM equation backwards. "If I want to be at X in 1 second, where should I put my ZMP now?"
3.  **Inverse Kinematics**: "The math says my CoM should be here and my Foot there. Move the knee joints to make that happen."

---

## 9.5 Whole-Body Control (WBC)

While the legs handle the ZMP, the arms and torso can do other things.
WBC allows us to stack tasks:

*   **Priority 1**: Balance (Don't fall).
*   **Priority 2**: Wave Hand (Social).
*   **Priority 3**: Look at human (Perception).

If Waving Hand makes the robot fall, WBC cancels the wave. **Survival comes first.**

---

## 9.6 Quiz

1.  **If the ZMP touches the edge of the foot, what is happening?**
    *   a) The robot is perfectly stable.
    *   b) The robot is beginning to tip over.
    *   *Answer: b*

2.  **Why do we use the LIPM model?**
    *   a) Because it's perfectly accurate.
    *   b) Because it simplifies the complex dynamics into a linear equation we can solve fast.
    *   *Answer: b*

3.  **What is "Static Stability"?**
    *   a) Walking without falling.
    *   b) Standing still without falling (CoM inside feet).
    *   *Answer: b*