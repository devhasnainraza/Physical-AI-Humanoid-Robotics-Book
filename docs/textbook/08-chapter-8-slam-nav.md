---
id: chapter-8-slam-nav
title: 'Chapter 8: SLAM & Navigation'
sidebar_label: '8. SLAM & Nav'
description: 'How robots map the world and move through it without hitting things.'
---

# Chapter 8: SLAM & Navigation

**"To move is simple. To know where you are moving is the challenge."**

Imagine waking up in a pitch-black unfamiliar room. You reach out, feel a wall. You step forward, feel a table. Slowly, you build a mental map. This is SLAM.

## 🎯 Learning Objectives
1.  **Solve the Chicken-and-Egg Problem**: Localization needs Mapping; Mapping needs Localization.
2.  **Visual GraphSLAM**: The "Rubber Band" intuition.
3.  **Master Nav2**: Global vs Local Planners.

---

## 8.1 The Intuition: Lost in the Forest

You are dropped in a forest with a blindfold. You count your steps (Odometry).
*   **Step 1**: "I walked 10 steps North." (Uncertainty: $\pm 1$ step).
*   **Step 2**: "I walked 10 steps East." (Uncertainty: $\pm 2$ steps).
*   **Observation**: You bump into a unique tree (Landmark).
*   **Step 100**: You walk in a circle and bump into the **same** unique tree.

**Loop Closure**:
Your step counting says you should be 5 meters away from the tree. But your hands tell you that you are *touching* the tree.
*   **Conclusion**: Your step counting was wrong. You retrospectively correct your entire path map to make the start and end match. This is SLAM.

---

## 8.2 The Math: GraphSLAM

We model the world as a **Graph**.
*   **Nodes**: Robot poses at different times ($x_1, x_2, ...$) and Landmarks ($L_1, L_2...$).
*   **Edges**: Constraints.
    *   *Odometry Edge*: "I think $x_2$ is 1m ahead of $x_1$." (Spring stiffness = Low).
    *   *Measurement Edge*: "I see Landmark $L_1$ 2m away from $x_2$." (Spring stiffness = High).

### 8.2.1 The Optimization
The graph is a system of springs. When we find a **Loop Closure** (connect the last node to the first node), the whole graph "snaps" into the correct shape to minimize the tension (Error) in all springs.

Mathematically, this is **Non-Linear Least Squares**:
$$ x^* = \arg\min_x \sum_i \| f_i(x) - z_i \|^2_{\Sigma_i} $$

---

## 8.3 The Navigation Stack (Nav2)

Once we have a map, how do we move? The ROS 2 Navigation Stack (Nav2) splits this into layers.

### 8.3.1 The Global Planner (The Strategist)
*   **Job**: Find a path from A to B across the whole map.
*   **Algorithm**: A* or Dijkstra.
*   **Analogy**: Using Google Maps to find a route from NY to LA. It doesn't care about potholes; it just cares about highways.

### 8.3.2 The Local Planner (The Pilot)
*   **Job**: Follow the Global Path while avoiding *dynamic* obstacles (people, dogs).
*   **Algorithm**: DWA (Dynamic Window Approach) or TEB (Timed Elastic Band).
*   **Analogy**: The actual driver steering the car. Google Maps says "Turn Left", but the driver sees a pedestrian and brakes.

### 8.3.3 Costmaps (The Danger Zones)
The robot views the world as a grid of "Danger".
1.  **Static Layer**: Walls from the SLAM map. (Black).
2.  **Inflation Layer**: We draw a "Ghost Zone" around walls.
    *   *Reason*: The robot has a radius. If the center of the robot is 1cm from the wall, the *body* is hitting the wall. We inflate obstacles by the robot's radius so the planner treats the robot as a point.
3.  **Obstacle Layer**: Live data from LiDAR. (People walking).

---

## 8.4 Common Pitfalls (Student Support)

**"My robot is spinning in circles!"**
*   **Check**: Is your LiDAR frame correct? If the robot rotates left but the LiDAR data rotates right, the AMCL (Localization) will panic and spin the robot to try to fix it.

**"My robot hits walls!"**
*   **Check**: Inflation Radius. Is it smaller than your actual robot?
*   **Check**: LiDAR update rate. If you are moving fast but LiDAR updates slowly, you are driving blind.

---

## 8.5 Quiz

1.  **What is a "Loop Closure"?**
    *   a) Tying your shoelaces.
    *   b) Recognizing a previously visited place to correct map drift.
    *   c) The end of a while loop.
    *   *Answer: b*

2.  **Why do we need both a Global and Local planner?**
    *   a) Global plans for the static map; Local handles dynamic obstacles and physics.
    *   b) Two is better than one.
    *   *Answer: a*

3.  **If your robot radius is 0.5m, what should your Inflation Radius be?**
    *   a) 0.1m
    *   b) At least 0.5m (plus a safety margin).
    *   c) 0m
    *   *Answer: b*