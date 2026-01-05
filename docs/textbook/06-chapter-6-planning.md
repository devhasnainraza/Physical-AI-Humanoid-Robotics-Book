---
id: chapter-6-planning
title: 'Chapter 6: Planning'
sidebar_label: '6. Planning'
description: 'How robots decide where to go: A*, RRT, and Behavior Trees.'
---

# Chapter 6: Planning

**"A robot without a plan is just a random number generator."**

Planning is the brain's executive function. It bridges "Here I am" (Perception) and "This is what I want" (Goal) with "This is how I get there" (Action).

## 🎯 Learning Objectives
1.  **Visualize Configuration Space**: Why we treat robots as points.
2.  **Master A***: The algorithm behind Google Maps.
3.  **Understand RRT**: How to plan for complex 7-DOF arms.

---

## 6.1 The Intuition: Configuration Space (C-Space)

### 6.1.1 The "Fat Robot" Problem
Imagine navigating a Pac-Man maze.
*   If you are a single pixel (Point), it's easy.
*   If you are a giant fat Pac-Man, you might get stuck in corridors.
Checking collision for a "fat" shape at every step is computationally expensive.

### 6.1.2 The Solution: Shrink the Robot, Grow the Wall
We transform the world so the robot becomes a **Point**.
*   **Real World**: Robot has radius $R$. Wall is thin.
*   **C-Space**: Robot is a Point. Wall is "inflated" by radius $R$.
*   **Benefit**: Now we can just draw a line between Start and Goal. If the line doesn't touch the inflated wall, the path is safe.

For a robotic arm, C-Space is the N-dimensional space of joint angles ($	heta_1, 	heta_2...$).

---

## 6.2 Graph Search: The GPS (A*)

Grid-based planning is like finding a route on a map.

### 6.2.1 Dijkstra's Algorithm (The "Flood")
Imagine pouring water at the Start. The water spreads in all directions. Eventually, it touches the Goal. The path the water took is the optimal path.
*   **Pros**: Guaranteed optimal.
*   **Cons**: Wastes time exploring opposite directions.

### 6.2.2 A* (The "Smart" Flood)
Instead of spreading uniformly, we bias the "water" to flow *towards* the goal.
We use a **Heuristic ($h$)**: "The straight-line distance to the goal".

**Cost Function**: $f(n) = g(n) + h(n)$
*   $g(n)$: "How much gas I used to get here."
*   $h(n)$: "How far I *think* the goal is."

A* always explores the node with the lowest $f(n)$. It is essentially "Guided Dijkstra".

---

## 6.3 Sampling-Based Planning: The Lightning (RRT)

What if you have a 7-Joint Arm? The grid would be $100 	imes 100 	imes 100 …$ (7 times). That's $10^{14}$ cells! Too big for A*.

We use **Randomness**.

### 6.3.1 RRT (Rapidly-exploring Random Trees)
Think of how lightning grows. It zaps randomly, branching out until it hits the ground.

**The Algorithm**:
1.  **Tree**: Start at $q_{start}$.
2.  **Sample**: Pick a random point in space $q_{rand}$.
3.  **Nearest**: Find the closest branch tip $q_{near}$.
4.  **Extend**: Grow a small step from $q_{near}$ towards $q_{rand}$. Let's call this $q_{new}$.
5.  **Check**: Did $q_{new}$ hit an obstacle?
    *   No: Add it to the tree.
    *   Yes: Discard.
6.  **Repeat** until a branch hits $q_{goal}$.

**Why it works**: It rapidly explores empty spaces without needing a grid.

---

## 6.4 Decision Making: The Brain

Path planning tells you *how* to move. Decision making tells you *what* to do.

### 6.4.1 Finite State Machines (FSM) - The "Mario" Logic
*   **State 1**: Idle. (Wait for button).
*   **State 2**: Jump. (If button pressed).
*   **State 3**: Fall. (If jump done).
*   **Problem**: Spaghetti code. "What if I get hit while jumping?" You need arrows from every state to every other state.

### 6.4.2 Behavior Trees (BT) - The "Corporate Manager" Logic
Standard in ROS 2. A tree of tasks.
*   **Sequence (->)**: Do Step 1, THEN Step 2, THEN Step 3. (AND logic).
    *   "Open Door" -> "Walk Through" -> "Close Door".
*   **Fallback (?)**: Try Plan A. If it fails, Try Plan B. (OR logic).
    *   "Open Door" (Locked?) -> "Kick Door".

---

## 🛠️ Practical Implementation (Python)

```python
import heapq

def a_star(grid, start, goal):
    # Priority Queue: (Cost, x, y)
    queue = [(0, start)]
    came_from = {}
    cost_so_far = {start: 0}
    
    while queue:
        _, current = heapq.heappop(queue)
        
        if current == goal:
            break
            
        for next_node in get_neighbors(current, grid):
            new_cost = cost_so_far[current] + 1
            
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristic(next_node, goal)
                heapq.heappush(queue, (priority, next_node))
                came_from[next_node] = current
                
    return reconstruct_path(came_from, start, goal)

def heuristic(a, b):
    # Manhattan distance
    return abs(a[0] - b[0]) + abs(a[1] - b[1])
```

## 6.5 Quiz

1.  **Why can't we use A* for a 7-DOF robot arm?**
    *   a) A* is too old.
    *   b) The "Curse of Dimensionality" makes the grid too big.
    *   c) Robot arms don't have GPS.
    *   *Answer: b*

2.  **In RRT, what ensures the tree covers the whole space?**
    *   a) Random sampling.
    *   b) A grid.
    *   c) Gravity.
    *   *Answer: a*

3.  **What is the benefit of "Inflating" obstacles in Costmaps?**
    *   a) It looks cool.
    *   b) It allows us to treat the robot as a single point for collision checking.
    *   c) It makes the map smaller.
    *   *Answer: b*