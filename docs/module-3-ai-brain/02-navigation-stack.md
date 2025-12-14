---
sidebar_position: 2
title: Nav2 Architecture
---

# Nav2: The Navigation Stack

## 1. Moving from A to B

**Nav2** is the industry-standard navigation framework for ROS 2. It turns a "Goal Pose" (x, y, theta) into motor commands, while avoiding obstacles.

### 1.1 The Architecture

Nav2 is not a single node; it is a collection of servers managed by a Behavior Tree.

1.  **Global Planner**: Calculates the path from Start to Goal.
    -   *Algorithms*: Dijkstra, A* (A-Star), Smac Planner (Hybrid A*).
    -   *Map*: Uses the Static Map (from SLAM).
2.  **Local Planner (Controller)**: Follows the path while avoiding dynamic obstacles (people, dogs).
    -   *Algorithms*: DWB (Dynamic Window Approach), MPPI (Model Predictive Path Integral).
    -   *Map*: Uses the Local Costmap (rolling window).
3.  **Recoveries (Behaviors)**: What to do if stuck.
    -   *Examples*: Spin in place, back up, clear costmap.

---

## 2. A* Algorithm: The Planner's Core

A* finds the shortest path by minimizing $f(n) = g(n) + h(n)$.
-   $g(n)$: Cost from Start to current node $n$.
-   $h(n)$: Heuristic cost from $n$ to Goal (usually Euclidean distance).

### Pseudocode

```python
open_set = {start_node}
came_from = {}

g_score[start] = 0
f_score[start] = heuristic(start, goal)

while open_set is not empty:
    current = node in open_set with lowest f_score
    if current == goal:
        return reconstruct_path(came_from, current)

    open_set.remove(current)
    
    for neighbor in neighbors(current):
        tentative_g = g_score[current] + dist(current, neighbor)
        if tentative_g < g_score[neighbor]:
            came_from[neighbor] = current
            g_score[neighbor] = tentative_g
            f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
            if neighbor not in open_set:
                open_set.add(neighbor)
```

In robotics, the cost function also includes the **Costmap Value** (don't hug the walls!).

---

## 3. Costmaps: The World Representation

A **Costmap** is a 2D grid where each cell has a value (0-255).
-   **0**: Free space (Safe).
-   **254**: Lethal Obstacle (Wall).
-   **255**: Unknown.
-   **1-253**: Inflation Radius (Danger zone near walls).

### 3.1 Layered Costmaps

The final map is a sum of layers:
1.  **Static Layer**: The building walls (from SLAM).
2.  **Obstacle Layer**: Real-time LiDAR hits (People).
3.  **Inflation Layer**: Adds a gradient around obstacles.
4.  **Voxel Layer**: 3D obstacles (e.g., tables) mapped to 2D.

---

## 4. Behavior Trees (BT)

Nav2 uses an XML-based **Behavior Tree** to orchestrate the logic. This is more flexible than a state machine.

**Example Logic:**
```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6">
      <PipelineSequence name="NavigateWithReplanning">
        
        <!-- Every 1s, plan a new path -->
        <RateController hz="1.0">
          <ComputePathToPose goal="{goal}" path="{path}"/>
        </RateController>

        <!-- Follow the path -->
        <FollowPath path="{path}" controller_id="FollowPath"/>
        
      </PipelineSequence>

      <!-- If failed, try to recover -->
      <Sequence name="RecoveryActions">
        <ClearEntireCostmap name="ClearLocalCostmap"/>
        <Spin spin_dist="1.57"/>
        <Wait wait_duration="5"/>
      </Sequence>
      
    </RecoveryNode>
  </BehaviorTree>
</root>
```

---

## 5. MPPI for Humanoids

For humanoid robots, standard controllers (DWB) often fail because they assume a circular or rectangular footprint that moves smoothly.

**MPPI (Model Predictive Path Integral)** is a predictive controller that:
1.  Simulates thousands of random trajectories into the future on the GPU.
2.  Scores them (low cost = good, hit wall = bad).
3.  Executes the weighted average of the best paths.

MPPI is robust for erratic movement and is highly recommended for legged robots.