---
sidebar_position: 4
title: "Capstone: The Butler Robot"
---

# Capstone Project: Building "Alfred", The Butler Robot

## 1. Project Overview

Congratulations! You have reached the end of the course. Your final challenge is to integrate everything you've learned—ROS 2, Navigation, Vision, and LLMs—to build a fully autonomous "Butler Robot".

**The Mission**:
The robot starts in the `Living Room`.
1.  **User Command**: "Alfred, please go to the kitchen and find me a soda."
2.  **Navigation**: Robot autonomously navigates to the `Kitchen`.
3.  **Search**: Robot scans the room using VSLAM to find the `Soda Can`.
4.  **Confirmation**: Robot says "I found the soda" (TTS).
5.  **Return**: Robot returns to the `Living Room`.

---

## 2. Architecture

You will build a ROS 2 system with the following architecture:

```mermaid
graph TD
    User[Microphone] -->|Audio| Whisper[Whisper Node];
    Whisper -->|Text| Brain[LLM Planner Node];
    
    Brain -->|Nav Goal| Nav2[Nav2 Stack];
    Brain -->|Search Cmd| Vision[YOLO/Isaac ROS];
    
    Vision -->|Detection (x,y)| Brain;
    Nav2 -->|Velocity| Wheels[Robot Base];
    
    subgraph "State Machine"
    Brain
    end
```

---

## 3. Step-by-Step Implementation

### Phase 1: The Map (Module 2 & 3)
1.  Launch your simulation (Gazebo) or real robot.
2.  Use `slam_toolbox` to drive around and build a map of your environment.
3.  Save the map: `ros2 run nav2_map_server map_saver_cli -f my_house_map`

### Phase 2: The Navigation (Module 3)
1.  Launch Nav2 with your map.
2.  Write a Python script that sends the robot to coordinates `(x=5.0, y=2.0)` (The Kitchen) and verifies it arrives.

### Phase 3: The Vision (Module 4)
1.  Train or download a YOLO model that detects "Soda Cans".
2.  Write a node that subscribes to `/camera/image_raw`.
3.  When `Brain` sends a `SEARCH_START` signal, the Vision node should spin the robot 360° until the soda is detected.

### Phase 4: The Brain (Module 4)
1.  Write the `alfred_brain.py` node.
2.  Use the `langchain` logic we learned to parse the user's voice command.
3.  Implement a simple State Machine:
    -   `IDLE` -> `NAVIGATING_TO_KITCHEN` -> `SEARCHING` -> `RETURNING` -> `IDLE`

---

## 4. Evaluation Criteria

| Criteria | Points |
| :--- | :--- |
| **Voice Recognition** | Correctly identifies "Soda" vs "Water" | 20 |
| **Navigation** | Reaches Kitchen without hitting walls | 30 |
| **Detection** | correctly centers the Soda in the camera frame | 30 |
| **Integration** | The entire loop runs without human intervention | 20 |

Good luck, Engineer. This is the beginning of your journey into Physical AI.
