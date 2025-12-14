---
sidebar_position: 1
title: ROS 2 Nodes & Topics
---

# ROS 2 Nodes, Topics, and Services

## The Nervous System of Robotics

In Physical AI, **ROS 2 (Robot Operating System 2)** acts as the central nervous system. It allows different parts of a robot—sensors (eyes/ears), actuators (muscles), and AI models (brain)—to communicate effectively.

### Key Concepts

1.  **Nodes**: Individual processes that perform computation.
    - Example: A `camera_node` captures images, while a `navigation_node` decides where to go.
2.  **Topics**: Channels for data streams (Pub/Sub).
    - Example: `camera_node` publishes to `/camera/image_raw`, and `object_detector_node` subscribes to it.
3.  **Services**: Synchronous Request/Response communication.
    - Example: "Turn on the LED" (Request) -> "LED is on" (Response).

## Interactive Diagram

*(Placeholder for an interactive diagram showing Nodes connected by Topics)*

## Hands-On: Listing Active Topics

To see what's happening in your robot's nervous system, use the CLI:

```bash
ros2 topic list
ros2 node list
```
