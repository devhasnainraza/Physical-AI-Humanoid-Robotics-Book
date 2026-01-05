---
id: foundations-physical-ai
title: '1.1 ROS 2: The Nervous System'
sidebar_label: '1.1 ROS 2 Foundations'
description: 'Nodes, Topics, Services, and the DDS Middleware.'
---

# 1.1 ROS 2: The Nervous System

**"A robot without a middleware is just a pile of parts screaming in different languages."**

In this lab, we build the digital nervous system of our robot using **ROS 2 (Robot Operating System)**.
ROS 2 is not an OS (like Windows); it is a **Middleware**. It handles the communication between the Camera (Eye) and the Motor (Hand) so you don't have to write raw socket code.

## 🎯 Lab Objectives
1.  **Install ROS 2** (Humble Hawksbill).
2.  **Write a Publisher/Subscriber** in Python.
3.  **Understand DDS**: The magic underneath.

---

## 1.1.1 The Architecture: Nodes & Topics

### The Node
A **Node** is a process that performs a specific task.
*   `camera_node`: Reads images.
*   `brain_node`: Detects objects.
*   `motor_node`: Spins wheels.

### The Topic (Pub/Sub)
Nodes talk via **Topics**. It's like a Radio Station.
*   **Publisher**: Broadcasts data (e.g., `/camera/image_raw`).
*   **Subscriber**: Listens to data.
*   **One-to-Many**: One camera can talk to the Brain, the Logger, and the GUI simultaneously.

### The Service (Req/Res)
Sometimes you need a reply. "Pick up the cup." "Done."
*   **Client**: Sends Request.
*   **Server**: Sends Response.

---

## 1.1.2 Hands-On: Your First Node

We will create a "Talker" (Publisher) and a "Listener" (Subscriber).

### 1. Create Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_first_package
```

### 2. The Publisher Code (`simple_publisher.py`)
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('talker_node')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello Physical AI!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### 3. The Subscriber Code (`simple_subscriber.py`)
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('listener_node')
        self.subscription = self.create_subscription(
            String, 'chatter', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
```

---

## 1.1.3 DDS: The Secret Sauce

ROS 1 used a central server (roscore). If it died, the robot died.
ROS 2 uses **DDS (Data Distribution Service)**.
*   **Decentralized**: No master. Nodes find each other automatically (Discovery).
*   **QoS (Quality of Service)**:
    *   *Reliable*: Like TCP. Guaranteed delivery (use for parameters).
    *   *Best Effort*: Like UDP. Fast, drop packets if needed (use for Video/LiDAR).

**Pro Tip**: If your nodes can't see each other, check your `ROS_DOMAIN_ID`. It separates robots on the same network.

## 1.1.4 Quiz

1.  **What happens if the ROS 2 "Master" dies?**
    *   a) The robot stops.
    *   b) Nothing. There is no Master in ROS 2.
    *   *Answer: b*

2.  **Which QoS setting should I use for a 30Hz Camera stream?**
    *   a) Reliable (Guaranteed delivery).
    *   b) Best Effort (Low latency, okay to drop frames).
    *   *Answer: b*