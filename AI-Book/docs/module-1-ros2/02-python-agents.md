---
sidebar_position: 2
title: Python Agents with rclpy
---

# Python Agents using `rclpy`

## Writing Your First Reflex Agent

In Physical AI, an "agent" is a program that perceives its environment and takes action. We use `rclpy` (ROS Client Library for Python) to write these agents.

### The Basic Structure

Every ROS 2 agent starts by inheriting from the `Node` class.

```python
import rclpy
from rclpy.node import Node

class ReflexAgent(Node):
    def __init__(self):
        super().__init__('reflex_agent')
        self.publisher_ = self.create_publisher(String, 'motor_commands', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = "Move Forward"
        self.publisher_.publish(msg)
        self.get_logger().info('Deciding to: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    agent = ReflexAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()
```

### Explanation

1.  **`create_publisher`**: Sets up a voice to speak to the motors.
2.  **`create_timer`**: Sets a heartbeat for the agent to act periodically.
3.  **`spin`**: Keeps the agent alive and listening/acting.

<HardwareOnly profile="Unitree_Go2">

### Unitree Go2 Specifics

For the Go2, your topics will be slightly different. You will publish to `/sport/request` to trigger high-level athletic motions.

</HardwareOnly>
