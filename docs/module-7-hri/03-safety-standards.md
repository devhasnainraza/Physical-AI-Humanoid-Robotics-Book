---
id: safety-standards
title: '7.3 Lab: Safety & E-Stop'
sidebar_label: '7.3 Lab: Safety'
description: 'Implementing a software E-Stop and Speed Limiter.'
---

# 7.3 Lab: Safety & E-Stop

**"Stop."**

Safety is not a feature; it is a requirement.

## 🎯 Lab Objectives
1.  **Write a Safety Node**.
2.  **Implement Speed Scaling** based on proximity.
3.  **Override `/cmd_vel`** (Multiplexer).

---

## 7.3.1 The Architecture: Twist Mux

We don't want 10 nodes fighting for `/cmd_vel`.
We use `twist_mux`.
*   **Priority 100**: Safety Stop (0, 0).
*   **Priority 50**: Teleop.
*   **Priority 10**: Nav2.

## 7.3.2 The Safety Node

```python
class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.stop_pub = self.create_publisher(Twist, '/safety/cmd_vel', 10)
        
    def scan_cb(self, msg):
        min_dist = min(msg.ranges)
        
        if min_dist < 0.3: # 30cm
            # E-STOP!
            stop_msg = Twist() # Zeros
            self.stop_pub.publish(stop_msg)
            self.get_logger().error("EMERGENCY STOP!")
```

## 7.3.3 Configuring Twist Mux

```yaml
topics:
  - name: safety
    topic: safety/cmd_vel
    timeout: 0.5
    priority: 100
  - name: nav
    topic: cmd_vel
    timeout: 0.5
    priority: 10
```

Now, if `SafetyNode` publishes zeros, `twist_mux` will ignore Nav2 and send Zeros to the wheels.

## 7.3.4 Quiz

1.  **Why do we use a Multiplexer (Mux)?**
    *   a) To allow higher-priority signals (Safety) to override lower-priority ones (Nav) cleanly.
    *   b) To mix signals.
    *   *Answer: a*

2.  **What is a "Dead Man's Switch"?**
    *   a) A switch you must hold to keep the robot moving. If you let go (or die), it stops.
    *   b) A switch for zombies.
    *   *Answer: a*
