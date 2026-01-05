---
id: robot-hardware
title: '1.2 Hardware Interfacing'
sidebar_label: '1.2 Hardware Interface'
description: 'micro-ROS, Serial Communication, and Custom Hardware Interfaces.'
---

# 1.2 Hardware Interfacing in ROS 2

**"Code meets Metal."**

ROS 2 runs on Linux (your brain). Motors run on Microcontrollers (your spine). This chapter bridges the gap.

## 🎯 Lab Objectives
1.  **Understand the Hardware Interface**: The standard way ROS talks to actuators.
2.  **micro-ROS**: Running ROS 2 directly on a microcontroller (ESP32/Teensy).
3.  **Lab**: Blink an LED using a ROS Topic.

---

## 1.2.1 The Problem: Real-Time Constraints

Linux is **not real-time**. It might pause your code to update Chrome.
Motors need updates every 1ms (1000Hz).
**Solution**:
*   **High Level**: ROS 2 on Jetson/Pi (Planning, Vision).
*   **Low Level**: C++ on MCU (Motor FOC, Safety Limits).
*   **Bridge**: Serial/USB/Ethernet.

---

## 1.2.2 micro-ROS: ROS on a Chip

micro-ROS puts a tiny ROS agent on the microcontroller.
*   Your MCU becomes a **First-Class ROS Node**.
*   It can Publish/Subscribe directly.

### Hands-On: ESP32 Setup

1.  **Install the micro-ROS Arduino library**.
2.  **Code (Arduino IDE)**:

```cpp
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void setup() {
  set_microros_transports();
  
  // Initialize Node
  rclc_support_t support;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  
  rcl_node_t node;
  rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support);
  
  // Create Publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_counter");
}

void loop() {
  msg.data++;
  rcl_publish(&publisher, &msg, NULL);
  delay(100);
}
```

3.  **Run the Agent on PC**:
    `docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0`

4.  **Result**: `ros2 topic echo /micro_ros_counter` shows the count!

---

## 1.2.3 ros2_control Hardware Interface

For complex robots, we don't just write random publishers. We use the `SystemInterface` C++ API.
This allows us to switch between **Simulation** and **Real Hardware** without changing a single line of control code.

*   `read()`: Get encoder values.
*   `write()`: Send torque commands.

We will cover this deeply in the **Control Systems** lab.

---

## 1.2.4 Common Protocols

1.  **UART (Serial)**: Easy. Slow. Good for debugging.
2.  **I2C/SPI**: Board level. Connect IMUs.
3.  **CAN Bus**: The gold standard.
    *   **Differential Signal**: Immune to noise.
    *   **Daisy Chain**: Connect 12 motors with 2 wires.
    *   **SocketCAN**: Linux treats CAN like a network interface (`can0`).

## 1.2.5 Quiz

1.  **Why can't we drive motors directly from the Raspberry Pi GPIO?**
    *   a) Not enough voltage.
    *   b) Linux is not real-time (jitter).
    *   c) Both.
    *   *Answer: c*

2.  **What is the benefit of micro-ROS?**
    *   a) It makes the chip faster.
    *   b) It allows the MCU to talk ROS natively (Topics/Services) without custom protocols.
    *   *Answer: b*