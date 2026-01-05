---
id: intro-hri
title: '7.1 Lab: Human Detection'
sidebar_label: '7.1 Lab: Detection'
description: 'Using YOLOv8 to detect and track humans.'
---

# 7.1 Lab: Human Detection

**"I see you."**

To interact, we must first detect. We will use **YOLOv8** (State of the Art) wrapped in a ROS 2 node.

## 🎯 Lab Objectives
1.  **Install Ultralytics YOLO**.
2.  **Write a ROS 2 Node** that subscribes to `/camera/image_raw`.
3.  **Publish** `/human/bounding_box`.

---

## 7.1.1 Installation

```bash
pip install ultralytics
```

## 7.1.2 The Node (`yolo_node.py`)

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.model = YOLO("yolov8n.pt") # Nano model (fast)
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        
    def callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(cv_img)
        
        # Process results
        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls = int(box.cls[0])
                if cls == 0: # Class 0 is Person
                    self.get_logger().info("Human Detected!")

def main():
    rclpy.init()
    rclpy.spin(YoloNode())
```

---

## 7.1.3 3D Localization

A bounding box `[u, v, w, h]` is 2D. We need 3D `[x, y, z]`.
**Solution**:
1.  Take center pixel $(u, v)$.
2.  Read Depth $Z$ from Depth Image at $(u, v)$.
3.  Deproject using Camera Matrix $K$:
    $$ X = \frac{(u - c_x) Z}{f_x} $$

## 7.1.4 Quiz

1.  **Why use YOLOv8 "Nano"?**
    *   a) It runs fast on edge devices (Jetson).
    *   b) It is cute.
    *   *Answer: a*

2.  **How do we get the distance to the human?**
    *   a) Align the 2D box with the Depth Image.
    *   b) Guess.
    *   *Answer: a*
