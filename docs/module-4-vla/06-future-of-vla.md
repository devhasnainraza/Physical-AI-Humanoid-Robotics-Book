---
id: future-of-vla
title: '06. The Future of VLA'
sidebar_label: '06. Future of VLA'
---

# The Future of VLA and Physical AI

The convergence of large foundation models and robotics is accelerating, but significant challenges remain.

## Current Limitations
1.  **Latency**: Large models are slow to inference, which is critical for real-time control (10Hz+).
2.  **Data Scarcity**: We lack "Internet-scale" robot data compared to text or images.
3.  **Safety**: End-to-end neural networks are "black boxes", making safety guarantees difficult.

## Emerging Trends

### 1. System 1 vs. System 2
Combining fast, reactive policies (System 1 - small networks) with slow, deliberative reasoning (System 2 - large VLA models). The VLA sets the high-level goal or waypoints, and the small policy executes the motion control.

### 2. General Purpose Robots
Moving away from task-specific engineering to general-purpose robots (like humanoids) that can perform any task described in natural language.

### 3. Sim-to-Real at Scale
Using procedural generation and generative AI to create infinite simulation training data to pre-train VLAs before fine-tuning on real hardware.

### 4. Continuous Learning
Robots that learn from their own mistakes and update their models online, without forgetting previous skills.
