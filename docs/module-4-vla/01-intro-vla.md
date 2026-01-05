---
id: intro-vla
title: '4.1 Vision-Language-Action (VLA)'
sidebar_label: '4.1 Intro to VLA'
description: 'The convergence of Vision, Language, and Control.'
---

# 4.1 Vision-Language-Action (VLA)

**"From 'See, Think, Act' to 'End-to-End'."**

Classic robotics is modular: Perception $\to$ Planning $\to$ Control.
VLA is end-to-end: Pixels + Text $\to$ Action.

## 🎯 Lab Objectives
1.  **Understand Multimodality**: How to merge Text and Image.
2.  **CLIP**: The glue that holds VLA together.

---

## 4.1.1 The Pre-VLA Era (CLIP)

**CLIP (Contrastive Language-Image Pre-training)** by OpenAI changed everything.
It maps Images and Text into the **Same Vector Space**.
*   Embedding("Dog image") $\approx$ Embedding("The word Dog").
*   Embedding("Cat image") $\neq$ Embedding("The word Dog").

**Why this matters**:
If a robot knows what "Pick up the apple" means in Text, and it knows what an "Apple" looks like in Image, it can connect the two **without explicit programming**.

---

## 4.1.2 The VLA Pipeline

1.  **User**: "Pick up the red block."
2.  **VLM (Vision Language Model)**: Sees the scene, identifies the red block's pixel coordinates.
3.  **Action Head**: Converts coordinates/intent into End-Effector Delta $(\Delta x, \Delta y, \Delta z, \text{Grip})$.