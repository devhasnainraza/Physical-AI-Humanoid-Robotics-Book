---
id: foundation-models
title: '4.2 Robotics Foundation Models'
sidebar_label: '4.2 RT-1 & RT-2'
description: 'Google DeepMind RT-1, RT-2, and Action Tokenization.'
---

# 4.2 Robotics Foundation Models

**"One model to rule them all."**

Can we train a single neural network to do *everything*?

## 4.2.1 RT-1 (Robotics Transformer 1)

DeepMind (2022).
*   **Architecture**: EfficientNet (Vision) + Transformer (Logic).
*   **Input**: Images + Text Instruction.
*   **Output**: **Action Tokens**.

### Tokenizing Action
How do you output "Move Arm"?
*   Discretize actions into 256 bins.
*   $x  [-1, 1] \to \text{Token } 0..255$.
*   Output is just a sequence of tokens: `[Token_x, Token_y, Token_z, Token_Grip]`.

## 4.2.2 RT-2 (Vision-Language-Action)

DeepMind (2023).
*   **Idea**: Take a VLM trained on the Internet (PaLI-X) and fine-tune it on robot data.
*   **Result**: The robot gains **Semantic Reasoning**.
    *   *Prompt*: "Move the extinct animal to the trash."
    *   *Reasoning*: "Extinct = Dinosaur. Plastic Dinosaur is on table. Trash is on floor."
    *   *Action*: Pick Dinosaur, Move to Trash.

This was impossible with RT-1 (which only knew what it was trained on).