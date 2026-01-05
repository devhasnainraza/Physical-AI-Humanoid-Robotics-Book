---
id: vla-architectures
title: '05. State-of-the-Art VLA Architectures'
sidebar_label: '05. VLA Architectures'
---

# State-of-the-Art VLA Architectures

Vision-Language-Action (VLA) models represent the frontier of embodied AI, integrating perception, reasoning, and control into a single end-to-end system.

## RT-1 (Robotics Transformer 1)
**Google DeepMind**

RT-1 uses a ViT backbone for vision and tokenizes robot actions. It is trained on a large dataset of real-world robot demonstrations.
-   **Architecture**: EfficientNet backbone + FiLM layers for conditioning on language + Token Learner + Transformer.
-   **Output**: Discretized action tokens.

## RT-2 (Robotics Transformer 2)
**Google DeepMind**

RT-2 shows that large generic VLMs can be fine-tuned to output robot actions directly.
-   **Approach**: Fine-tuning PaLI-X and PaLM-E models.
-   **Key Insight**: The model retains its web-scale knowledge (reasoning, semantic understanding) while learning to control the robot, enabling generalization to unseen commands.

## PaLM-E
**Google**

An embodied multimodal language model.
-   **Inputs**: Text, Images, State vectors (sensor data).
-   **Outputs**: Text (planning) or control commands.
-   **Mechanism**: Injects continuous sensor observations into the language model's embedding space.

## Octo
**UC Berkeley / Stanford**

An open-source general-purpose robot policy.
-   **Transformer-based**: Uses a diffusion head or categorical head for action prediction.
-   **Training**: Trained on the Open X-Embodiment dataset (diverse robot data).
