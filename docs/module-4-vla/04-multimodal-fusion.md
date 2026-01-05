---
id: multimodal-fusion
title: '04. Multimodal Fusion Strategies'
sidebar_label: '04. Multimodal Fusion'
---

# Multimodal Fusion Strategies

To build a Vision-Language-Action (VLA) model, we must effectively combine information from different modalities (Vision, Text, Proprioception).

## Fusion Architectures

### 1. Early Fusion
Concatenating raw features (e.g., image patches and text tokens) at the input level. The model processes them jointly from the start.
-   *Pros*: Allows rich interaction between modalities at all levels.
-   *Cons*: Computationally expensive; requires a unified architecture.

### 2. Late Fusion
Processing each modality with a separate encoder and combining them at the decision level.
-   *Pros*: Modular; allows using pre-trained specialized encoders.
-   *Cons*: Misses low-level correlations between modalities.

### 3. Cross-Attention Fusion
Using attention mechanisms to query information from one modality based on another. For example, using text instructions to attend to specific parts of an image.
-   *Example*: Perceiver, Flamingo.

## Aligning Representations

Ensuring that "an apple" in text and an image of an apple map to compatible representations is critical. Contrastive learning (like in CLIP) is the standard approach for this alignment.

## Incorporation of Action

In VLA models, "Action" is treated as another modality. Action tokens (e.g., discretized joint velocities or end-effector poses) are predicted autoregressively alongside text or visual tokens.
