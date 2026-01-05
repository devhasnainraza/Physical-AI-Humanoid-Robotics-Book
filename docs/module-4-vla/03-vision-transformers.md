---
id: vision-transformers
title: '03. Vision Transformers (ViT)'
sidebar_label: '03. Vision Transformers'
---

# Vision Transformers (ViT)

The Transformer architecture, originally designed for NLP, has been successfully adapted for Computer Vision, challenging the dominance of Convolutional Neural Networks (CNNs).

## How ViT Works

1.  **Patching**: The input image is split into fixed-size patches (e.g., 16x16 pixels).
2.  **Linear Projection**: Each patch is flattened and linearly projected into an embedding vector.
3.  **Positional Embeddings**: Information about the patch's position is added to the embedding.
4.  **Transformer Encoder**: The sequence of embeddings is processed by standard Transformer layers (Self-Attention and MLP).
5.  **Classification Head**: The output of the special [CLS] token (or global average pooling) is used for prediction.

## Importance for VLA

ViTs provide a unified architecture for processing both text (tokens) and images (patches), making them ideal backbones for multimodal models.

## CLIP (Contrastive Language-Image Pre-Training)

CLIP is a seminal model that learns to associate images and text by training on 400 million (image, text) pairs. It learns a joint embedding space where semantically similar images and text are close together. This zero-shot capability is crucial for open-vocabulary robot perception.
