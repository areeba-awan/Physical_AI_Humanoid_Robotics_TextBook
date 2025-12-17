---
sidebar_position: 1
title: "4.1 Introduction to VLA"
description: Vision-Language-Action models for robotics
keywords: [VLA, vision, language, action, multimodal, AI]
---

# Chapter 4.1: Introduction to VLA

## Learning Objectives

- Understand Vision-Language-Action (VLA) models
- Learn the components of VLA systems
- Explore state-of-the-art VLA architectures
- Set up a VLA development environment

## What is VLA?

**Vision-Language-Action (VLA)** models combine three modalities:

```
┌─────────────────────────────────────────────────────────────┐
│                    VLA ARCHITECTURE                          │
│                                                              │
│   ┌─────────┐     ┌─────────────────────┐     ┌─────────┐  │
│   │ Camera  │────>│                     │────>│ Robot   │  │
│   │ (Vision)│     │   Multimodal        │     │ (Action)│  │
│   └─────────┘     │   Transformer       │     └─────────┘  │
│                   │                     │                   │
│   ┌─────────┐     │   - Encode vision   │                   │
│   │ Natural │────>│   - Understand text │                   │
│   │ Language│     │   - Generate actions│                   │
│   └─────────┘     └─────────────────────┘                   │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### Key Capabilities

| Capability | Example |
|------------|---------|
| Grounding | "Pick up the red apple" → Locate red apple |
| Reasoning | "Put the cup near the plate" → Spatial reasoning |
| Planning | "Make a sandwich" → Break into sub-tasks |
| Execution | Generate motor commands |

## VLA Model Landscape

### RT-2 (Robotics Transformer 2)

```
┌────────────────┐
│ Vision Encoder │──┐
│    (ViT)       │  │    ┌─────────────┐    ┌──────────┐
└────────────────┘  ├───>│   PaLM-E    │───>│  Actions │
┌────────────────┐  │    │  (54B)      │    │ (tokens) │
│ Language Input │──┘    └─────────────┘    └──────────┘
└────────────────┘
```

### OpenVLA

- Open-source VLA model
- 7B parameters
- Fine-tunable on custom tasks

### Other Models

- **PaLM-E**: 562B multimodal model
- **Gato**: Generalist agent
- **RT-X**: Cross-embodiment transfer

## VLA Components

### 1. Vision Encoder

```python
from transformers import ViTModel

# Pre-trained vision encoder
vision_encoder = ViTModel.from_pretrained('google/vit-base-patch16-224')

# Process image
vision_features = vision_encoder(images).last_hidden_state
```

### 2. Language Encoder

```python
from transformers import T5EncoderModel

# Language encoder
language_encoder = T5EncoderModel.from_pretrained('t5-base')

# Process instruction
instruction = "Pick up the red block"
language_features = language_encoder(tokenized_instruction)
```

### 3. Action Decoder

```python
# Action tokens vocabulary
action_vocab = {
    0: 'move_to',
    1: 'grasp',
    2: 'release',
    3: 'rotate',
    # ... 256 action tokens
}

# Decode action tokens to robot commands
def tokens_to_actions(tokens):
    actions = []
    for token in tokens:
        action = action_vocab[token]
        actions.append(action)
    return actions
```

## Setup Environment

```bash
# Install VLA dependencies
pip install torch transformers accelerate
pip install open-vla  # If using OpenVLA

# Clone OpenVLA
git clone https://github.com/openvla/openvla.git
cd openvla
pip install -e .
```

## Quick Example

```python
from openvla import OpenVLA

# Load model
model = OpenVLA.from_pretrained("openvla/openvla-7b")

# Inference
image = load_image("scene.jpg")
instruction = "Pick up the blue cup"

actions = model.predict(image, instruction)
# Output: [move_to(0.3, 0.5), grasp(), lift(), ...]
```

## Hands-on Lab

### Lab 4.1: Run a VLA Model

1. Install OpenVLA
2. Load a pre-trained model
3. Run inference on sample images
4. Visualize predicted actions

## Summary

- VLA combines vision, language, and action
- Models like RT-2 and OpenVLA enable instruction-following
- VLA reduces manual programming for robot tasks

[Continue to Chapter 4.2 →](/docs/module-4-vla/chapter-2-vision)


