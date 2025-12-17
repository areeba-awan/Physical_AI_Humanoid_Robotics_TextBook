---
sidebar_position: 2
title: "4.2 Vision Transformers for Robots"
description: Visual perception with transformers
keywords: [ViT, vision transformer, perception, features]
---

# Chapter 4.2: Vision Transformers for Robots

## Learning Objectives

- Understand Vision Transformer (ViT) architecture
- Extract visual features for robotics
- Fine-tune vision models for robot tasks
- Implement multi-view perception

## Vision Transformer Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                  VISION TRANSFORMER                          │
│                                                              │
│    ┌─────────────────────────────────────────────────────┐  │
│    │              Input Image (224x224)                   │  │
│    └─────────────────────────────────────────────────────┘  │
│                          │                                   │
│                    Patch Embedding                           │
│                          │                                   │
│    ┌─────────────────────────────────────────────────────┐  │
│    │  [CLS] P1  P2  P3  P4  ...  P196  +  Position Emb  │  │
│    └─────────────────────────────────────────────────────┘  │
│                          │                                   │
│              Transformer Encoder (x12)                       │
│                          │                                   │
│    ┌─────────────────────────────────────────────────────┐  │
│    │              Visual Features (768-dim)               │  │
│    └─────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

## Implementing ViT

```python
import torch
from transformers import ViTModel, ViTImageProcessor

class RobotVisionEncoder:
    def __init__(self, model_name='google/vit-base-patch16-224'):
        self.processor = ViTImageProcessor.from_pretrained(model_name)
        self.model = ViTModel.from_pretrained(model_name)

    def encode(self, image):
        inputs = self.processor(images=image, return_tensors="pt")
        outputs = self.model(**inputs)

        # Global feature (CLS token)
        global_features = outputs.last_hidden_state[:, 0]

        # Patch features for spatial reasoning
        patch_features = outputs.last_hidden_state[:, 1:]

        return global_features, patch_features
```

## CLIP for Robotics

CLIP provides aligned vision-language features:

```python
import clip
import torch

class CLIPEncoder:
    def __init__(self):
        self.model, self.preprocess = clip.load("ViT-B/32")

    def encode_image(self, image):
        image = self.preprocess(image).unsqueeze(0)
        return self.model.encode_image(image)

    def encode_text(self, text):
        tokens = clip.tokenize([text])
        return self.model.encode_text(tokens)

    def similarity(self, image, text):
        img_feat = self.encode_image(image)
        txt_feat = self.encode_text(text)
        return torch.cosine_similarity(img_feat, txt_feat)
```

## Multi-View Perception

```python
class MultiViewEncoder:
    def __init__(self):
        self.encoder = RobotVisionEncoder()

    def encode_views(self, images):
        """Encode multiple camera views"""
        features = []
        for img in images:
            feat, _ = self.encoder.encode(img)
            features.append(feat)

        # Aggregate features
        aggregated = torch.stack(features).mean(dim=0)
        return aggregated
```

## Fine-Tuning for Robot Tasks

```python
from torch.utils.data import DataLoader
from transformers import ViTForImageClassification

# Fine-tune for object classification
model = ViTForImageClassification.from_pretrained(
    'google/vit-base-patch16-224',
    num_labels=10  # Number of object classes
)

# Training loop
optimizer = torch.optim.AdamW(model.parameters(), lr=1e-5)

for epoch in range(10):
    for batch in dataloader:
        outputs = model(batch['image'], labels=batch['label'])
        loss = outputs.loss
        loss.backward()
        optimizer.step()
        optimizer.zero_grad()
```

## Spatial Reasoning

```python
def extract_object_features(image, boxes, encoder):
    """Extract features for detected objects"""
    object_features = []

    for box in boxes:
        # Crop object region
        x1, y1, x2, y2 = box
        crop = image[:, y1:y2, x1:x2]

        # Resize and encode
        crop_resized = resize(crop, (224, 224))
        features = encoder.encode(crop_resized)
        object_features.append(features)

    return torch.stack(object_features)
```

## Hands-on Lab

### Lab 4.2: Build Object Grounding

Create a system that:
1. Encodes scene image with ViT
2. Encodes object descriptions with CLIP
3. Locates objects matching descriptions

## Summary

- ViT extracts rich visual features
- CLIP enables vision-language alignment
- Multi-view improves 3D understanding
- Fine-tuning adapts models to robot tasks

[Continue to Chapter 4.3 →](/docs/module-4-vla/chapter-3-language)


