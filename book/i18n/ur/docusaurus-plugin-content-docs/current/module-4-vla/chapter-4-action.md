---
sidebar_position: 4
title: "4.4 Action Generation"
description: "Generating robot actions from multimodal inputs"
keywords: ["action", "generation", "policy", "robot control"]
---

# باب 4.4: Action Generation

## سیکھنے کے مقاصد

- سمجھیں action representation schemes
- Implement action decoders
- Bridge discrete tokens to continuous control
- Handle action space variations

## Action Representations

### Discrete Action Tokens

```python
# Action vocabulary
ACTION_TOKENS = {
    # Movement
    0: {'type': 'move', 'dx': -0.01, 'dy': 0, 'dz': 0},
    1: {'type': 'move', 'dx': 0.01, 'dy': 0, 'dz': 0},
    2: {'type': 'move', 'dx': 0, 'dy': -0.01, 'dz': 0},
    # ... 256 total tokens covering position, rotation, gripper

    # Gripper
    254: {'type': 'gripper', 'state': 'open'},
    255: {'type': 'gripper', 'state': 'close'},
}
```

### Continuous ایکشنز

```python
# 7-DOF action space
class ContinuousAction:
    def __init__(self):
        self.delta_position = np.zeros(3)  # dx, dy, dz
        self.delta_rotation = np.zeros(3)  # roll, pitch, yaw
        self.gripper = 0.0  # 0=open, 1=closed
```

## Action Decoder

```python
import torch
import torch.nn as nn

class ActionDecoder(nn.مودیول):
    def __init__(self, hidden_dim=768, action_dim=7):
        super().__init__()

        self.mlp = nn.Sequential(
            nn.Linear(hidden_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, action_dim),
            nn.Tanh()  # Bound actions to [-1, 1]
        )

        self.action_scale = nn.Parameter(torch.ones(action_dim))

    def forward(self, features):
        raw_actions = self.mlp(features)
        scaled_actions = raw_actions * self.action_scale
        return scaled_actions
```

## Token to Action Conversion

```python
def tokens_to_continuous(tokens, tokenizer):
    """Convert discrete tokens to continuous actions"""
    actions = []

    for token in tokens:
        if token < 128:  # Position tokens
            # Decode position delta
            position = tokenizer.decode_position(token)
            actions.append({'position': position})

        elif token < 192:  # Rotation tokens
            rotation = tokenizer.decode_rotation(token - 128)
            actions.append({'rotation': rotation})

        else:  # Gripper tokens
            gripper = tokenizer.decode_gripper(token - 192)
            actions.append({'gripper': gripper})

    return merge_actions(actions)
```

## Action Chunking

```python
class ActionChunker:
    """Generate multiple future actions at once"""

    def __init__(self, chunk_size=10):
        self.chunk_size = chunk_size

    def predict_chunk(self, model, observation):
        """Predict a sequence of future actions"""
        actions = []

        features = model.encode(observation)

        for i in range(self.chunk_size):
            action = model.decode_action(features)
            actions.append(action)

            # Update features with predicted action
            features = model.update_features(features, action)

        return actions
```

## Diffusion Policy

```python
class DiffusionPolicy:
    """Generate actions via denoising diffusion"""

    def __init__(self, action_dim=7, num_steps=100):
        self.action_dim = action_dim
        self.num_steps = num_steps
        self.denoiser = Denoiser(action_dim)

    def sample(self, condition):
        """Sample action via reverse diffusion"""
        # Start with noise
        x = torch.randn(self.action_dim)

        for t in reversed(range(self.num_steps)):
            # Predict noise
            noise_pred = self.denoiser(x, t, condition)

            # Denoise
            x = self.denoise_step(x, noise_pred, t)

        return x
```

## ہاتھ سے کام کرنے والی لیب

### Lab 4.4: Implement Action Decoder

تعمیر کریں an action decoder that:
1. Takes vision-language features
2. Outputs 7-DOF robot actions
3. Handles action chunking for smoother motion

## خلاصہ

- ایکشنز can be discrete tokens or continuous values
- Decoders map multimodal features to robot commands
- Action chunking predicts future action sequences
- Diffusion policies enable diverse action generation

[Continue to باب 4.5 →](/docs/module-4-vla/chapter-5-systems)


