---
sidebar_position: 4
title: "4.4 ایکشن جنریشن"
description: ملٹی موڈل ان پٹس سے روبوٹ ایکشنز بنانا
keywords: [ایکشن, جنریشن, پالیسی, روبوٹ کنٹرول]
---

# باب 4.4: ایکشن جنریشن

## سیکھنے کے مقاصد

- ایکشن ریپریزنٹیشن سکیمز کو سمجھیں
- ایکشن ڈیکوڈرز لاگو کریں
- ڈسکریٹ ٹوکنز کو مسلسل کنٹرول سے جوڑیں
- ایکشن اسپیس کی تبدیلیوں کو سنبھالیں

## ایکشن ریپریزنٹیشنز

### ڈسکریٹ ایکشن ٹوکنز

```python
# ایکشن لغت
ACTION_TOKENS = {
    # حرکت
    0: {'type': 'move', 'dx': -0.01, 'dy': 0, 'dz': 0},
    1: {'type': 'move', 'dx': 0.01, 'dy': 0, 'dz': 0},
    2: {'type': 'move', 'dx': 0, 'dy': -0.01, 'dz': 0},
    # ... 256 ٹوکنز پوزیشن، روٹیشن، گرپر کے لیے

    # گرپر
    254: {'type': 'gripper', 'state': 'open'},     # کھولیں
    255: {'type': 'gripper', 'state': 'close'},    # بند کریں
}
```

### مسلسل ایکشنز

```python
# 7-DOF ایکشن اسپیس
class ContinuousAction:
    def __init__(self):
        self.delta_position = np.zeros(3)  # dx, dy, dz
        self.delta_rotation = np.zeros(3)  # رول، پچ، یا
        self.gripper = 0.0  # 0=کھلا، 1=بند
```

## ایکشن ڈیکوڈر

```python
import torch
import torch.nn as nn

class ActionDecoder(nn.Module):
    def __init__(self, hidden_dim=768, action_dim=7):
        super().__init__()

        self.mlp = nn.Sequential(
            nn.Linear(hidden_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, action_dim),
            nn.Tanh()  # ایکشنز کو [-1, 1] میں محدود کریں
        )

        self.action_scale = nn.Parameter(torch.ones(action_dim))

    def forward(self, features):
        raw_actions = self.mlp(features)
        scaled_actions = raw_actions * self.action_scale
        return scaled_actions
```

## ٹوکن سے ایکشن میں تبدیلی

```python
def tokens_to_continuous(tokens, tokenizer):
    """ڈسکریٹ ٹوکنز کو مسلسل ایکشنز میں تبدیل کریں"""
    actions = []

    for token in tokens:
        if token < 128:  # پوزیشن ٹوکنز
            # پوزیشن ڈیلٹا ڈیکوڈ کریں
            position = tokenizer.decode_position(token)
            actions.append({'position': position})

        elif token < 192:  # روٹیشن ٹوکنز
            rotation = tokenizer.decode_rotation(token - 128)
            actions.append({'rotation': rotation})

        else:  # گرپر ٹوکنز
            gripper = tokenizer.decode_gripper(token - 192)
            actions.append({'gripper': gripper})

    return merge_actions(actions)
```

## ایکشن چنکنگ

```python
class ActionChunker:
    """ایک ساتھ متعدد مستقبل کے ایکشنز بنائیں"""

    def __init__(self, chunk_size=10):
        self.chunk_size = chunk_size

    def predict_chunk(self, model, observation):
        """مستقبل کے ایکشنز کی ترتیب پیش گوئی کریں"""
        actions = []

        features = model.encode(observation)

        for i in range(self.chunk_size):
            action = model.decode_action(features)
            actions.append(action)

            # پیش گوئی شدہ ایکشن کے ساتھ فیچرز اپڈیٹ کریں
            features = model.update_features(features, action)

        return actions
```

## ڈفیوژن پالیسی

```python
class DiffusionPolicy:
    """ڈی نوائزنگ ڈفیوژن کے ذریعے ایکشنز بنائیں"""

    def __init__(self, action_dim=7, num_steps=100):
        self.action_dim = action_dim
        self.num_steps = num_steps
        self.denoiser = Denoiser(action_dim)

    def sample(self, condition):
        """ریورس ڈفیوژن کے ذریعے ایکشن حاصل کریں"""
        # شور سے شروع کریں
        x = torch.randn(self.action_dim)

        for t in reversed(range(self.num_steps)):
            # شور کی پیش گوئی کریں
            noise_pred = self.denoiser(x, t, condition)

            # شور ہٹائیں
            x = self.denoise_step(x, noise_pred, t)

        return x
```

## عملی لیب

### لیب 4.4: ایکشن ڈیکوڈر لاگو کریں

ایک ایکشن ڈیکوڈر بنائیں جو:
1. ویژن-لینگویج فیچرز لے
2. 7-DOF روبوٹ ایکشنز آؤٹ پٹ کرے
3. ہموار موشن کے لیے ایکشن چنکنگ سنبھالے

## خلاصہ

- ایکشنز ڈسکریٹ ٹوکنز یا مسلسل ویلیوز ہو سکتے ہیں
- ڈیکوڈرز ملٹی موڈل فیچرز کو روبوٹ کمانڈز میں میپ کرتے ہیں
- ایکشن چنکنگ مستقبل کی ایکشن سیکوینسز پیش گوئی کرتی ہے
- ڈفیوژن پالیسیز متنوع ایکشن جنریشن ممکن بناتی ہیں

[باب 4.5 پر جائیں →](/docs/module-4-vla/chapter-5-systems)
