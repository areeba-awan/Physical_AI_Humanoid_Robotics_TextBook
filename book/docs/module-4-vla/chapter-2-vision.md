---
sidebar_position: 2
title: "4.2 روبوٹس کے لیے ویژن ٹرانسفارمرز"
description: ٹرانسفارمرز کے ساتھ بصری ادراک
keywords: [ViT, ویژن ٹرانسفارمر, ادراک, فیچرز]
---

# باب 4.2: روبوٹس کے لیے ویژن ٹرانسفارمرز

## سیکھنے کے مقاصد

- ویژن ٹرانسفارمر (ViT) آرکیٹیکچر کو سمجھیں
- روبوٹکس کے لیے بصری فیچرز نکالیں
- روبوٹ کاموں کے لیے ویژن ماڈلز فائن ٹیون کریں
- ملٹی ویو پرسیپشن لاگو کریں

## ویژن ٹرانسفارمر آرکیٹیکچر

```
┌─────────────────────────────────────────────────────────────┐
│                  ویژن ٹرانسفارمر                            │
│                                                              │
│    ┌─────────────────────────────────────────────────────┐  │
│    │              ان پٹ تصویر (224x224)                   │  │
│    └─────────────────────────────────────────────────────┘  │
│                          │                                   │
│                    پیچ ایمبیڈنگ                              │
│                          │                                   │
│    ┌─────────────────────────────────────────────────────┐  │
│    │  [CLS] P1  P2  P3  P4  ...  P196  +  پوزیشن ایمب   │  │
│    └─────────────────────────────────────────────────────┘  │
│                          │                                   │
│              ٹرانسفارمر انکوڈر (x12)                        │
│                          │                                   │
│    ┌─────────────────────────────────────────────────────┐  │
│    │              بصری فیچرز (768-dim)                    │  │
│    └─────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

## ViT کا نفاذ

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

        # عالمی فیچر (CLS ٹوکن)
        global_features = outputs.last_hidden_state[:, 0]

        # مقامی استدلال کے لیے پیچ فیچرز
        patch_features = outputs.last_hidden_state[:, 1:]

        return global_features, patch_features
```

## روبوٹکس کے لیے CLIP

CLIP ہم آہنگ ویژن-لینگویج فیچرز فراہم کرتا ہے:

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

## ملٹی ویو پرسیپشن

```python
class MultiViewEncoder:
    def __init__(self):
        self.encoder = RobotVisionEncoder()

    def encode_views(self, images):
        """متعدد کیمرہ ویوز انکوڈ کریں"""
        features = []
        for img in images:
            feat, _ = self.encoder.encode(img)
            features.append(feat)

        # فیچرز کو یکجا کریں
        aggregated = torch.stack(features).mean(dim=0)
        return aggregated
```

## روبوٹ کاموں کے لیے فائن ٹیوننگ

```python
from torch.utils.data import DataLoader
from transformers import ViTForImageClassification

# آبجیکٹ کلاسیفیکیشن کے لیے فائن ٹیون کریں
model = ViTForImageClassification.from_pretrained(
    'google/vit-base-patch16-224',
    num_labels=10  # آبجیکٹ کلاسز کی تعداد
)

# تربیتی لوپ
optimizer = torch.optim.AdamW(model.parameters(), lr=1e-5)

for epoch in range(10):
    for batch in dataloader:
        outputs = model(batch['image'], labels=batch['label'])
        loss = outputs.loss
        loss.backward()
        optimizer.step()
        optimizer.zero_grad()
```

## مقامی استدلال

```python
def extract_object_features(image, boxes, encoder):
    """پتہ لگائے گئے آبجیکٹس کے لیے فیچرز نکالیں"""
    object_features = []

    for box in boxes:
        # آبجیکٹ ریجن کاٹیں
        x1, y1, x2, y2 = box
        crop = image[:, y1:y2, x1:x2]

        # سائز تبدیل کریں اور انکوڈ کریں
        crop_resized = resize(crop, (224, 224))
        features = encoder.encode(crop_resized)
        object_features.append(features)

    return torch.stack(object_features)
```

## عملی لیب

### لیب 4.2: آبجیکٹ گراؤنڈنگ بنائیں

ایک سسٹم بنائیں جو:
1. ViT کے ساتھ سین کی تصویر انکوڈ کرے
2. CLIP کے ساتھ آبجیکٹ کی تفصیلات انکوڈ کرے
3. تفصیلات سے ملتے آبجیکٹس تلاش کرے

## خلاصہ

- ViT بھرپور بصری فیچرز نکالتا ہے
- CLIP ویژن-لینگویج ہم آہنگی کو ممکن بناتا ہے
- ملٹی ویو 3D سمجھ بوجھ بہتر بناتا ہے
- فائن ٹیوننگ ماڈلز کو روبوٹ کاموں کے مطابق ڈھالتی ہے

[باب 4.3 پر جائیں →](/docs/module-4-vla/chapter-3-language)
