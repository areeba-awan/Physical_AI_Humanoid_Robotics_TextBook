---
sidebar_position: 1
title: "4.1 VLA کا تعارف"
description: روبوٹکس کے لیے ویژن-لینگویج-ایکشن ماڈلز
keywords: [VLA, ویژن, زبان, ایکشن, ملٹی موڈل, AI]
---

# باب 4.1: VLA کا تعارف

## سیکھنے کے مقاصد

- ویژن-لینگویج-ایکشن (VLA) ماڈلز کو سمجھیں
- VLA سسٹمز کے اجزاء سیکھیں
- جدید ترین VLA آرکیٹیکچرز کی تلاش کریں
- VLA ڈیولپمنٹ انوائرنمنٹ سیٹ اپ کریں

## VLA کیا ہے؟

**ویژن-لینگویج-ایکشن (VLA)** ماڈلز تین موڈیلیٹیز کو یکجا کرتے ہیں:

```
┌─────────────────────────────────────────────────────────────┐
│                    VLA آرکیٹیکچر                            │
│                                                              │
│   ┌─────────┐     ┌─────────────────────┐     ┌─────────┐  │
│   │ کیمرہ   │────>│                     │────>│ روبوٹ   │  │
│   │ (ویژن) │     │   ملٹی موڈل         │     │ (ایکشن) │  │
│   └─────────┘     │   ٹرانسفارمر        │     └─────────┘  │
│                   │                     │                   │
│   ┌─────────┐     │   - ویژن انکوڈ کریں │                   │
│   │ قدرتی   │────>│   - متن سمجھیں     │                   │
│   │ زبان    │     │   - ایکشنز بنائیں  │                   │
│   └─────────┘     └─────────────────────┘                   │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### اہم صلاحیتیں

| صلاحیت | مثال |
|--------|------|
| گراؤنڈنگ | "سرخ سیب اٹھاؤ" → سرخ سیب تلاش کریں |
| استدلال | "کپ پلیٹ کے قریب رکھو" → مقامی استدلال |
| منصوبہ بندی | "سینڈوچ بناؤ" → ذیلی کاموں میں تقسیم |
| عمل درآمد | موٹر کمانڈز تیار کریں |

## VLA ماڈل لینڈسکیپ

### RT-2 (روبوٹکس ٹرانسفارمر 2)

```
┌────────────────┐
│ ویژن انکوڈر    │──┐
│    (ViT)       │  │    ┌─────────────┐    ┌──────────┐
└────────────────┘  ├───>│   PaLM-E    │───>│  ایکشنز  │
┌────────────────┐  │    │  (54B)      │    │ (ٹوکنز)  │
│ زبان ان پٹ    │──┘    └─────────────┘    └──────────┘
└────────────────┘
```

### OpenVLA

- اوپن سورس VLA ماڈل
- 7B پیرامیٹرز
- اپنی مرضی کے کاموں پر فائن ٹیون کیا جا سکتا ہے

### دیگر ماڈلز

- **PaLM-E**: 562B ملٹی موڈل ماڈل
- **Gato**: جنرلسٹ ایجنٹ
- **RT-X**: کراس-امباڈیمنٹ ٹرانسفر

## VLA کے اجزاء

### 1. ویژن انکوڈر

```python
from transformers import ViTModel

# پہلے سے تربیت یافتہ ویژن انکوڈر
vision_encoder = ViTModel.from_pretrained('google/vit-base-patch16-224')

# تصویر پراسیس کریں
vision_features = vision_encoder(images).last_hidden_state
```

### 2. زبان انکوڈر

```python
from transformers import T5EncoderModel

# زبان انکوڈر
language_encoder = T5EncoderModel.from_pretrained('t5-base')

# ہدایات پراسیس کریں
instruction = "Pick up the red block"
language_features = language_encoder(tokenized_instruction)
```

### 3. ایکشن ڈیکوڈر

```python
# ایکشن ٹوکنز کی لغت
action_vocab = {
    0: 'move_to',      # منتقل ہوں
    1: 'grasp',        # پکڑیں
    2: 'release',      # چھوڑیں
    3: 'rotate',       # گھمائیں
    # ... 256 ایکشن ٹوکنز
}

# ایکشن ٹوکنز کو روبوٹ کمانڈز میں ڈیکوڈ کریں
def tokens_to_actions(tokens):
    actions = []
    for token in tokens:
        action = action_vocab[token]
        actions.append(action)
    return actions
```

## انوائرنمنٹ سیٹ اپ

```bash
# VLA ڈیپنڈنسیز انسٹال کریں
pip install torch transformers accelerate
pip install open-vla  # اگر OpenVLA استعمال کر رہے ہیں

# OpenVLA کلون کریں
git clone https://github.com/openvla/openvla.git
cd openvla
pip install -e .
```

## فوری مثال

```python
from openvla import OpenVLA

# ماڈل لوڈ کریں
model = OpenVLA.from_pretrained("openvla/openvla-7b")

# انفرنس
image = load_image("scene.jpg")
instruction = "Pick up the blue cup"

actions = model.predict(image, instruction)
# آؤٹ پٹ: [move_to(0.3, 0.5), grasp(), lift(), ...]
```

## عملی لیب

### لیب 4.1: VLA ماڈل چلائیں

1. OpenVLA انسٹال کریں
2. پہلے سے تربیت یافتہ ماڈل لوڈ کریں
3. نمونہ تصاویر پر انفرنس چلائیں
4. پیش گوئی شدہ ایکشنز کو ویژولائز کریں

## خلاصہ

- VLA ویژن، زبان، اور ایکشن کو یکجا کرتا ہے
- RT-2 اور OpenVLA جیسے ماڈلز ہدایات کی پیروی کو ممکن بناتے ہیں
- VLA روبوٹ کاموں کے لیے دستی پروگرامنگ کم کرتا ہے

[باب 4.2 پر جائیں →](/docs/module-4-vla/chapter-2-vision)
