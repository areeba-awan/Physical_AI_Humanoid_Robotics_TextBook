---
sidebar_position: 1
title: "3.1 NVIDIA Isaac کا تعارف"
description: NVIDIA Isaac کے ساتھ GPU سے تیز روبوٹکس
keywords: [NVIDIA, Isaac, GPU, روبوٹکس, AI]
---

# باب 3.1: NVIDIA Isaac کا تعارف

## سیکھنے کے مقاصد

- NVIDIA Isaac ایکوسسٹم کو سمجھیں
- Isaac Sim ماحول کی تنصیب کریں
- Isaac ROS اور Isaac SDK کے بارے میں جانیں
- GPU سے تیز روبوٹکس کی تحقیق کریں

## Isaac ایکوسسٹم

```
┌─────────────────────────────────────────────────────────────┐
│                    NVIDIA ISAAC ایکوسسٹم                    │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │                    ISAAC SIM                          │  │
│  │  • Omniverse پر مبنی سمولیشن                         │  │
│  │  • RTX رے ٹریسنگ                                     │  │
│  │  • PhysX فزکس                                        │  │
│  │  • ڈومین رینڈمائزیشن                                 │  │
│  └──────────────────────────────────────────────────────┘  │
│                           │                                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │                    ISAAC ROS                          │  │
│  │  • GPU سے تیز پرسیپشن                                │  │
│  │  • DNN انفرنس                                        │  │
│  │  • ویژول SLAM                                        │  │
│  │  • cuMotion پلاننگ                                   │  │
│  └──────────────────────────────────────────────────────┘  │
│                           │                                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │                    JETSON                             │  │
│  │  • ایج ڈیپلائمنٹ                                     │  │
│  │  • ریئل ٹائم انفرنس                                  │  │
│  │  • کم بجلی کی کھپت                                   │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

## سسٹم کی ضروریات

| جزو | کم از کم | تجویز کردہ |
|-----------|---------|-------------|
| GPU | RTX 2070 | RTX 3080+ |
| VRAM | 8 GB | 12+ GB |
| RAM | 32 GB | 64 GB |
| سٹوریج | 100 GB SSD | 500 GB NVMe |

## Isaac Sim کی تنصیب

### Omniverse Launcher کے ذریعے

1. [Omniverse Launcher](https://developer.nvidia.com/omniverse) ڈاؤن لوڈ کریں
2. Exchange سے Isaac Sim انسٹال کریں
3. لانچ کریں اور ROS 2 برج کنفیگر کریں

### Docker آپشن

```bash
# Isaac Sim کنٹینر پل کریں
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1

# GPU رسائی کے ساتھ چلائیں
docker run --gpus all -it \
  -v ~/isaac_workspace:/workspace \
  nvcr.io/nvidia/isaac-sim:2023.1.1
```

## Isaac Sim انٹرفیس

### اہم اجزاء

- **Stage** - USD اثاثوں کے ساتھ سین گراف
- **Viewport** - 3D ویژولائزیشن
- **Property Panel** - آبجیکٹ پراپرٹیز
- **Timeline** - اینیمیشن اور سمولیشن کنٹرول

### USD فارمیٹ

Universal Scene Description مندرجہ ذیل کو فعال کرتا ہے:
- بڑے پیمانے پر سین کمپوزیشن
- نان ڈیسٹرکٹیو ایڈیٹنگ
- تعاون ورک فلوز

## Isaac ROS پیکجز

```bash
# Isaac ROS انسٹال کریں
sudo apt install ros-humble-isaac-ros-*

# اہم پیکجز
ros-humble-isaac-ros-dnn-inference
ros-humble-isaac-ros-visual-slam
ros-humble-isaac-ros-apriltag
ros-humble-isaac-ros-yolov8
```

## پہلا Isaac Sim سین

```python
# Isaac Sim میں Python اسکرپٹ
from omni.isaac.core import World
from omni.isaac.wheeled_robots.robots import WheeledRobot

world = World()
robot = world.scene.add(
    WheeledRobot(
        prim_path="/World/Robot",
        name="my_robot",
        wheel_dof_names=["left_wheel", "right_wheel"]
    )
)
world.reset()
```

## عملی لیب

### لیب 3.1: Isaac Sim لانچ کریں

1. Omniverse کے ذریعے Isaac Sim انسٹال کریں
2. ایک نمونہ سین لوڈ کریں
3. اثاثہ لائبریری سے روبوٹ شامل کریں
4. سمولیشن چلائیں

## خلاصہ

- Isaac GPU سے تیز روبوٹکس ٹولز فراہم کرتا ہے
- Isaac Sim فوٹو ریئلسٹک سمولیشن پیش کرتا ہے
- Isaac ROS مؤثر پرسیپشن فعال کرتا ہے
- USD فارمیٹ قابل توسیع سین کمپوزیشن فعال کرتا ہے

[باب 3.2 پر جائیں ←](/docs/module-3-isaac/chapter-2-sim)
