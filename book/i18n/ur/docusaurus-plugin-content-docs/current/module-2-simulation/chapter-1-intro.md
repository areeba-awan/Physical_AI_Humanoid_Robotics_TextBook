---
sidebar_position: 1
title: "2.1 سیمیولیشن کا تعارف"
description: "روبوٹکس کی ترقی کے لیے سیمیولیشن کیوں ضروری ہے"
keywords: ["simulation", "digital twin", "گیزیبو", "یونٹی", "robotics"]
---

# باب 2.1: سیمیولیشن کا تعارف

## سیکھنے کے مقاصد

- سمجھیں the role of simulation in robotics development
-  موازنہ کریں different simulation platforms
- سیکھیں the digital twin concept
-  تیار کریں your simulation environment

## Why Simulate?

سیمیولیشن is essential for modern robotics because:

1. **Safety** - جانچ dangerous scenarios without risk
2. **Speed** - چلائیں experiments faster than real-time
3. **Cost** - No hardware wear or damage
4. **Scale** - جانچ thousands of scenarios
5. **Reproducibility** - Exact same conditions every time

### ڈیجیٹل ٹوئن  تصور

```
┌──────────────────────┐         ┌──────────────────────┐
│     REAL WORLD       │         │    DIGITAL TWIN      │
│                      │         │                      │
│  ┌──────────────┐   │  Sync   │   ┌──────────────┐   │
│  │ Physical     │◄──┼─────────┼──►│ Simulated    │   │
│  │ Robot        │   │         │   │ Robot        │   │
│  └──────────────┘   │         │   └──────────────┘   │
│                      │         │                      │
│  ┌──────────────┐   │         │   ┌──────────────┐   │
│  │ ماحول  │◄──┼─────────┼──►│ Virtual      │   │
│  │              │   │         │   │ ماحول  │   │
│  └──────────────┘   │         │   └──────────────┘   │
└──────────────────────┘         └──────────────────────┘
```

## سیمیولیشن پلیٹ فارم

|  پلیٹ فارم | Strengths | استعمال کریں Cases |
|----------|-----------|-----------|
| **گیزیبو** | ROS integration, physics | General robotics |
| **یونٹی** | Graphics, ML-Agents | Vision, perception |
| **ایزیک سیم** | GPU physics, synthetic data | AI training |
| **Webots** | Easy to use, tutorials | Education |

### گیزیبو (Open ماخذ)

```bash
# انسٹال کریں گیزیبو with ROS 2
sudo apt install ros-humble-gazebo-ros-pkgs

# لانچ کریں empty world
gazebo
```

### یونٹی روبوٹکس ہب

- فوٹو ریئلیسٹک rendering
- ML-Agents integration
- Cross-platform export

### این ویڈیا ایزیک Sim

- RTX ray tracing
- Domain randomization
- Synthetic data generation

## گیزیبو تیار کرنا

### Basic دنیا فائل

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="my_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
  </world>
</sdf>
```

### لانچ کریں گیزیبو with ROS 2

```python
# launch/gazebo.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={'world': 'my_world.sdf'}.items()
    )
    return LaunchDescription([gazebo])
```

## سیمیولیشن وفاداری

### طبیعات کی درستی

| Level | Speed | Accuracy | استعمال کریں Case |
|-------|-------|----------|----------|
| Low | 100x RT | ~50% | Quick tests |
| Medium | 10x RT | ~80% |  ترقی |
| High | 1x RT | ~95% |  توثیق |

## ہاتھ سے کام کرنے والی لیب

### Lab 2.1: لانچ کریں آپ کا پہلا سیمیولیشن

1. انسٹال کریں گیزیبو
2. لانچ کریں the empty world
3. شامل کریں a simple shape (box, sphere)
4. اپلائی کریں forces and observe physics

## خلاصہ

- سیمیولیشن enables safe, fast, and cheap robot development
- Digital twins mirror physical robots in virtual environments
- منتخب کریں your platform based on your needs

[Continue to باب 2.2: گیزیبو Basics →](/docs/module-2-simulation/chapter-2-gazebo)


