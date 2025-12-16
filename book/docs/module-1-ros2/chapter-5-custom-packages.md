---
sidebar_position: 5
title: "1.5 کسٹم پیکیجز بنانا"
description: دوبارہ قابل استعمال آر او ایس 2 پیکیجز بنانا
keywords: [ROS 2, packages, colcon, ament]
---

# باب 1.5: کسٹم پیکیجز بنانا

## سیکھنے کے مقاصد

- پائتھون اور سی++ آر او ایس 2 پیکیجز بنائیں
- کسٹم پیغامات اور سروسز کی تعریف کریں
- پیکیج انحصارات کنفیگر کریں
- کولکون کے ساتھ پیکیجز بلڈ اور انسٹال کریں

## پائتھون پیکیج بنانا

```bash
cd ~/ros2_ws/src
ros2 pkg create my_robot_pkg --build-type ament_python --dependencies rclpy std_msgs
```

### پیکیج کا ڈھانچہ

```
my_robot_pkg/
├── my_robot_pkg/
│   ├── __init__.py
│   └── robot_node.py
├── resource/
│   └── my_robot_pkg
├── test/
├── package.xml
├── setup.py
└── setup.cfg
```

### setup.py

```python
from setuptools import setup

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'robot_node = my_robot_pkg.robot_node:main',
        ],
    },
)
```

## کسٹم پیغامات

### پیغام کی تعریف

```
# msg/RobotState.msg
string robot_name
float64 x
float64 y
float64 theta
float64 battery_level
bool is_moving
```

### CMakeLists.txt کو اپ ڈیٹ کریں

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  DEPENDENCIES std_msgs
)
```

### کسٹم پیغام استعمال کریں

```python
from my_robot_pkg.msg import RobotState

msg = RobotState()
msg.robot_name = "atlas"
msg.x = 1.0
msg.y = 2.0
```

## پیکیجز بلڈ کرنا

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_pkg
source install/setup.bash
```

## عملی لیب

### لیب 1.5: روبوٹ کنٹرولر پیکیج

ایک مکمل پیکیج بنائیں جس میں:
- روبوٹ کی حیثیت کے لیے کسٹم پیغام
- حیثیت کے لیے پبلشر نوڈ
- ویژولائزیشن کے لیے سبسکرائبر نوڈ
- سسٹم شروع کرنے کے لیے لانچ فائل

## خلاصہ

- فوری پیکیج سکیفولڈنگ کے لیے `ros2 pkg create` استعمال کریں
- کسٹم پیغامات آر او ایس 2 کمیونیکیشن کو بڑھاتے ہیں
- کولکون پیکیجز کو مؤثر طریقے سے بلڈ اور انسٹال کرتا ہے

[باب 1.6: لیب پر جائیں →](/docs/module-1-ros2/chapter-6-lab)
