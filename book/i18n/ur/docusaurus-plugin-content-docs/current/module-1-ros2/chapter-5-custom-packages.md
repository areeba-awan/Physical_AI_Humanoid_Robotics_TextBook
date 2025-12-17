---
sidebar_position: 5
title: "1.5 Building اپنی مرضی کے پیکجز"
description: "Creating reusable ROS 2 packages"
keywords: ["ROS 2", "packages", "colcon", "ament"]
---

# باب 1.5: Building اپنی مرضی کے پیکجز

## سیکھنے کے مقاصد

- تخلیق کریں Python and C++ ROS 2 packages
- Define custom messages and services
- Configure package dependencies
- تعمیر کریں and install packages with colcon

## Creating a Python Package

```bash
cd ~/ros2_ws/src
ros2 pkg create my_robot_pkg --build-type ament_python --dependencies rclpy std_msgs
```

### Package Structure

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

## Custom پیغامات

### Define Message

```
# msg/RobotState.msg
string robot_name
float64 x
float64 y
float64 theta
float64 battery_level
bool is_moving
```

### Update CMakeLists.txt

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  DEPENDENCIES std_msgs
)
```

### استعمال کریں Custom Message

```python
from my_robot_pkg.msg import RobotState

msg = RobotState()
msg.robot_name = "atlas"
msg.x = 1.0
msg.y = 2.0
```

## Building پیکجز

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_pkg
source install/setup.bash
```

## ہاتھ سے کام کرنے والی لیب

### Lab 1.5: Robot Controller Package

تخلیق کریں a complete package with:
- Custom message for robot status
- شائع کنندہ node for status
-  مسیحین node for visualization
- لانچ کریں file to start the system

## خلاصہ

- استعمال کریں `ros2 pkg create` for quick package scaffolding
- Custom messages extend ROS 2 communication
- colcon builds and installs packages efficiently

[Continue to باب 1.6: Lab →](/ur/docs/module-1-ros2/chapter-6-lab)


