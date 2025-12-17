---
sidebar_position: 5
title: "1.5 Building Custom Packages"
description: Creating reusable ROS 2 packages
keywords: [ROS 2, packages, colcon, ament]
---

# Chapter 1.5: Building Custom Packages

## Learning Objectives

- Create Python and C++ ROS 2 packages
- Define custom messages and services
- Configure package dependencies
- Build and install packages with colcon

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

## Custom Messages

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

### Use Custom Message

```python
from my_robot_pkg.msg import RobotState

msg = RobotState()
msg.robot_name = "atlas"
msg.x = 1.0
msg.y = 2.0
```

## Building Packages

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_pkg
source install/setup.bash
```

## Hands-on Lab

### Lab 1.5: Robot Controller Package

Create a complete package with:
- Custom message for robot status
- Publisher node for status
- Subscriber node for visualization
- Launch file to start the system

## Summary

- Use `ros2 pkg create` for quick package scaffolding
- Custom messages extend ROS 2 communication
- colcon builds and installs packages efficiently

[Continue to Chapter 1.6: Lab →](/docs/module-1-ros2/chapter-6-lab)


