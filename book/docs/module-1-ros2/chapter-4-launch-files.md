---
sidebar_position: 4
title: "1.4 Launch Files and Configuration"
description: Orchestrating complex robot systems with launch files
keywords: [ROS 2, launch, configuration, orchestration]
---

# Chapter 1.4: Launch Files and Configuration

## Learning Objectives

- Create Python launch files for ROS 2
- Configure multiple nodes with arguments
- Use conditional logic in launch files
- Organize launch files for complex systems

## Introduction to Launch Files

Launch files allow you to:
- Start multiple nodes with one command
- Set parameters and remappings
- Include other launch files
- Use conditional logic

## Python Launch Files

### Basic Launch File

```python
# launch/robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='sensor_node',
            name='lidar_sensor',
            output='screen',
        ),
        Node(
            package='my_package',
            executable='controller_node',
            name='robot_controller',
            output='screen',
        ),
    ])
```

### With Parameters

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='robot_node',
            name='robot',
            parameters=[{
                'robot_name': 'atlas',
                'max_speed': 2.0,
                'use_sim': True,
            }],
        ),
    ])
```

### With Arguments

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='default_robot',
        description='Name of the robot'
    )

    return LaunchDescription([
        robot_name_arg,
        Node(
            package='my_package',
            executable='robot_node',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name'),
            }],
        ),
    ])
```

### Running Launch Files

```bash
# Basic launch
ros2 launch my_package robot.launch.py

# With arguments
ros2 launch my_package robot.launch.py robot_name:=atlas
```

## Advanced Features

### Including Other Launch Files

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ])
    )

    return LaunchDescription([gazebo_launch])
```

### Conditional Logic

```python
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_rviz = DeclareLaunchArgument('use_rviz', default_value='true')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([use_rviz, rviz_node])
```

## Hands-on Lab

### Lab 1.4: Multi-Robot Launch

Create a launch file that spawns 3 robots with unique namespaces.

## Summary

- Launch files orchestrate complex multi-node systems
- Use arguments for flexible configuration
- Conditional logic enables reusable launch files

[Continue to Chapter 1.5 →](/docs/module-1-ros2/chapter-5-custom-packages)


