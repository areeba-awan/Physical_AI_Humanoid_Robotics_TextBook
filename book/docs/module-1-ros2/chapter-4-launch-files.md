---
sidebar_position: 4
title: "1.4 لانچ فائلز اور کنفیگریشن"
description: لانچ فائلز کے ساتھ پیچیدہ روبوٹ سسٹمز کو منظم کرنا
keywords: [ROS 2, launch, configuration, orchestration]
---

# باب 1.4: لانچ فائلز اور کنفیگریشن

## سیکھنے کے مقاصد

- آر او ایس 2 کے لیے پائتھون لانچ فائلز بنائیں
- آرگیومنٹس کے ساتھ متعدد نوڈز کنفیگر کریں
- لانچ فائلز میں شرطی منطق استعمال کریں
- پیچیدہ سسٹمز کے لیے لانچ فائلز منظم کریں

## لانچ فائلز کا تعارف

لانچ فائلز آپ کو یہ کرنے دیتی ہیں:
- ایک کمانڈ سے متعدد نوڈز شروع کریں
- پیرامیٹرز اور ری میپنگز سیٹ کریں
- دوسری لانچ فائلز شامل کریں
- شرطی منطق استعمال کریں

## پائتھون لانچ فائلز

### بنیادی لانچ فائل

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

### پیرامیٹرز کے ساتھ

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

### آرگیومنٹس کے ساتھ

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='default_robot',
        description='روبوٹ کا نام'
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

### لانچ فائلز چلانا

```bash
# بنیادی لانچ
ros2 launch my_package robot.launch.py

# آرگیومنٹس کے ساتھ
ros2 launch my_package robot.launch.py robot_name:=atlas
```

## جدید خصوصیات

### دوسری لانچ فائلز شامل کرنا

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

### شرطی منطق

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

## عملی لیب

### لیب 1.4: ملٹی روبوٹ لانچ

ایک لانچ فائل بنائیں جو منفرد نیم سپیسز کے ساتھ 3 روبوٹس سپان کرے۔

## خلاصہ

- لانچ فائلز پیچیدہ ملٹی نوڈ سسٹمز کو منظم کرتی ہیں
- لچکدار کنفیگریشن کے لیے آرگیومنٹس استعمال کریں
- شرطی منطق دوبارہ قابل استعمال لانچ فائلز فعال کرتی ہے

[باب 1.5 پر جائیں →](/docs/module-1-ros2/chapter-5-custom-packages)
