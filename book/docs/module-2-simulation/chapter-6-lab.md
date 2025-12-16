---
sidebar_position: 6
title: "2.6 لیب: ڈیجیٹل ٹوئن پائپ لائن"
description: مکمل ڈیجیٹل ٹوئن سسٹم بنانا
keywords: [ڈیجیٹل ٹوئن, سمولیشن, گیزیبو, یونٹی, پائپ لائن]
---

# باب 2.6: لیب - ڈیجیٹل ٹوئن پائپ لائن

## لیب کا جائزہ

ایک مکمل ڈیجیٹل ٹوئن بنائیں جو سمولیشن اور ویژولائزیشن کے درمیان ہم آہنگی رکھے، جس سے حقیقت پسندانہ ورچوئل ماحول میں روبوٹ رویوں کی جانچ ممکن ہو۔

## پروجیکٹ: گودام روبوٹ ڈیجیٹل ٹوئن

### سسٹم آرکیٹیکچر

```
┌─────────────────────────────────────────────────────────────┐
│                    ڈیجیٹل ٹوئن پائپ لائن                     │
│                                                              │
│  ┌──────────────┐              ┌──────────────────────┐    │
│  │   گیزیبو      │    /tf      │      RViz2           │    │
│  │  سمولیشن     │────────────>│   ویژولائزیشن        │    │
│  │              │   /scan     │                      │    │
│  │  - فزکس     │────────────>│   - روبوٹ ماڈل       │    │
│  │  - سینسرز   │  /camera    │   - سینسر ڈیٹا       │    │
│  │              │────────────>│   - پاتھ پلاننگ      │    │
│  └──────────────┘              └──────────────────────┘    │
│         │                               ▲                   │
│         │ /odom                         │                   │
│         ▼                               │                   │
│  ┌──────────────┐              ┌────────┴─────────────┐    │
│  │   Nav2       │    /cmd_vel │     آپریٹر           │    │
│  │  نیویگیشن   │<────────────│     انٹرفیس          │    │
│  │              │             │                      │    │
│  │  - SLAM      │             │   - گول سیٹنگ       │    │
│  │  - پلانر    │             │   - مانیٹرنگ         │    │
│  └──────────────┘             └──────────────────────┘    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## مرحلہ 1: گودام کی دنیا بنائیں

```xml
<!-- worlds/warehouse.sdf -->
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="warehouse">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <!-- دیواریں -->
    <model name="wall_north">
      <static>true</static>
      <pose>0 5 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>10 0.2 2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>10 0.2 2</size></box></geometry>
        </visual>
      </link>
    </model>

    <!-- شیلفیں -->
    <model name="shelf_1">
      <static>true</static>
      <pose>-2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.5 2 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.5 2 1</size></box></geometry>
          <material>
            <ambient>0.5 0.3 0.1 1</ambient>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## مرحلہ 2: روبوٹ URDF

```xml
<!-- urdf/warehouse_robot.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="warehouse_robot">
  <!-- بنیادی فائلز شامل کریں -->
  <xacro:include filename="base.xacro"/>
  <xacro:include filename="sensors.xacro"/>

  <!-- روبوٹ کے اجزاء -->
  <xacro:base/>
  <xacro:lidar parent="base_link"/>
  <xacro:camera parent="base_link"/>
  <xacro:imu parent="base_link"/>
</robot>
```

## مرحلہ 3: لانچ فائل

```python
# launch/digital_twin.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # گیزیبو شروع کریں
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('gazebo_ros'),
                '/launch/gazebo.launch.py'
            ]),
            launch_arguments={'world': 'warehouse.sdf'}.items()
        ),

        # روبوٹ اسپان کریں
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'robot', '-file', 'warehouse_robot.urdf']
        ),

        # RViz شروع کریں
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', 'config.rviz']
        ),

        # Nav2 نیویگیشن شروع کریں
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('nav2_bringup'),
                '/launch/navigation_launch.py'
            ])
        ),
    ])
```

## مرحلہ 4: ڈیجیٹل ٹوئن چلائیں

```bash
# مکمل سسٹم شروع کریں
ros2 launch warehouse_robot digital_twin.launch.py

# نیویگیشن گول بھیجیں
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {pose: {position: {x: 2.0, y: 2.0}}}}"
```

## تصدیق کی چیک لسٹ

- [ ] روبوٹ گودام میں ظاہر ہوتا ہے
- [ ] سینسرز ڈیٹا شائع کرتے ہیں
- [ ] RViz روبوٹ اور سینسر ڈیٹا دکھاتا ہے
- [ ] نیویگیشن گولز کام کرتے ہیں
- [ ] رکاوٹوں سے بچاؤ کام کرتا ہے

## چیلنج ایکسٹینشنز

1. **متحرک رکاوٹیں شامل کریں** - چلتے ہوئے لوگ یا فورک لفٹس
2. **پک اینڈ پلیس نافذ کریں** - ہیرا پھیری کے لیے MoveIt استعمال کریں
3. **ملٹی روبوٹ کوآرڈینیشن** - فلیٹ مینجمنٹ

## لیب مکمل!

آپ نے ایک مکمل ڈیجیٹل ٹوئن پائپ لائن بنا لی ہے۔ اگلے ماڈیول میں، ہم NVIDIA Isaac کے ساتھ AI صلاحیتیں شامل کریں گے۔

[ماڈیول 3: NVIDIA Isaac جاری رکھیں ←](/docs/module-3-isaac/chapter-1-intro)
