---
sidebar_position: 6
title: "2.6 Lab: Digital Twin Pipeline"
description: Building a complete digital twin system
keywords: [digital twin, simulation, Gazebo, Unity, pipeline]
---

# Chapter 2.6: Lab - Digital Twin Pipeline

## Lab Overview

Build a complete digital twin that synchronizes between simulation and visualization, enabling testing of robot behaviors in a realistic virtual environment.

## Project: Warehouse Robot Digital Twin

### System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    DIGITAL TWIN PIPELINE                     │
│                                                              │
│  ┌──────────────┐              ┌──────────────────────┐    │
│  │   Gazebo     │    /tf      │      RViz2           │    │
│  │  Simulation  │────────────>│   Visualization      │    │
│  │              │   /scan     │                      │    │
│  │  - Physics   │────────────>│   - Robot model      │    │
│  │  - Sensors   │  /camera    │   - Sensor data      │    │
│  │              │────────────>│   - Path planning    │    │
│  └──────────────┘              └──────────────────────┘    │
│         │                               ▲                   │
│         │ /odom                         │                   │
│         ▼                               │                   │
│  ┌──────────────┐              ┌────────┴─────────────┐    │
│  │   Nav2       │    /cmd_vel │     Operator         │    │
│  │  Navigation  │<────────────│     Interface        │    │
│  │              │             │                      │    │
│  │  - SLAM      │             │   - Goal setting     │    │
│  │  - Planner   │             │   - Monitoring       │    │
│  └──────────────┘             └──────────────────────┘    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Step 1: Create Warehouse World

```xml
<!-- worlds/warehouse.sdf -->
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="warehouse">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <!-- Walls -->
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

    <!-- Shelves -->
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

## Step 2: Robot URDF

```xml
<!-- urdf/warehouse_robot.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="warehouse_robot">
  <xacro:include filename="base.xacro"/>
  <xacro:include filename="sensors.xacro"/>

  <xacro:base/>
  <xacro:lidar parent="base_link"/>
  <xacro:camera parent="base_link"/>
  <xacro:imu parent="base_link"/>
</robot>
```

## Step 3: Launch File

```python
# launch/digital_twin.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('gazebo_ros'),
                '/launch/gazebo.launch.py'
            ]),
            launch_arguments={'world': 'warehouse.sdf'}.items()
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'robot', '-file', 'warehouse_robot.urdf']
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', 'config.rviz']
        ),

        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('nav2_bringup'),
                '/launch/navigation_launch.py'
            ])
        ),
    ])
```

## Step 4: Run the Digital Twin

```bash
# Launch the complete system
ros2 launch warehouse_robot digital_twin.launch.py

# Send navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {pose: {position: {x: 2.0, y: 2.0}}}}"
```

## Verification Checklist

- [ ] Robot spawns in warehouse
- [ ] Sensors publish data
- [ ] RViz displays robot and sensor data
- [ ] Navigation goals work
- [ ] Obstacle avoidance functions

## Challenge Extensions

1. **Add dynamic obstacles** - Moving people or forklifts
2. **Implement pick-and-place** - Use MoveIt for manipulation
3. **Multi-robot coordination** - Fleet management

## Lab Complete!

You've built a complete digital twin pipeline. Next, we'll add AI capabilities with NVIDIA Isaac.

[Continue to Module 3: NVIDIA Isaac →](/docs/module-3-isaac/chapter-1-intro)


