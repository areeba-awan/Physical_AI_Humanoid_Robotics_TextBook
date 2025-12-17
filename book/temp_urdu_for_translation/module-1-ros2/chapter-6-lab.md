---
sidebar_position: 6
title: "1.6 Lab: Building a ROS 2 Robot"
description: Complete hands-on lab bringing together all ROS 2 concepts
keywords: [ROS 2, lab, hands-on, project]
---

# Chapter 1.6: Lab - Building a ROS 2 Robot System

## Lab Overview

In this comprehensive lab, you'll build a complete robot system using everything learned in Module 1:

- Nodes for sensors, control, and monitoring
- Topics for continuous data streaming
- Services for configuration
- Actions for navigation tasks
- Parameters for runtime configuration
- Launch files for orchestration

## Project: Autonomous Patrol Robot

### System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Patrol Robot System                       │
│                                                              │
│  ┌──────────────┐    /scan     ┌──────────────────────┐    │
│  │ Lidar Node   │─────────────>│                      │    │
│  └──────────────┘              │   Navigation Node    │    │
│                                │                      │    │
│  ┌──────────────┐   /odom      │  - Path planning     │    │
│  │ Odometry Node│─────────────>│  - Obstacle avoid    │    │
│  └──────────────┘              │  - Goal handling     │    │
│                                └───────────┬──────────┘    │
│                                            │               │
│                                    /cmd_vel│               │
│                                            ▼               │
│  ┌──────────────┐  /patrol     ┌──────────────────────┐   │
│  │ Patrol       │<─────────────│   Motor Controller   │   │
│  │ Coordinator  │   (action)   │                      │   │
│  └──────────────┘              └──────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Step 1: Create the Package

```bash
cd ~/ros2_ws/src
ros2 pkg create patrol_robot --build-type ament_python \
  --dependencies rclpy std_msgs geometry_msgs sensor_msgs nav_msgs
```

## Step 2: Implement Sensor Node

```python
# patrol_robot/lidar_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import random

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)
        self.get_logger().info('Lidar node started')

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.angle_min = -3.14159
        msg.angle_max = 3.14159
        msg.angle_increment = 0.01745
        msg.range_min = 0.1
        msg.range_max = 10.0
        msg.ranges = [random.uniform(0.5, 5.0) for _ in range(360)]
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = LidarNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

## Step 3: Implement Navigation Action

```python
# patrol_robot/navigation_server.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav2_msgs.action import NavigateToPose
import time

class NavigationServer(Node):
    def __init__(self):
        super().__init__('navigation_server')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Navigating to goal...')

        # Simulate navigation with feedback
        for i in range(10):
            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = float(10 - i)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.5)

        goal_handle.succeed()
        result = NavigateToPose.Result()
        return result
```

## Step 4: Create Launch File

```python
# launch/patrol.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='patrol_robot',
            executable='lidar_node',
            name='lidar',
        ),
        Node(
            package='patrol_robot',
            executable='navigation_server',
            name='navigation',
            parameters=[{'max_speed': 1.0}],
        ),
    ])
```

## Step 5: Build and Run

```bash
cd ~/ros2_ws
colcon build --packages-select patrol_robot
source install/setup.bash
ros2 launch patrol_robot patrol.launch.py
```

## Verification

```bash
# Check nodes are running
ros2 node list

# Monitor scan topic
ros2 topic echo /scan

# Send navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{}"
```

## Challenges

1. **Add battery monitoring** - Create a node that publishes battery level
2. **Implement patrol waypoints** - Store and cycle through patrol points
3. **Add emergency stop** - Service to immediately halt the robot

## Lab Complete!

Congratulations! You've built a complete ROS 2 robot system. You're now ready to move on to **Module 2: Simulation** where you'll bring your robot to life in Gazebo and Unity.

[Continue to Module 2 →](/ur/docs/module-2-simulation/chapter-1-intro)
