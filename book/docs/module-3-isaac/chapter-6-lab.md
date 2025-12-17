---
sidebar_position: 6
title: "3.6 Lab: AI-Powered Manipulation"
description: Building an intelligent manipulation system
keywords: [manipulation, AI, Isaac, perception, planning]
---

# Chapter 3.6: Lab - AI-Powered Manipulation

## Lab Overview

Build a complete AI-powered manipulation system that:
- Detects objects using neural networks
- Plans collision-free motions
- Executes pick-and-place tasks

## Project: Smart Sorting Robot

### System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│               SMART SORTING SYSTEM                           │
│                                                              │
│  ┌─────────────────────────────────────────────────────┐   │
│  │                  ISAAC SIM                            │   │
│  │                                                       │   │
│  │  ┌───────────┐  ┌───────────┐  ┌───────────┐       │   │
│  │  │  Camera   │  │  Robot    │  │  Objects  │       │   │
│  │  │  (RGBD)   │  │  Arm      │  │  (Random) │       │   │
│  │  └─────┬─────┘  └─────┬─────┘  └───────────┘       │   │
│  │        │              │                              │   │
│  └────────┼──────────────┼──────────────────────────────┘   │
│           │              │                                   │
│           │ /camera      │ /joint_states                    │
│           ▼              ▼                                   │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │  YOLOv8     │  │  cuMotion   │  │  Task       │        │
│  │  Detection  │──│  Planner    │──│  Manager    │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## Step 1: Scene Setup

```python
# setup_scene.py
from omni.isaac.core import World
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.core.objects import DynamicCuboid
import random

world = World()

# Add robot arm
robot = world.scene.add(
    SingleManipulator(
        prim_path="/World/Franka",
        name="franka",
        usd_path="/Isaac/Robots/Franka/franka.usd"
    )
)

# Add random objects
colors = ["red", "green", "blue"]
for i in range(5):
    world.scene.add(
        DynamicCuboid(
            prim_path=f"/World/cube_{i}",
            size=0.05,
            position=[
                random.uniform(-0.3, 0.3),
                random.uniform(0.3, 0.5),
                0.025
            ],
            color=random.choice(colors)
        )
    )
```

## Step 2: Object Detection Node

```python
# object_detector.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
import cv2
from ultralytics import YOLO

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.model = YOLO('yolov8n.pt')

        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.publisher = self.create_publisher(
            Detection2DArray, '/detections', 10
        )

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        image = self.bridge.imgmsg_to_cv2(msg)

        # Run detection
        results = self.model(image)

        # Publish detections
        detection_msg = self.results_to_msg(results)
        self.publisher.publish(detection_msg)
```

## Step 3: Task Manager

```python
# task_manager.py
class SortingTaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/detections', self.detection_callback, 10
        )
        self.planner = MotionPlanner()
        self.gripper = GripperController()

    def detection_callback(self, msg):
        for detection in msg.detections:
            obj_pose = self.get_object_pose(detection)
            target_bin = self.get_target_bin(detection.class_id)

            # Execute pick and place
            self.execute_pick_place(obj_pose, target_bin)

    def execute_pick_place(self, pick_pose, place_pose):
        # Approach
        approach_pose = self.compute_approach(pick_pose)
        self.planner.move_to_pose(approach_pose)

        # Pick
        self.planner.move_to_pose(pick_pose)
        self.gripper.close()

        # Lift
        lift_pose = pick_pose.copy()
        lift_pose.z += 0.1
        self.planner.move_to_pose(lift_pose)

        # Place
        self.planner.move_to_pose(place_pose)
        self.gripper.open()

        # Return home
        self.planner.move_to_home()
```

## Step 4: Launch System

```python
# launch/sorting_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sorting_robot',
            executable='object_detector',
            name='detector',
        ),
        Node(
            package='sorting_robot',
            executable='task_manager',
            name='task_manager',
        ),
        Node(
            package='sorting_robot',
            executable='motion_planner',
            name='planner',
        ),
    ])
```

## Verification

```bash
# Launch Isaac Sim with scene
./isaac_sim.sh --scene sorting_scene.usd

# Launch ROS 2 nodes
ros2 launch sorting_robot sorting_system.launch.py

# Monitor detections
ros2 topic echo /detections

# Monitor robot state
ros2 topic echo /joint_states
```

## Challenge Extensions

1. **Add conveyor belt** - Objects arrive continuously
2. **Handle occlusions** - Detect partially visible objects
3. **Optimize cycle time** - Parallel planning and execution

## Lab Complete!

You've built an AI-powered manipulation system. Next, we'll explore Vision-Language-Action models for even more intelligent robots.

[Continue to Module 4: VLA →](/docs/module-4-vla/chapter-1-intro)


