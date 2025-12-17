---
sidebar_position: 5
title: "3.5 Isaac ROS Integration"
description: Connecting Isaac Sim with ROS 2
keywords: [Isaac ROS, ROS 2, integration, bridge]
---

# Chapter 3.5: Isaac ROS Integration

## Learning Objectives

- Set up Isaac Sim ROS 2 bridge
- Stream sensor data from simulation
- Control robots via ROS 2
- Deploy to Jetson hardware

## ROS 2 Bridge Setup

### Enable in Isaac Sim

```python
import omni.isaac.ros2_bridge as ros2_bridge

# Initialize ROS 2
ros2_bridge.init()

# Create publishers
clock_pub = ros2_bridge.create_clock_publisher()
camera_pub = ros2_bridge.create_camera_publisher("/camera/image_raw")
lidar_pub = ros2_bridge.create_lidar_publisher("/scan")
```

### OmniGraph Nodes

Create visual pipelines with OmniGraph:

```python
import omni.graph.core as og

keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/ActionGraph"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
            ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
            ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
        ],
    }
)
```

## Sensor Streaming

### Camera to ROS 2

```python
from omni.isaac.sensor import Camera
from omni.isaac.ros2_bridge import ROS2CameraHelper

camera = Camera(prim_path="/World/Camera")
ros2_helper = ROS2CameraHelper(camera, "/camera")
ros2_helper.publish_rgb()
ros2_helper.publish_depth()
ros2_helper.publish_camera_info()
```

### LiDAR to ROS 2

```python
from omni.isaac.sensor import RotatingLidarPhysX

lidar = RotatingLidarPhysX(prim_path="/World/Lidar")
lidar.add_point_cloud_data_to_frame()
```

## Robot Control

### Joint Commands

```python
# Subscribe to joint commands
from sensor_msgs.msg import JointState

def joint_command_callback(msg):
    for i, name in enumerate(msg.name):
        joint_idx = robot.get_dof_index(name)
        robot.set_joint_position_target(msg.position[i], joint_idx)
```

### Velocity Commands

```python
from geometry_msgs.msg import Twist

def cmd_vel_callback(msg):
    linear = msg.linear.x
    angular = msg.angular.z

    # Differential drive kinematics
    left_vel = linear - angular * wheel_separation / 2
    right_vel = linear + angular * wheel_separation / 2
    robot.apply_wheel_velocities([left_vel, right_vel])
```

## Jetson Deployment

### Isaac ROS on Jetson

```bash
# On Jetson
sudo apt install nvidia-l4t-isaac-ros-packages

# Run perception pipeline
ros2 launch isaac_ros_yolov8 isaac_ros_yolov8_jetson.launch.py
```

### Performance Optimization

| Package | Xavier NX | Orin Nano |
|---------|-----------|-----------|
| YOLOv8 | 15 FPS | 30 FPS |
| Visual SLAM | 30 FPS | 60 FPS |
| cuMotion | 100 Hz | 200 Hz |

## Hands-on Lab

### Lab 3.5: Full Stack Integration

Create a system with:
1. Isaac Sim simulation
2. ROS 2 perception nodes
3. Navigation stack
4. Visualization in RViz

## Summary

- ROS 2 bridge connects Isaac Sim to ROS ecosystem
- OmniGraph enables visual programming
- Jetson enables edge deployment

[Continue to Chapter 3.6: Lab →](/ur/docs/module-3-isaac/chapter-6-lab)
