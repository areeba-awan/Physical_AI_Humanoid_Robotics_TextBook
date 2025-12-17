---
sidebar_position: 3
title: "3.3 Perception with Isaac"
description: GPU-accelerated perception pipelines
keywords: [perception, DNN, object detection, segmentation]
---

# Chapter 3.3: Perception with Isaac

## Learning Objectives

- Deploy DNN models for object detection
- Use Isaac ROS perception packages
- Implement visual SLAM
- Create perception pipelines

## Isaac ROS DNN Inference

### Object Detection Pipeline

```python
# Launch object detection
ros2 launch isaac_ros_yolov8 isaac_ros_yolov8.launch.py \
    model_file_path:=/models/yolov8n.onnx \
    input_image_width:=640 \
    input_image_height:=480
```

### Detection Results

```python
# Subscribe to detections
from vision_msgs.msg import Detection2DArray

def detection_callback(msg):
    for detection in msg.detections:
        bbox = detection.bbox
        class_id = detection.results[0].hypothesis.class_id
        score = detection.results[0].hypothesis.score
        print(f"Detected {class_id}: {score:.2f}")
```

## Visual SLAM

### Isaac ROS Visual SLAM

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

### Outputs

- `/visual_slam/tracking/odometry` - Pose estimate
- `/visual_slam/vis/slam_odometry` - Visualization
- `/visual_slam/status` - Tracking status

## Semantic Segmentation

```python
# Launch segmentation
ros2 launch isaac_ros_unet isaac_ros_unet.launch.py \
    model_file_path:=/models/unet.onnx
```

## AprilTag Detection

```python
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py

# Subscribe to tag poses
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

def apriltag_callback(msg):
    for detection in msg.detections:
        tag_id = detection.id
        pose = detection.pose
```

## Perception Pipeline

```
┌─────────┐    ┌──────────────┐    ┌────────────┐
│ Camera  │───>│ DNN Inference │───>│ Detection │
│         │    │   (TensorRT)  │    │  Results   │
└─────────┘    └──────────────┘    └────────────┘
     │
     │         ┌──────────────┐    ┌────────────┐
     └────────>│ Visual SLAM  │───>│   Pose     │
               │   (cuVSLAM)  │    │  Estimate  │
               └──────────────┘    └────────────┘
```

## Hands-on Lab

### Lab 3.3: Object Detection System

Create a perception system that:
1. Detects objects using YOLOv8
2. Tracks camera pose with Visual SLAM
3. Publishes 3D object positions

## Summary

- Isaac ROS provides GPU-accelerated perception
- TensorRT optimizes DNN inference
- Visual SLAM enables localization

[Continue to Chapter 3.4 →](/ur/docs/module-3-isaac/chapter-4-manipulation)
