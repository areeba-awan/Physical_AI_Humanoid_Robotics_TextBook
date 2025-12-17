---
sidebar_position: 3
title: "3.3 ادراک with Isaac"
description: "GPU-accelerated perception pipelines"
keywords: ["perception", "DNN", "object detection", "segmentation"]
---

# باب 3.3: ادراک with Isaac

## سیکھنے کے مقاصد

- Deploy DNN models for object detection
- استعمال کریں Isaac ROS perception packages
- Implement visual SLAM
- تخلیق کریں perception pipelines

## Isaac ROS DNN Inference

### Object Detection Pipeline

```python
# لانچ کریں object detection
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
- `/visual_slam/vis/slam_odometry` -  وژولائزیشن
- `/visual_slam/status` - Tracking status

## Semantic Segmentation

```python
# لانچ کریں segmentation
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

## ادراک Pipeline

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

## ہاتھ سے کام کرنے والی لیب

### Lab 3.3: Object Detection سسٹم

تخلیق کریں a perception system that:
1. Detects objects using YOLOv8
2. Tracks camera pose with Visual SLAM
3. Publishes 3D object positions

## خلاصہ

- Isaac ROS provides GPU-accelerated perception
- TensorRT optimizes DNN inference
- Visual SLAM enables localization

[Continue to باب 3.4 →](/docs/module-3-isaac/chapter-4-manipulation)


