---
sidebar_position: 3
title: "3.3 Isaac کے ساتھ پرسیپشن"
description: GPU سے تیز پرسیپشن پائپ لائنز
keywords: [پرسیپشن, DNN, آبجیکٹ ڈیٹیکشن, سیگمنٹیشن]
---

# باب 3.3: Isaac کے ساتھ پرسیپشن

## سیکھنے کے مقاصد

- آبجیکٹ ڈیٹیکشن کے لیے DNN ماڈلز ڈیپلائے کریں
- Isaac ROS پرسیپشن پیکجز استعمال کریں
- ویژول SLAM نافذ کریں
- پرسیپشن پائپ لائنز بنائیں

## Isaac ROS DNN انفرنس

### آبجیکٹ ڈیٹیکشن پائپ لائن

```python
# آبجیکٹ ڈیٹیکشن لانچ کریں
ros2 launch isaac_ros_yolov8 isaac_ros_yolov8.launch.py \
    model_file_path:=/models/yolov8n.onnx \
    input_image_width:=640 \
    input_image_height:=480
```

### ڈیٹیکشن کے نتائج

```python
# ڈیٹیکشنز کی سبسکرائب کریں
from vision_msgs.msg import Detection2DArray

def detection_callback(msg):
    for detection in msg.detections:
        bbox = detection.bbox
        class_id = detection.results[0].hypothesis.class_id
        score = detection.results[0].hypothesis.score
        print(f"پتہ لگایا {class_id}: {score:.2f}")
```

## ویژول SLAM

### Isaac ROS ویژول SLAM

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

### آؤٹ پٹس

- `/visual_slam/tracking/odometry` - پوز کا اندازہ
- `/visual_slam/vis/slam_odometry` - ویژولائزیشن
- `/visual_slam/status` - ٹریکنگ کی حیثیت

## سیمنٹک سیگمنٹیشن

```python
# سیگمنٹیشن لانچ کریں
ros2 launch isaac_ros_unet isaac_ros_unet.launch.py \
    model_file_path:=/models/unet.onnx
```

## AprilTag ڈیٹیکشن

```python
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py

# ٹیگ پوزز کی سبسکرائب کریں
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

def apriltag_callback(msg):
    for detection in msg.detections:
        tag_id = detection.id
        pose = detection.pose
```

## پرسیپشن پائپ لائن

```
┌─────────┐    ┌──────────────┐    ┌────────────┐
│ کیمرہ   │───>│ DNN انفرنس   │───>│ ڈیٹیکشن    │
│         │    │  (TensorRT)  │    │  کے نتائج  │
└─────────┘    └──────────────┘    └────────────┘
     │
     │         ┌──────────────┐    ┌────────────┐
     └────────>│ ویژول SLAM  │───>│   پوز      │
               │  (cuVSLAM)   │    │  کا اندازہ │
               └──────────────┘    └────────────┘
```

## عملی لیب

### لیب 3.3: آبجیکٹ ڈیٹیکشن سسٹم

ایک پرسیپشن سسٹم بنائیں جو:
1. YOLOv8 استعمال کرتے ہوئے آبجیکٹس کا پتہ لگائے
2. ویژول SLAM کے ساتھ کیمرہ پوز ٹریک کرے
3. 3D آبجیکٹ پوزیشنز پبلش کرے

## خلاصہ

- Isaac ROS GPU سے تیز پرسیپشن فراہم کرتا ہے
- TensorRT DNN انفرنس کو بہتر بناتا ہے
- ویژول SLAM لوکلائزیشن فعال کرتا ہے

[باب 3.4 پر جائیں ←](/docs/module-3-isaac/chapter-4-manipulation)
