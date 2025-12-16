---
sidebar_position: 6
title: "3.6 لیب: AI سے چلنے والی مینیپولیشن"
description: ایک ذہین مینیپولیشن سسٹم بنانا
keywords: [مینیپولیشن, AI, Isaac, پرسیپشن, پلاننگ]
---

# باب 3.6: لیب - AI سے چلنے والی مینیپولیشن

## لیب کا جائزہ

ایک مکمل AI سے چلنے والا مینیپولیشن سسٹم بنائیں جو:
- نیورل نیٹ ورکس کا استعمال کرتے ہوئے آبجیکٹس کا پتہ لگائے
- ٹکراؤ سے پاک موشنز کی منصوبہ بندی کرے
- پک اینڈ پلیس کام انجام دے

## پروجیکٹ: سمارٹ چھانٹنے والا روبوٹ

### سسٹم آرکیٹیکچر

```
┌─────────────────────────────────────────────────────────────┐
│               سمارٹ چھانٹنے کا سسٹم                          │
│                                                              │
│  ┌─────────────────────────────────────────────────────┐   │
│  │                  ISAAC SIM                            │   │
│  │                                                       │   │
│  │  ┌───────────┐  ┌───────────┐  ┌───────────┐       │   │
│  │  │  کیمرہ    │  │  روبوٹ   │  │  آبجیکٹس  │       │   │
│  │  │  (RGBD)   │  │  آرم     │  │  (رینڈم)  │       │   │
│  │  └─────┬─────┘  └─────┬─────┘  └───────────┘       │   │
│  │        │              │                              │   │
│  └────────┼──────────────┼──────────────────────────────┘   │
│           │              │                                   │
│           │ /camera      │ /joint_states                    │
│           ▼              ▼                                   │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │  YOLOv8     │  │  cuMotion   │  │  ٹاسک      │        │
│  │  ڈیٹیکشن   │──│  پلانر     │──│  مینیجر    │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## مرحلہ 1: سین سیٹ اپ

```python
# setup_scene.py
from omni.isaac.core import World
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.core.objects import DynamicCuboid
import random

world = World()

# روبوٹ آرم شامل کریں
robot = world.scene.add(
    SingleManipulator(
        prim_path="/World/Franka",
        name="franka",
        usd_path="/Isaac/Robots/Franka/franka.usd"
    )
)

# رینڈم آبجیکٹس شامل کریں
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

## مرحلہ 2: آبجیکٹ ڈیٹیکشن نوڈ

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
        # ROS تصویر کو OpenCV میں تبدیل کریں
        image = self.bridge.imgmsg_to_cv2(msg)

        # ڈیٹیکشن چلائیں
        results = self.model(image)

        # ڈیٹیکشنز پبلش کریں
        detection_msg = self.results_to_msg(results)
        self.publisher.publish(detection_msg)
```

## مرحلہ 3: ٹاسک مینیجر

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

            # پک اینڈ پلیس انجام دیں
            self.execute_pick_place(obj_pose, target_bin)

    def execute_pick_place(self, pick_pose, place_pose):
        # اپروچ
        approach_pose = self.compute_approach(pick_pose)
        self.planner.move_to_pose(approach_pose)

        # اٹھائیں
        self.planner.move_to_pose(pick_pose)
        self.gripper.close()

        # اوپر اٹھائیں
        lift_pose = pick_pose.copy()
        lift_pose.z += 0.1
        self.planner.move_to_pose(lift_pose)

        # رکھیں
        self.planner.move_to_pose(place_pose)
        self.gripper.open()

        # گھر واپس جائیں
        self.planner.move_to_home()
```

## مرحلہ 4: سسٹم لانچ کریں

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

## تصدیق

```bash
# سین کے ساتھ Isaac Sim لانچ کریں
./isaac_sim.sh --scene sorting_scene.usd

# ROS 2 نوڈز لانچ کریں
ros2 launch sorting_robot sorting_system.launch.py

# ڈیٹیکشنز مانیٹر کریں
ros2 topic echo /detections

# روبوٹ کی حالت مانیٹر کریں
ros2 topic echo /joint_states
```

## چیلنج ایکسٹینشنز

1. **کنویئر بیلٹ شامل کریں** - آبجیکٹس مسلسل آتے ہیں
2. **اوکلوژنز ہینڈل کریں** - جزوی طور پر نظر آنے والے آبجیکٹس کا پتہ لگائیں
3. **سائیکل ٹائم بہتر بنائیں** - متوازی پلاننگ اور ایگزیکیوشن

## لیب مکمل!

آپ نے ایک AI سے چلنے والا مینیپولیشن سسٹم بنایا ہے۔ اگلے میں، ہم Vision-Language-Action ماڈلز کی تحقیق کریں گے تاکہ اور بھی ذہین روبوٹس بنائیں۔

[ماڈیول 4: VLA پر جائیں ←](/docs/module-4-vla/chapter-1-intro)
