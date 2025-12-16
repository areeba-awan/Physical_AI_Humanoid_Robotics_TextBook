---
sidebar_position: 5
title: "3.5 Isaac ROS انٹیگریشن"
description: Isaac Sim کو ROS 2 سے منسلک کرنا
keywords: [Isaac ROS, ROS 2, انٹیگریشن, برج]
---

# باب 3.5: Isaac ROS انٹیگریشن

## سیکھنے کے مقاصد

- Isaac Sim ROS 2 برج سیٹ اپ کریں
- سمولیشن سے سینسر ڈیٹا سٹریم کریں
- ROS 2 کے ذریعے روبوٹس کنٹرول کریں
- Jetson ہارڈویئر پر ڈیپلائے کریں

## ROS 2 برج سیٹ اپ

### Isaac Sim میں فعال کریں

```python
import omni.isaac.ros2_bridge as ros2_bridge

# ROS 2 شروع کریں
ros2_bridge.init()

# پبلشرز بنائیں
clock_pub = ros2_bridge.create_clock_publisher()
camera_pub = ros2_bridge.create_camera_publisher("/camera/image_raw")
lidar_pub = ros2_bridge.create_lidar_publisher("/scan")
```

### OmniGraph نوڈز

OmniGraph کے ساتھ ویژول پائپ لائنز بنائیں:

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

## سینسر سٹریمنگ

### کیمرہ سے ROS 2

```python
from omni.isaac.sensor import Camera
from omni.isaac.ros2_bridge import ROS2CameraHelper

camera = Camera(prim_path="/World/Camera")
ros2_helper = ROS2CameraHelper(camera, "/camera")
ros2_helper.publish_rgb()
ros2_helper.publish_depth()
ros2_helper.publish_camera_info()
```

### LiDAR سے ROS 2

```python
from omni.isaac.sensor import RotatingLidarPhysX

lidar = RotatingLidarPhysX(prim_path="/World/Lidar")
lidar.add_point_cloud_data_to_frame()
```

## روبوٹ کنٹرول

### جوائنٹ کمانڈز

```python
# جوائنٹ کمانڈز کی سبسکرائب کریں
from sensor_msgs.msg import JointState

def joint_command_callback(msg):
    for i, name in enumerate(msg.name):
        joint_idx = robot.get_dof_index(name)
        robot.set_joint_position_target(msg.position[i], joint_idx)
```

### رفتار کمانڈز

```python
from geometry_msgs.msg import Twist

def cmd_vel_callback(msg):
    linear = msg.linear.x
    angular = msg.angular.z

    # ڈفرنشل ڈرائیو کینیمیٹکس
    left_vel = linear - angular * wheel_separation / 2
    right_vel = linear + angular * wheel_separation / 2
    robot.apply_wheel_velocities([left_vel, right_vel])
```

## Jetson ڈیپلائمنٹ

### Jetson پر Isaac ROS

```bash
# Jetson پر
sudo apt install nvidia-l4t-isaac-ros-packages

# پرسیپشن پائپ لائن چلائیں
ros2 launch isaac_ros_yolov8 isaac_ros_yolov8_jetson.launch.py
```

### کارکردگی کی اصلاح

| پیکج | Xavier NX | Orin Nano |
|---------|-----------|-----------|
| YOLOv8 | 15 FPS | 30 FPS |
| ویژول SLAM | 30 FPS | 60 FPS |
| cuMotion | 100 Hz | 200 Hz |

## عملی لیب

### لیب 3.5: مکمل اسٹیک انٹیگریشن

مندرجہ ذیل کے ساتھ ایک سسٹم بنائیں:
1. Isaac Sim سمولیشن
2. ROS 2 پرسیپشن نوڈز
3. نیویگیشن اسٹیک
4. RViz میں ویژولائزیشن

## خلاصہ

- ROS 2 برج Isaac Sim کو ROS ایکوسسٹم سے جوڑتا ہے
- OmniGraph ویژول پروگرامنگ فعال کرتا ہے
- Jetson ایج ڈیپلائمنٹ فعال کرتا ہے

[باب 3.6: لیب پر جائیں ←](/docs/module-3-isaac/chapter-6-lab)
