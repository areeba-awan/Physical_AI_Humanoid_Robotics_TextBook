---
sidebar_position: 6
title: "1.6 لیب: آر او ایس 2 روبوٹ بنانا"
description: تمام آر او ایس 2 تصورات کو یکجا کرنے والی مکمل عملی لیب
keywords: [ROS 2, lab, hands-on, project]
---

# باب 1.6: لیب - آر او ایس 2 روبوٹ سسٹم بنانا

## لیب کا جائزہ

اس جامع لیب میں، آپ ماڈیول 1 میں سیکھی گئی ہر چیز کا استعمال کرتے ہوئے ایک مکمل روبوٹ سسٹم بنائیں گے:

- سینسرز، کنٹرول، اور مانیٹرنگ کے لیے نوڈز
- مسلسل ڈیٹا سٹریمنگ کے لیے ٹاپکس
- کنفیگریشن کے لیے سروسز
- نیویگیشن کاموں کے لیے ایکشنز
- رن ٹائم کنفیگریشن کے لیے پیرامیٹرز
- آرکیسٹریشن کے لیے لانچ فائلز

## پروجیکٹ: خودکار گشت روبوٹ

### سسٹم آرکیٹیکچر

```
┌─────────────────────────────────────────────────────────────┐
│                    گشت روبوٹ سسٹم                           │
│                                                              │
│  ┌──────────────┐    /scan     ┌──────────────────────┐    │
│  │ لیڈار نوڈ    │─────────────>│                      │    │
│  └──────────────┘              │   نیویگیشن نوڈ       │    │
│                                │                      │    │
│  ┌──────────────┐   /odom      │  - راستے کی منصوبہ بندی│    │
│  │ اوڈومیٹری نوڈ│─────────────>│  - رکاوٹ سے بچاؤ     │    │
│  └──────────────┘              │  - گول ہینڈلنگ       │    │
│                                └───────────┬──────────┘    │
│                                            │               │
│                                    /cmd_vel│               │
│                                            ▼               │
│  ┌──────────────┐  /patrol     ┌──────────────────────┐   │
│  │ گشت          │<─────────────│   موٹر کنٹرولر      │   │
│  │ کوآرڈینیٹر   │   (ایکشن)    │                      │   │
│  └──────────────┘              └──────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## مرحلہ 1: پیکیج بنائیں

```bash
cd ~/ros2_ws/src
ros2 pkg create patrol_robot --build-type ament_python \
  --dependencies rclpy std_msgs geometry_msgs sensor_msgs nav_msgs
```

## مرحلہ 2: سینسر نوڈ نافذ کریں

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
        self.get_logger().info('لیڈار نوڈ شروع ہو گیا')

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

## مرحلہ 3: نیویگیشن ایکشن نافذ کریں

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
        self.get_logger().info('گول کی طرف نیویگیٹ کر رہا ہے...')

        # فیڈبیک کے ساتھ نیویگیشن کی سمولیشن
        for i in range(10):
            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = float(10 - i)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.5)

        goal_handle.succeed()
        result = NavigateToPose.Result()
        return result
```

## مرحلہ 4: لانچ فائل بنائیں

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

## مرحلہ 5: بلڈ اور چلائیں

```bash
cd ~/ros2_ws
colcon build --packages-select patrol_robot
source install/setup.bash
ros2 launch patrol_robot patrol.launch.py
```

## تصدیق

```bash
# چیک کریں کہ نوڈز چل رہے ہیں
ros2 node list

# سکین ٹاپک مانیٹر کریں
ros2 topic echo /scan

# نیویگیشن گول بھیجیں
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{}"
```

## چیلنجز

1. **بیٹری مانیٹرنگ شامل کریں** - ایک نوڈ بنائیں جو بیٹری کی سطح پبلش کرے
2. **گشت وے پوائنٹس نافذ کریں** - گشت پوائنٹس سٹور کریں اور ان میں چکر لگائیں
3. **ایمرجنسی اسٹاپ شامل کریں** - روبوٹ کو فوری طور پر روکنے کی سروس

## لیب مکمل!

مبارک ہو! آپ نے ایک مکمل آر او ایس 2 روبوٹ سسٹم بنا لیا ہے۔ اب آپ **ماڈیول 2: سمولیشن** پر جانے کے لیے تیار ہیں جہاں آپ اپنے روبوٹ کو گزیبو اور یونٹی میں زندہ کریں گے۔

[ماڈیول 2 پر جائیں →](/docs/module-2-simulation/chapter-1-intro)
