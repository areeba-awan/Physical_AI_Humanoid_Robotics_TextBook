---
sidebar_position: 3
title: "1.3 ایکشنز اور پیرامیٹرز"
description: آر او ایس 2 میں طویل المدت کام اور رن ٹائم کنفیگریشن
keywords: [ROS 2, actions, parameters, goal, feedback]
---

# باب 1.3: ایکشنز اور پیرامیٹرز

## سیکھنے کے مقاصد

- طویل المدت کاموں کے لیے آر او ایس 2 ایکشنز نافذ کریں
- رن ٹائم کنفیگریشن کے لیے پیرامیٹرز استعمال کریں
- ایکشن گولز، فیڈبیک، اور نتائج سنبھالیں

## ایکشنز: طویل المدت کام

ایکشنز ایسے کاموں کے لیے مثالی ہیں جو:
- مکمل ہونے میں زیادہ وقت لیتے ہیں
- پیشرفت فیڈبیک فراہم کرنے کی ضرورت ہے
- عمل کے دوران منسوخ ہو سکتے ہیں

### ایکشن کا ڈھانچہ

```
┌─────────────┐     گول       ┌─────────────┐
│   کلائنٹ    │──────────────>│   سرور     │
│             │               │             │
│             │<──────────────│             │
│             │    نتیجہ      │             │
│             │               │             │
│             │<──────────────│             │
│             │   فیڈبیک     │             │
└─────────────┘    (سٹریم)    └─────────────┘
```

### ایکشن کی تعریف

```
# action/Navigate.action
# گول
float64 target_x
float64 target_y
---
# نتیجہ
bool success
float64 time_elapsed
---
# فیڈبیک
float64 distance_remaining
float64 current_x
float64 current_y
```

### ایکشن سرور

```python
from rclpy.action import ActionServer
from my_interfaces.action import Navigate

class NavigationServer(Node):
    def __init__(self):
        super().__init__('navigation_server')
        self._action_server = ActionServer(
            self,
            Navigate,
            'navigate',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info('نیویگیشن کا عمل جاری...')

        feedback_msg = Navigate.Feedback()

        # نیویگیشن کی سمولیشن
        for i in range(10):
            feedback_msg.distance_remaining = 10.0 - i
            goal_handle.publish_feedback(feedback_msg)
            await asyncio.sleep(1.0)

        goal_handle.succeed()

        result = Navigate.Result()
        result.success = True
        result.time_elapsed = 10.0
        return result
```

### ایکشن کلائنٹ

```python
from rclpy.action import ActionClient

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self._action_client = ActionClient(self, Navigate, 'navigate')

    def send_goal(self, x, y):
        goal_msg = Navigate.Goal()
        goal_msg.target_x = x
        goal_msg.target_y = y

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'باقی فاصلہ: {feedback.distance_remaining}')
```

## پیرامیٹرز: رن ٹائم کنفیگریشن

پیرامیٹرز آپ کو دوبارہ کمپائل کیے بغیر نوڈز کنفیگر کرنے دیتے ہیں۔

### پیرامیٹرز کا اعلان

```python
class ParameterizedNode(Node):
    def __init__(self):
        super().__init__('parameterized_node')

        # ڈیفالٹ ویلیوز کے ساتھ اعلان کریں
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('sensors', ['lidar', 'camera'])

    def get_config(self):
        name = self.get_parameter('robot_name').value
        speed = self.get_parameter('max_speed').value
        return name, speed
```

### سی ایل آئی سے پیرامیٹرز سیٹ کرنا

```bash
# لانچ پر سیٹ کریں
ros2 run my_package my_node --ros-args -p robot_name:=my_robot -p max_speed:=2.0

# رن ٹائم پر سیٹ کریں
ros2 param set /my_node robot_name "new_robot"

# پیرامیٹر حاصل کریں
ros2 param get /my_node robot_name

# تمام پیرامیٹرز کی فہرست
ros2 param list /my_node
```

### پیرامیٹر فائلز (یامل)

```yaml
# config/params.yaml
my_node:
  ros__parameters:
    robot_name: "atlas"
    max_speed: 1.5
    sensors:
      - lidar
      - camera
      - imu
```

```bash
ros2 run my_package my_node --ros-args --params-file config/params.yaml
```

## عملی لیب

### لیب 1.3: گرپر ایکشن سرور

ایک ایکشن سرور نافذ کریں جو:
1. گرپ فورس گول قبول کرے
2. گرپر بند ہونے کے دوران فیڈبیک بھیجے
3. کامیابی/ناکامی کا نتیجہ واپس کرے

## علم کی جانچ

1. سروسز کی بجائے ایکشنز کب استعمال کرنی چاہیئیں؟
   - [x] فیڈبیک کے ساتھ طویل المدت کاموں کے لیے
   - [ ] فوری ایک بار کی درخواستوں کے لیے
   - [ ] سٹریمنگ سینسر ڈیٹا کے لیے

## خلاصہ

- **ایکشنز** گول، فیڈبیک، اور نتیجے کے ساتھ طویل المدت کام سنبھالتے ہیں
- **پیرامیٹرز** دوبارہ کمپائل کیے بغیر رن ٹائم کنفیگریشن فراہم کرتے ہیں
- متعدد پیرامیٹرز کے انتظام کے لیے یامل فائلز استعمال کریں

## اگلے اقدامات

[باب 1.4: لانچ فائلز پر جائیں →](/docs/module-1-ros2/chapter-4-launch-files)
