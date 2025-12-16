---
sidebar_position: 2
title: "1.2 نوڈز، ٹاپکس، اور سروسز"
description: آر او ایس 2 کمیونیکیشن پیٹرنز کو سمجھنا
keywords: [ROS 2, nodes, topics, services, publishers, subscribers]
---

# باب 1.2: نوڈز، ٹاپکس، اور سروسز

## سیکھنے کے مقاصد

اس باب کے آخر تک، آپ یہ کر سکیں گے:

- آر او ایس 2 نوڈز بنائیں اور ان کا انتظام کریں
- ٹاپکس کے ساتھ پبلشر-سبسکرائبر پیٹرنز نافذ کریں
- درخواست-جواب کمیونیکیشن کے لیے سروسز بنائیں اور کال کریں
- اپنے استعمال کے کیس کے لیے صحیح کمیونیکیشن پیٹرن منتخب کریں

## شرائط

- [ ] باب 1.1 مکمل
- [ ] آر او ایس 2 ورک سپیس سیٹ اپ
- [ ] بنیادی پائتھون کا علم

## نوڈز کو سمجھنا

**نوڈ** ایک پروسیس ہے جو حساب کتاب کرتی ہے۔ نوڈز آر او ایس 2 ایپلیکیشنز کے بنیادی بلڈنگ بلاکس ہیں۔

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('نوڈ شروع ہو گیا!')

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

### نوڈ لائف سائیکل

```
┌─────────────┐
│  غیر کنفیگرڈ  │
└──────┬──────┘
       │ configure()
       ▼
┌─────────────┐
│   غیر فعال   │
└──────┬──────┘
       │ activate()
       ▼
┌─────────────┐
│    فعال     │◄──── عام آپریشن
└──────┬──────┘
       │ deactivate()
       ▼
┌─────────────┐
│   غیر فعال   │
└─────────────┘
```

## ٹاپکس: پبلش-سبسکرائب

ٹاپکس **غیر ہم وقت ساز**، **کثیر سے کثیر** کمیونیکیشن فعال کرتے ہیں۔

### پبلشر بنانا

```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'my_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = 'ہیلو، آر او ایس 2!'
        self.publisher.publish(msg)
```

### سبسکرائبر بنانا

```python
class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'موصول ہوا: {msg.data}')
```

### سروس کا معیار (کیو او ایس)

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

self.publisher = self.create_publisher(String, 'topic', qos_profile)
```

## سروسز: درخواست-جواب

سروسز درخواست-جواب پیٹرنز کے لیے **ہم وقت ساز** کمیونیکیشن فراہم کرتی ہیں۔

### سروس کی تعریف

```python
# سروس کی تعریف (srv/AddTwoInts.srv)
# int64 a
# int64 b
# ---
# int64 sum
```

### سروس سرور

```python
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
```

### سروس کلائنٹ

```python
class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('سروس کا انتظار...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        return future
```

## کب کیا استعمال کریں؟

| پیٹرن | استعمال کا کیس | مثال |
|---------|----------|---------|
| ٹاپکس | مسلسل ڈیٹا سٹریمز | سینسر ریڈنگز، کیمرہ تصاویر |
| سروسز | ایک بار کی درخواستیں | روبوٹ سپان کریں، نقشہ محفوظ کریں |
| ایکشنز | طویل المدت کام | منزل پر جائیں، شے اٹھائیں |

## عملی لیب

### لیب 1.2: درجہ حرارت مانیٹر بنائیں

ایک سسٹم بنائیں جس میں:
1. درجہ حرارت کی ریڈنگز پبلش کرنے والا سینسر نوڈ
2. سبسکرائب کرنے والا اور زیادہ درجہ حرارت پر الرٹ دینے والا مانیٹر نوڈ
3. موجودہ درجہ حرارت حاصل کرنے کے لیے سروس

```bash
# اپنا حل چلائیں
ros2 run my_package temperature_sensor
ros2 run my_package temperature_monitor
ros2 service call /get_temperature ...
```

## علم کی جانچ

1. مسلسل سینسر ڈیٹا کے لیے کون سا کمیونیکیشن پیٹرن استعمال کرنا چاہیے؟
   - [x] ٹاپکس
   - [ ] سروسز
   - [ ] ایکشنز

2. سروسز _____ ہیں جبکہ ٹاپکس _____ ہیں۔
   - [x] ہم وقت ساز، غیر ہم وقت ساز
   - [ ] غیر ہم وقت ساز، ہم وقت ساز
   - [ ] دونوں ہم وقت ساز

## خلاصہ

- **نوڈز** حساب کتاب کرنے والی آزاد پروسیسز ہیں
- **ٹاپکس** سٹریمنگ ڈیٹا کے لیے پبلش-سبسکرائب استعمال کرتے ہیں
- **سروسز** ایک بار کے آپریشنز کے لیے درخواست-جواب استعمال کرتی ہیں
- **کیو او ایس** پروفائلز اعتمادیت اور ڈیلیوری گارنٹیز کنٹرول کرتے ہیں

## اگلے اقدامات

[باب 1.3: ایکشنز اور پیرامیٹرز پر جائیں →](/docs/module-1-ros2/chapter-3-actions-params)
