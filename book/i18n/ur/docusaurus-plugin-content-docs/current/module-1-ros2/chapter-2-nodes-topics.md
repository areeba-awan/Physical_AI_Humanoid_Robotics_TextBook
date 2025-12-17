---
sidebar_position: 2
title: "1.2 نوڈز، ٹاپکس، اور سروسز"
description: "کو سمجھنا ROS 2 communication patterns"
keywords: ["ROS 2", "nodes", "topics", "services", "publishers", "subscribers"]
---

# باب 1.2: نوڈز، ٹاپکس، اور سروسز

## سیکھنے کے مقاصد

By the end of this chapter, you will be able to:

- تخلیق کریں and manage ROS 2 nodes
- Implement publisher-subscriber patterns with topics
- تخلیق کریں and call services for request-response communication
- منتخب کریں the right communication pattern for your use case

## شرائطِ لازمہ

- [ ] Completed باب 1.1
- [ ] ROS 2 workspace set up
- [ ] Basic Python knowledge

## کو سمجھنا نوڈز

A **node** is a process that performs computation. نوڈز are the fundamental building blocks of ROS 2 applications.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('Node started!')

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

### Node Lifecycle

```
┌─────────────┐
│  Unconfigured │
└──────┬──────┘
       │ configure()
       ▼
┌─────────────┐
│   Inactive   │
└──────┬──────┘
       │ activate()
       ▼
┌─────────────┐
│    Active    │◄──── Normal operation
└──────┬──────┘
       │ deactivate()
       ▼
┌─────────────┐
│   Inactive   │
└─────────────┘
```

## ٹاپکس: Publish-Subscribe

ٹاپکس enable **asynchronous**, **many-to-many** communication.

### Creating a شائع کنندہ

```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'my_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher.publish(msg)
```

### Creating a  مسیحین

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
        self.get_logger().info(f'Received: {msg.data}')
```

### سروس کی معیار (QoS)

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

self.publisher = self.create_publisher(String, 'topic', qos_profile)
```

##  سروسز: Request-Response

 سروسز provide **synchronous** communication for request-response patterns.

### Defining a Service

```python
# Service definition (srv/AddTwoInts.srv)
# int64 a
# int64 b
# ---
# int64 sum
```

### Service Server

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

### Service Client

```python
class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        return future
```

## When to استعمال کریں What?

| Pattern | استعمال کریں Case | Example |
|---------|----------|---------|
| ٹاپکس | Continuous data streams | Sensor readings, camera images |
|  سروسز | One-time requests | Spawn robot, save map |
| ایکشنز | Long-running tasks | Navigate to goal, pick object |

## ہاتھ سے کام کرنے والی لیب

### Lab 1.2: تعمیر کریں a Temperature Monitor

تخلیق کریں a system with:
1. A sensor node publishing temperature readings
2. A monitor node subscribing and alerting on high temps
3. A service to get the current temperature on demand

```bash
# چلائیں your solution
ros2 run my_package temperature_sensor
ros2 run my_package temperature_monitor
ros2 service call /get_temperature ...
```

## نالej چیک

1. What communication pattern should you use for continuous sensor data?
   - [x] ٹاپکس
   - [ ]  سروسز
   - [ ] ایکشنز

2.  سروسز are _____ while topics are _____.
   - [x] Synchronous, Asynchronous
   - [ ] Asynchronous, Synchronous
   - [ ] Both synchronous

## خلاصہ

- **نوڈز** are independent processes performing computation
- **ٹاپکس** use publish-subscribe for streaming data
- ** سروسز** use request-response for one-time operations
- **QoS** profiles control reliability and delivery guarantees

## اگلے اقدامات

[Continue to باب 1.3: ایکشنز اور پیرامیٹر →](/ur/docs/module-1-ros2/chapter-3-actions-params)


