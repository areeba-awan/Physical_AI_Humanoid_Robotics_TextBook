---
sidebar_position: 1
title: "1.1 ROS 2 کا تعارف"
description: "روبوٹ آپریٹنگ سسٹم 2 کے معماری اور فلسفے کو سمجھنا"
keywords: ["ROS 2", "robotics", "middleware", "DDS"]
---

# باب 1.1: ROS 2 کا تعارف

## سیکھنے کے مقاصد

By the end of this chapter, you will be able to:

- Explain what ROS 2 is and why it's essential for robotics
- Describe the key differences between ROS 1 and ROS 2
- سمجھیں the DDS middleware architecture
-  تیار کریں a ROS 2 workspace
- چلائیں your first ROS 2 nodes

## شرائطِ لازمہ

Before starting this chapter, ensure you have:

- [ ] Ubuntu 22.04 installed (or Windows WSL2)
- [ ] Basic Python programming knowledge
- [ ] Terminal/command line familiarity

## کیا ہے ROS 2?

**ROS 2** (روبوٹ آپریٹنگ سسٹم 2) is not an actual operating system, but rather a flexible framework for writing robot software. It provides:

- **Communication infrastructure** between processes
- **ہارڈ ویئر abstraction** for sensors and actuators
- **اوزار** for visualization, debugging, and simulation
- ** لائبریری** for common robotics tasks

### The Evolution from ROS 1

ROS 1 served the robotics community well for over a decade, but had limitations:

| Aspect | ROS 1 | ROS 2 |
|--------|-------|-------|
| ریل ٹائم | Limited | Supported |
| سیکیورٹی | None | Built-in |
| پلیٹ فارم | Linux only | Linux, Windows, macOS |
| مڈل ویئر | Custom | DDS standard |
| متعدد روبوٹ | Difficult | Native support |

## DDS: The Communication Backbone

ROS 2 uses the **ڈیٹا تقسیم کی سروس (DDS)** as its middleware. DDS provides:

```
┌─────────────────────────────────────────────────────────┐
│                    ROS 2 Application                     │
├─────────────────────────────────────────────────────────┤
│                    RCL (ROS کلائنٹ لائبریری)              │
├─────────────────────────────────────────────────────────┤
│                    RMW (ROS مڈل ویئر)                  │
├─────────────────────────────────────────────────────────┤
│              DDS Implementation (FastDDS, etc.)          │
├─────────────────────────────────────────────────────────┤
│                    Network (UDP/TCP)                     │
└─────────────────────────────────────────────────────────┘
```

### Key DDS Concepts

1. **خودکار دریافت** - نوڈز find each other without a central master
2. **سروس کی معیار (QoS)** - Configure reliability, durability, deadlines
3. **ٹائپ سیفٹی** - Message types are strictly defined

## ماحول تیار کرنا Your  ورک سپیس

### تخلیق کریں a ROS 2  ورک سپیس

```bash
# تخلیق کریں workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# تعمیر کریں (even empty)
colcon build

# ماخذ the workspace
source install/setup.bash
```

###  ورک سپیس Structure

```
ros2_ws/
├── build/          # تعمیر کریں artifacts
├── install/        # Installed packages
├── log/            # تعمیر کریں logs
└── src/            # ماخذ code
    └── my_package/
        ├── package.xml
        ├── setup.py
        ├── my_package/
        │   └── __init__.py
        └── resource/
```

## آپ کا پہلا ROS 2 پروگرام

Let's create a simple publisher:

```python
# my_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### چلائیں the شائع کنندہ

```bash
# In terminal 1
python3 my_publisher.py

# In terminal 2 - see the messages
ros2 topic echo /topic
```

## ROS 2 کمانڈ لائن اوزار

ضروری کمانڈز جو آپ روزانہ استعمال کریں گے:

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# دیکھیں topic info
ros2 topic info /topic

# Echo topic messages
ros2 topic echo /topic

# Publish to a topic
ros2 topic pub /topic std_msgs/String "data: 'Hello'"

# List services
ros2 service list

# Call a service
ros2 service call /service_name std_srvs/srv/Empty
```

## ہاتھ سے کام کرنے والی لیب

### Lab 1.1: تخلیق کریں a Talker-Listener سسٹم

**Objective**: تخلیق کریں two nodes that communicate via a topic.

**Steps**:

1. تخلیق کریں a publisher node that sends sensor data
2. تخلیق کریں a subscriber node that receives and processes the data
3. چلائیں both nodes and verify communication

**Expected Output**:
```
[شائع کنندہ] Publishing: temperature=23.5
[ مسیحین] Received: temperature=23.5
```

:::tip Lab Solution
The complete solution is available in the [گیتھب repository](https://github.com/physicalai/textbook/tree/main/labs/module-1/lab-1-1).
:::

## نالej چیک

### Quiz

1. What middleware does ROS 2 use for communication?
   - [ ] ZeroMQ
   - [x] DDS (ڈیٹا تقسیم کی سروس)
   - [ ] MQTT
   - [ ] Custom TCP/UDP

2. Which of the following is NOT a benefit of ROS 2 over ROS 1?
   - [ ] ریل ٹائم support
   - [ ] Multi-platform support
   - [x] Simpler to learn
   - [ ] Built-in security

3. What command lists all running nodes?
   - [ ] `ros2 list nodes`
   - [x] `ros2 node list`
   - [ ] `ros2 nodes`
   - [ ] `ros2 show nodes`

## خلاصہ

In this chapter, you learned:

- ROS 2 is a flexible framework for robot software development
- DDS provides the communication backbone with automatic discovery
- Workspaces organize your ROS 2 packages
- Basic ROS 2 CLI tools for debugging and monitoring

## مزید پڑھائی

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [DDS Specification](https://www.omg.org/spec/DDS/)
- [ROS 2  ڈیزائن](https://design.ros2.org/)

## اگلے اقدامات

In the next chapter, we'll dive deeper into **نوڈز، ٹاپکس، اور سروسز** - the building blocks of ROS 2 applications.

[Continue to باب 1.2 →](/ur/docs/module-1-ros2/chapter-2-nodes-topics)

