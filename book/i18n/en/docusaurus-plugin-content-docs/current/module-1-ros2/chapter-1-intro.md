---
sidebar_position: 1
title: "1.1 Introduction to ROS 2"
description: Understanding the Robot Operating System 2 architecture and philosophy
keywords: [ROS 2, robotics, middleware, DDS]
---

# Chapter 1.1: Introduction to ROS 2

## Learning Objectives

By the end of this chapter, you will be able to:

- Explain what ROS 2 is and why it's essential for robotics
- Describe the key differences between ROS 1 and ROS 2
- Understand the DDS middleware architecture
- Set up a ROS 2 workspace
- Run your first ROS 2 nodes

## Prerequisites

Before starting this chapter, ensure you have:

- [ ] Ubuntu 22.04 installed (or Windows WSL2)
- [ ] Basic Python programming knowledge
- [ ] Terminal/command line familiarity

## What is ROS 2?

**ROS 2** (Robot Operating System 2) is not an actual operating system, but rather a flexible framework for writing robot software. It provides:

- **Communication infrastructure** between processes
- **Hardware abstraction** for sensors and actuators
- **Tools** for visualization, debugging, and simulation
- **Libraries** for common robotics tasks

### The Evolution from ROS 1

ROS 1 served the robotics community well for over a decade, but had limitations:

| Aspect | ROS 1 | ROS 2 |
|--------|-------|-------|
| Real-time | Limited | Supported |
| Security | None | Built-in |
| Platforms | Linux only | Linux, Windows, macOS |
| Middleware | Custom | DDS standard |
| Multi-robot | Difficult | Native support |

## DDS: The Communication Backbone

ROS 2 uses the **Data Distribution Service (DDS)** as its middleware. DDS provides:

```
┌─────────────────────────────────────────────────────────┐
│                    ROS 2 Application                     │
├─────────────────────────────────────────────────────────┤
│                    RCL (ROS Client Library)              │
├─────────────────────────────────────────────────────────┤
│                    RMW (ROS Middleware)                  │
├─────────────────────────────────────────────────────────┤
│              DDS Implementation (FastDDS, etc.)          │
├─────────────────────────────────────────────────────────┤
│                    Network (UDP/TCP)                     │
└─────────────────────────────────────────────────────────┘
```

### Key DDS Concepts

1. **Automatic Discovery** - Nodes find each other without a central master
2. **Quality of Service (QoS)** - Configure reliability, durability, deadlines
3. **Type Safety** - Message types are strictly defined

## Setting Up Your Workspace

### Create a ROS 2 Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build (even empty)
colcon build

# Source the workspace
source install/setup.bash
```

### Workspace Structure

```
ros2_ws/
├── build/          # Build artifacts
├── install/        # Installed packages
├── log/            # Build logs
└── src/            # Source code
    └── my_package/
        ├── package.xml
        ├── setup.py
        ├── my_package/
        │   └── __init__.py
        └── resource/
```

## Your First ROS 2 Program

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

### Run the Publisher

```bash
# In terminal 1
python3 my_publisher.py

# In terminal 2 - see the messages
ros2 topic echo /topic
```

## ROS 2 Command Line Tools

Essential commands you'll use daily:

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# See topic info
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

## Hands-on Lab

### Lab 1.1: Create a Talker-Listener System

**Objective**: Create two nodes that communicate via a topic.

**Steps**:

1. Create a publisher node that sends sensor data
2. Create a subscriber node that receives and processes the data
3. Run both nodes and verify communication

**Expected Output**:
```
[Publisher] Publishing: temperature=23.5
[Subscriber] Received: temperature=23.5
```

:::tip Lab Solution
The complete solution is available in the [GitHub repository](https://github.com/physicalai/textbook/tree/main/labs/module-1/lab-1-1).
:::

## Knowledge Check

### Quiz

1. What middleware does ROS 2 use for communication?
   - [ ] ZeroMQ
   - [x] DDS (Data Distribution Service)
   - [ ] MQTT
   - [ ] Custom TCP/UDP

2. Which of the following is NOT a benefit of ROS 2 over ROS 1?
   - [ ] Real-time support
   - [ ] Multi-platform support
   - [x] Simpler to learn
   - [ ] Built-in security

3. What command lists all running nodes?
   - [ ] `ros2 list nodes`
   - [x] `ros2 node list`
   - [ ] `ros2 nodes`
   - [ ] `ros2 show nodes`

## Summary

In this chapter, you learned:

- ROS 2 is a flexible framework for robot software development
- DDS provides the communication backbone with automatic discovery
- Workspaces organize your ROS 2 packages
- Basic ROS 2 CLI tools for debugging and monitoring

## Further Reading

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [DDS Specification](https://www.omg.org/spec/DDS/)
- [ROS 2 Design](https://design.ros2.org/)

## Next Steps

In the next chapter, we'll dive deeper into **Nodes, Topics, and Services** - the building blocks of ROS 2 applications.

[Continue to Chapter 1.2 →](/ur/docs/module-1-ros2/chapter-2-nodes-topics)
