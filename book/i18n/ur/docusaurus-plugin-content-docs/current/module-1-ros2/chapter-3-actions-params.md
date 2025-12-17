---
sidebar_position: 3
title: "1.3 ایکشنز اور پیرامیٹر"
description: "Long-running tasks and runtime configuration in ROS 2"
keywords: ["ROS 2", "actions", "parameters", "goal", "feedback"]
---

# باب 1.3: ایکشنز اور پیرامیٹر

## سیکھنے کے مقاصد

- Implement ROS 2 actions for long-running tasks
- استعمال کریں parameters for runtime configuration
- Handle action goals, feedback, and results

## ایکشنز: Long-Running Tasks

ایکشنز are ideal for tasks that:
- Take a long time to complete
- Need to provide progress feedback
- Can be canceled mid-execution

### Action Structure

```
┌─────────────┐     Goal      ┌─────────────┐
│   Client    │──────────────>│   Server    │
│             │               │             │
│             │<──────────────│             │
│             │    Result     │             │
│             │               │             │
│             │<──────────────│             │
│             │   Feedback    │             │
└─────────────┘    (stream)   └─────────────┘
```

### Defining an Action

```
# action/Navigate.action
# Goal
float64 target_x
float64 target_y
---
# Result
bool success
float64 time_elapsed
---
# Feedback
float64 distance_remaining
float64 current_x
float64 current_y
```

### Action Server

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
        self.get_logger().info('Executing navigation...')

        feedback_msg = Navigate.Feedback()

        # Simulate navigation
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

### Action Client

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
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining}')
```

##  پیرامیٹر: Runtime Configuration

 پیرامیٹر allow you to configure nodes without recompiling.

### Declaring  پیرامیٹر

```python
class ParameterizedNode(Node):
    def __init__(self):
        super().__init__('parameterized_node')

        # Declare with default values
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('sensors', ['lidar', 'camera'])

    def get_config(self):
        name = self.get_parameter('robot_name').value
        speed = self.get_parameter('max_speed').value
        return name, speed
```

### Setting  پیرامیٹر from CLI

```bash
# Set at launch
ros2 run my_package my_node --ros-args -p robot_name:=my_robot -p max_speed:=2.0

# Set at runtime
ros2 param set /my_node robot_name "new_robot"

# Get parameter
ros2 param get /my_node robot_name

# List all parameters
ros2 param list /my_node
```

### Parameter Files (YAML)

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

## ہاتھ سے کام کرنے والی لیب

### Lab 1.3: Gripper Action Server

Implement an action server that:
1. Accepts a grip force goal
2. Sends feedback as gripper closes
3. Returns success/failure result

## نالej چیک

1. When should you use actions instead of services?
   - [x] For long-running tasks with feedback
   - [ ] For instant one-time requests
   - [ ] For streaming sensor data

## خلاصہ

- **ایکشنز** handle long-running tasks with goal, feedback, and result
- ** پیرامیٹر** provide runtime configuration without recompilation
- استعمال کریں YAML files for managing multiple parameters

## اگلے اقدامات

[Continue to باب 1.4: لانچ کریں Files →](/ur/docs/module-1-ros2/chapter-4-launch-files)


