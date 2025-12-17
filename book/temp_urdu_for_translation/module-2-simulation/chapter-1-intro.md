---
sidebar_position: 1
title: "2.1 Introduction to Simulation"
description: Why simulation is essential for robotics development
keywords: [simulation, digital twin, Gazebo, Unity, robotics]
---

# Chapter 2.1: Introduction to Simulation

## Learning Objectives

- Understand the role of simulation in robotics development
- Compare different simulation platforms
- Learn the digital twin concept
- Set up your simulation environment

## Why Simulate?

Simulation is essential for modern robotics because:

1. **Safety** - Test dangerous scenarios without risk
2. **Speed** - Run experiments faster than real-time
3. **Cost** - No hardware wear or damage
4. **Scale** - Test thousands of scenarios
5. **Reproducibility** - Exact same conditions every time

### The Digital Twin Concept

```
┌──────────────────────┐         ┌──────────────────────┐
│     REAL WORLD       │         │    DIGITAL TWIN      │
│                      │         │                      │
│  ┌──────────────┐   │  Sync   │   ┌──────────────┐   │
│  │ Physical     │◄──┼─────────┼──►│ Simulated    │   │
│  │ Robot        │   │         │   │ Robot        │   │
│  └──────────────┘   │         │   └──────────────┘   │
│                      │         │                      │
│  ┌──────────────┐   │         │   ┌──────────────┐   │
│  │ Environment  │◄──┼─────────┼──►│ Virtual      │   │
│  │              │   │         │   │ Environment  │   │
│  └──────────────┘   │         │   └──────────────┘   │
└──────────────────────┘         └──────────────────────┘
```

## Simulation Platforms

| Platform | Strengths | Use Cases |
|----------|-----------|-----------|
| **Gazebo** | ROS integration, physics | General robotics |
| **Unity** | Graphics, ML-Agents | Vision, perception |
| **Isaac Sim** | GPU physics, synthetic data | AI training |
| **Webots** | Easy to use, tutorials | Education |

### Gazebo (Open Source)

```bash
# Install Gazebo with ROS 2
sudo apt install ros-humble-gazebo-ros-pkgs

# Launch empty world
gazebo
```

### Unity Robotics Hub

- Photorealistic rendering
- ML-Agents integration
- Cross-platform export

### NVIDIA Isaac Sim

- RTX ray tracing
- Domain randomization
- Synthetic data generation

## Setting Up Gazebo

### Basic World File

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="my_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
  </world>
</sdf>
```

### Launch Gazebo with ROS 2

```python
# launch/gazebo.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={'world': 'my_world.sdf'}.items()
    )
    return LaunchDescription([gazebo])
```

## Simulation Fidelity

### Physics Accuracy

| Level | Speed | Accuracy | Use Case |
|-------|-------|----------|----------|
| Low | 100x RT | ~50% | Quick tests |
| Medium | 10x RT | ~80% | Development |
| High | 1x RT | ~95% | Validation |

## Hands-on Lab

### Lab 2.1: Launch Your First Simulation

1. Install Gazebo
2. Launch the empty world
3. Add a simple shape (box, sphere)
4. Apply forces and observe physics

## Summary

- Simulation enables safe, fast, and cheap robot development
- Digital twins mirror physical robots in virtual environments
- Choose your platform based on your needs

[Continue to Chapter 2.2: Gazebo Basics →](/ur/docs/module-2-simulation/chapter-2-gazebo)
