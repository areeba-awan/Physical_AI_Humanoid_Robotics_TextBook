---
sidebar_position: 2
title: Quickstart Guide
description: Get up and running with Physical AI in minutes
---

# Quickstart Guide

Get your development environment ready in minutes with our one-command setup.

## System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| OS | Ubuntu 22.04 | Ubuntu 22.04 |
| RAM | 8 GB | 16+ GB |
| Storage | 50 GB | 100+ GB |
| GPU | - | NVIDIA RTX 3060+ |

## Quick Install

### Option 1: Docker (Recommended)

```bash
# Clone the repository
git clone https://github.com/physicalai/robotics-workspace.git
cd robotics-workspace

# Start the development environment
docker-compose up -d

# Enter the container
docker exec -it physicalai-dev bash
```

### Option 2: Native Installation

```bash
# Install ROS 2 Humble
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop

# Source ROS 2
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Verify Installation

```bash
# Check ROS 2 version
ros2 --version

# Run a simple talker-listener demo
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener
```

You should see messages being published and received!

## Install Simulation Tools

### Gazebo

```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs

# Test Gazebo
gazebo
```

### Isaac Sim (GPU Required)

1. Download from [NVIDIA Omniverse](https://developer.nvidia.com/isaac-sim)
2. Install using the Omniverse Launcher
3. Enable ROS 2 bridge in Isaac Sim settings

## Your First Robot

Let's spawn a simple robot in Gazebo:

```bash
# Create a workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone a demo robot
git clone https://github.com/ros/urdf_tutorial.git

# Build
cd ~/ros2_ws
colcon build
source install/setup.bash

# Launch the robot in Gazebo
ros2 launch urdf_tutorial display.launch.py
```

## Next Steps

Now that your environment is ready:

1. **Learn ROS 2 basics** → [Module 1: ROS 2](/docs/module-1-ros2/chapter-1-intro)
2. **Build simulations** → [Module 2: Simulation](/docs/module-2-simulation/chapter-1-intro)
3. **Add AI capabilities** → [Module 3: Isaac](/docs/module-3-isaac/chapter-1-intro)
4. **Integrate VLA** → [Module 4: VLA](/docs/module-4-vla/chapter-1-intro)

:::tip Need Help?
Use our AI assistant in the bottom-right corner to ask questions about setup or troubleshooting!
:::

