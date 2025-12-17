---
sidebar_position: 2
title: "فوراً شروع کریں گائیڈ"
description: "Get up and running with Physical AI in minutes"
---

# فوراً شروع کریں گائیڈ

Get your development environment ready in minutes with our one-command setup.

## سسٹم Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| OS | Ubuntu 22.04 | Ubuntu 22.04 |
| RAM | 8 GB | 16+ GB |
| Storage | 50 GB | 100+ GB |
| GPU | - | NVIDIA RTX 3060+ |

## Quick انسٹال کریں

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
# انسٹال کریں ROS 2 Humble
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop

# ماخذ ROS 2
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Verify Installation

```bash
# Check ROS 2 version
ros2 --version

# چلائیں a simple talker-listener demo
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener
```

You should see messages being published and received!

## انسٹال کریں سیمیولیشن اوزار

### گیزیبو

```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs

# جانچ گیزیبو
gazebo
```

### ایزیک سیم (GPU Required)

1. Download from [NVIDIA اومنی ورس](https://developer.nvidia.com/isaac-sim)
2. انسٹال کریں using the اومنی ورس Launcher
3. فعال کریں ROS 2 bridge in ایزیک سیم settings

## آپ کا پہلا Robot

Let's spawn a simple robot in گیزیبو:

```bash
# تخلیق کریں a workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone a demo robot
git clone https://github.com/ros/urdf_tutorial.git

# تعمیر کریں
cd ~/ros2_ws
colcon build
source install/setup.bash

# لانچ کریں the robot in گیزیبو
ros2 launch urdf_tutorial display.launch.py
```

## اگلے اقدامات

Now that your environment is ready:

1. **سیکھیں ROS 2 basics** → [مودیول 1: ROS 2](/docs/module-1-ros2/chapter-1-intro)
2. **تعمیر کریں simulations** → [مودیول 2: سیمیولیشن](/docs/module-2-simulation/chapter-1-intro)
3. **شامل کریں AI capabilities** → [مودیول 3: Isaac](/docs/module-3-isaac/chapter-1-intro)
4. **Integrate VLA** → [مودیول 4: VLA](/docs/module-4-vla/chapter-1-intro)

:::tip Need Help?
استعمال کریں our AI assistant in the bottom-right corner to ask questions about setup or troubleshooting!
:::

