---
sidebar_position: 2
title: کوئیک اسٹارٹ گائیڈ
description: منٹوں میں فزیکل اے آئی کے ساتھ شروع کریں
---

# کوئیک اسٹارٹ گائیڈ

ہماری ون-کمانڈ سیٹ اپ کے ساتھ منٹوں میں اپنا ڈیولپمنٹ ماحول تیار کریں۔

## سسٹم کی ضروریات

| جزو | کم از کم | تجویز کردہ |
|-----------|---------|-------------|
| او ایس | اوبنٹو 22.04 | اوبنٹو 22.04 |
| ریم | 8 جی بی | 16+ جی بی |
| اسٹوریج | 50 جی بی | 100+ جی بی |
| جی پی یو | - | این ویڈیا آر ٹی ایکس 3060+ |

## فوری انسٹالیشن

### آپشن 1: ڈاکر (تجویز کردہ)

```bash
# ریپوزٹری کلون کریں
git clone https://github.com/physicalai/robotics-workspace.git
cd robotics-workspace

# ڈیولپمنٹ ماحول شروع کریں
docker-compose up -d

# کنٹینر میں داخل ہوں
docker exec -it physicalai-dev bash
```

### آپشن 2: نیٹو انسٹالیشن

```bash
# آر او ایس 2 ہمبل انسٹال کریں
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop

# آر او ایس 2 سورس کریں
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## انسٹالیشن کی تصدیق کریں

```bash
# آر او ایس 2 ورژن چیک کریں
ros2 --version

# ایک سادہ ٹاکر-لسنر ڈیمو چلائیں
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener
```

آپ کو پیغامات پبلش اور وصول ہوتے دیکھنے چاہیئیں!

## سمولیشن ٹولز انسٹال کریں

### گزیبو

```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs

# گزیبو ٹیسٹ کریں
gazebo
```

### آئزک سم (جی پی یو ضروری)

1. [این ویڈیا اومنیورس](https://developer.nvidia.com/isaac-sim) سے ڈاؤن لوڈ کریں
2. اومنیورس لانچر استعمال کرکے انسٹال کریں
3. آئزک سم سیٹنگز میں آر او ایس 2 برج فعال کریں

## آپ کا پہلا روبوٹ

آئیے گزیبو میں ایک سادہ روبوٹ اسپان کریں:

```bash
# ورک سپیس بنائیں
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# ڈیمو روبوٹ کلون کریں
git clone https://github.com/ros/urdf_tutorial.git

# بلڈ کریں
cd ~/ros2_ws
colcon build
source install/setup.bash

# گزیبو میں روبوٹ لانچ کریں
ros2 launch urdf_tutorial display.launch.py
```

## اگلے اقدامات

اب جب آپ کا ماحول تیار ہے:

1. **آر او ایس 2 بنیادیات سیکھیں** → [ماڈیول 1: آر او ایس 2](/docs/module-1-ros2/chapter-1-intro)
2. **سمولیشنز بنائیں** → [ماڈیول 2: سمولیشن](/docs/module-2-simulation/chapter-1-intro)
3. **اے آئی صلاحیتیں شامل کریں** → [ماڈیول 3: آئزک](/docs/module-3-isaac/chapter-1-intro)
4. **وی ایل اے مربوط کریں** → [ماڈیول 4: وی ایل اے](/docs/module-4-vla/chapter-1-intro)

:::tip مدد چاہیے؟
سیٹ اپ یا ٹربل شوٹنگ کے بارے میں سوالات پوچھنے کے لیے نیچے دائیں کونے میں ہمارا اے آئی اسسٹنٹ استعمال کریں!
:::
