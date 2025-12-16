---
sidebar_position: 2
title: "2.2 گیزیبو کی بنیادیں"
description: گیزیبو سمولیشن کے ساتھ شروعات
keywords: [گیزیبو, سمولیشن, فزکس, ROS 2]
---

# باب 2.2: گیزیبو کی بنیادیں

## سیکھنے کے مقاصد

- گیزیبو انٹرفیس کو نیویگیٹ کریں
- ورلڈ فائلز بنائیں اور ان میں تبدیلی کریں
- ماڈلز شامل کریں اور فزکس کے ساتھ تعامل کریں
- گیزیبو کو ROS 2 سے جوڑیں

## گیزیبو انٹرفیس

### اہم اجزاء

- **سین** - دنیا کا 3D ویژولائزیشن
- **پینل** - ورلڈ ٹری، پراپرٹیز، لیئرز
- **ٹول بار** - نیویگیشن، اضافہ، ہیرا پھیری کے ٹولز
- **ٹائم لائن** - سمولیشن وقت کا کنٹرول

## دنیائیں بنانا

### ورلڈ فائل کی ساخت

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="robot_world">
    <!-- روشنی -->
    <include><uri>model://sun</uri></include>

    <!-- زمین -->
    <include><uri>model://ground_plane</uri></include>

    <!-- اپنی مرضی کے ماڈلز -->
    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>1 1 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1 1 1</size></box></geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## ROS 2 انٹیگریشن

### گیزیبو پلگ انز

```xml
<plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
  <robotNamespace>/my_robot</robotNamespace>
</plugin>
```

### ROS 2 سے اسپان کرنا

```bash
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file robot.urdf
```

## فزکس کنفیگریشن

```xml
<physics type="ode">
  <real_time_update_rate>1000</real_time_update_rate>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
</physics>
```

## عملی لیب

### لیب 2.2: اپنی مرضی کی دنیا بنائیں

گودام کا ماحول بنائیں جس میں:
- شیلفنگ یونٹس
- فرش کے نشانات
- مناسب روشنی

## خلاصہ

- گیزیبو حقیقت پسندانہ فزکس سمولیشن فراہم کرتا ہے
- ورلڈ فائلز SDF فارمیٹ میں ماحول کی تعریف کرتی ہیں
- ROS 2 پلگ انز بغیر کسی رکاوٹ کے انٹیگریشن کو ممکن بناتے ہیں

[باب 2.3 جاری رکھیں ←](/docs/module-2-simulation/chapter-3-urdf)
