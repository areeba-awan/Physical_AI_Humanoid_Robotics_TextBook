---
sidebar_position: 5
title: "2.5 سینسر سمولیشن"
description: کیمرے، لائیڈار، اور دیگر سینسرز کی سمولیشن
keywords: [سینسرز, لائیڈار, کیمرہ, IMU, سمولیشن]
---

# باب 2.5: سینسر سمولیشن

## سیکھنے کے مقاصد

- حقیقت پسندانہ شور کے ساتھ کیمرہ سینسرز کی سمولیشن کریں
- لائیڈار سینسرز ترتیب دیں
- IMU اور دیگر پروپریوسیپٹیو سینسرز شامل کریں
- سینسر نوائز ماڈلز لاگو کریں

## کیمرہ سمولیشن

### گیزیبو میں RGB کیمرہ

```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>image_raw:=camera/image</remapping>
    </ros>
    <camera_name>front_camera</camera_name>
  </plugin>
  <update_rate>30</update_rate>
</sensor>
```

### ڈیپتھ کیمرہ (RGBD)

```xml
<sensor name="depth_camera" type="depth">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
  </camera>
  <plugin name="depth_camera" filename="libgazebo_ros_camera.so">
    <ros><namespace>/robot</namespace></ros>
  </plugin>
</sensor>
```

## لائیڈار سمولیشن

### 2D لائیڈار

```xml
<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <!-- گاؤسین شور -->
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </ray>
  <plugin name="lidar" filename="libgazebo_ros_ray_sensor.so">
    <ros><namespace>/robot</namespace></ros>
    <output_type>sensor_msgs/LaserScan</output_type>
  </plugin>
</sensor>
```

### 3D لائیڈار

```xml
<sensor name="lidar_3d" type="gpu_lidar">
  <ray>
    <scan>
      <horizontal>
        <samples>1800</samples>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <min_angle>-0.26</min_angle>
        <max_angle>0.26</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.5</min>
      <max>100</max>
    </range>
  </ray>
  <plugin name="lidar_3d" filename="libgazebo_ros_ray_sensor.so">
    <output_type>sensor_msgs/PointCloud2</output_type>
  </plugin>
</sensor>
```

## IMU سمولیشن

```xml
<sensor name="imu" type="imu">
  <imu>
    <angular_velocity>
      <!-- زاویائی ویلاسٹی کا شور -->
      <x><noise type="gaussian"><mean>0</mean><stddev>0.001</stddev></noise></x>
      <y><noise type="gaussian"><mean>0</mean><stddev>0.001</stddev></noise></y>
      <z><noise type="gaussian"><mean>0</mean><stddev>0.001</stddev></noise></z>
    </angular_velocity>
    <linear_acceleration>
      <!-- خطی ایکسلریشن کا شور -->
      <x><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></x>
      <y><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></y>
      <z><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></z>
    </linear_acceleration>
  </imu>
  <plugin name="imu" filename="libgazebo_ros_imu_sensor.so">
    <ros><namespace>/robot</namespace></ros>
  </plugin>
</sensor>
```

## سینسر نوائز ماڈلز

| سینسر | شور کی قسم | عام اقدار |
|--------|------------|----------------|
| کیمرہ | گاؤسین | σ = 0.01 |
| لائیڈار | گاؤسین | σ = 0.01m |
| IMU گائرو | گاؤسین + بائیس | σ = 0.001 rad/s |
| IMU ایکسل | گاؤسین + بائیس | σ = 0.01 m/s² |

## عملی لیب

### لیب 2.5: ملٹی سینسر روبوٹ

اپنے روبوٹ میں شامل کریں:
- سامنے والا RGB کیمرہ
- ڈیپتھ کیمرہ
- 2D لائیڈار
- IMU

RViz میں ڈیٹا کی تصدیق کریں۔

## خلاصہ

- سینسرز سمولیشن اور پرسیپشن الگورتھمز کے درمیان پل ہیں
- نوائز ماڈلز حقیقت پسندانہ سینسر رویے کو یقینی بناتے ہیں
- متعدد سینسرز ریڈنڈنسی اور کوریج فراہم کرتے ہیں

[باب 2.6: لیب جاری رکھیں ←](/docs/module-2-simulation/chapter-6-lab)
