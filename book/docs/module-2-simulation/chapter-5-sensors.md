---
sidebar_position: 5
title: "2.5 Sensor Simulation"
description: Simulating cameras, LiDAR, and other sensors
keywords: [sensors, LiDAR, camera, IMU, simulation]
---

# Chapter 2.5: Sensor Simulation

## Learning Objectives

- Simulate camera sensors with realistic noise
- Configure LiDAR sensors
- Add IMU and other proprioceptive sensors
- Apply sensor noise models

## Camera Simulation

### RGB Camera in Gazebo

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

### Depth Camera (RGBD)

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

## LiDAR Simulation

### 2D LiDAR

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

### 3D LiDAR

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

## IMU Simulation

```xml
<sensor name="imu" type="imu">
  <imu>
    <angular_velocity>
      <x><noise type="gaussian"><mean>0</mean><stddev>0.001</stddev></noise></x>
      <y><noise type="gaussian"><mean>0</mean><stddev>0.001</stddev></noise></y>
      <z><noise type="gaussian"><mean>0</mean><stddev>0.001</stddev></noise></z>
    </angular_velocity>
    <linear_acceleration>
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

## Sensor Noise Models

| Sensor | Noise Type | Typical Values |
|--------|------------|----------------|
| Camera | Gaussian | σ = 0.01 |
| LiDAR | Gaussian | σ = 0.01m |
| IMU Gyro | Gaussian + Bias | σ = 0.001 rad/s |
| IMU Accel | Gaussian + Bias | σ = 0.01 m/s² |

## Hands-on Lab

### Lab 2.5: Multi-Sensor Robot

Add to your robot:
- Front RGB camera
- Depth camera
- 2D LiDAR
- IMU

Verify data in RViz.

## Summary

- Sensors bridge simulation and perception algorithms
- Noise models ensure realistic sensor behavior
- Multiple sensors provide redundancy and coverage

[Continue to Chapter 2.6: Lab →](/docs/module-2-simulation/chapter-6-lab)


