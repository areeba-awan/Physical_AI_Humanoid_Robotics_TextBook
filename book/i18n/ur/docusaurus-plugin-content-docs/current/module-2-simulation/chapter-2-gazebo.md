---
sidebar_position: 2
title: "2.2 گیزیبو Basics"
description: "Getting started with گیزیبو simulation"
keywords: ["گیزیبو", "simulation", "physics", "ROS 2"]
---

# باب 2.2: گیزیبو Basics

## سیکھنے کے مقاصد

- Navigate the گیزیبو interface
- تخلیق کریں and manipulate world files
- شامل کریں models and interact with physics
- Connect گیزیبو کا تعارف

## گیزیبو Interface

### Main Components

- **Scene** - 3D visualization of the world
- **Panel** - دنیا tree, properties, layers
- **Toolbar** - نیویگیشن, insertion, manipulation tools
- **Timeline** - سیمیولیشن time control

## Creating Worlds

### دنیا فائل Structure

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="robot_world">
    <!-- Lighting -->
    <include><uri>model://sun</uri></include>

    <!-- Ground -->
    <include><uri>model://ground_plane</uri></include>

    <!-- Custom models -->
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

## ROS 2  انضمام

### گیزیبو Plugins

```xml
<plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
  <robotNamespace>/my_robot</robotNamespace>
</plugin>
```

### Spawning from ROS 2

```bash
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file robot.urdf
```

## طبیعات Configuration

```xml
<physics type="ode">
  <real_time_update_rate>1000</real_time_update_rate>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
</physics>
```

## ہاتھ سے کام کرنے والی لیب

### Lab 2.2: تخلیق کریں a Custom دنیا

تعمیر کریں a warehouse environment with:
- Shelving units
- Floor markings
- Proper lighting

## خلاصہ

- گیزیبو provides realistic physics simulation
- دنیا files define environments in SDF format
- ROS 2 plugins enable seamless integration

[Continue to باب 2.3 →](/docs/module-2-simulation/chapter-3-urdf)


