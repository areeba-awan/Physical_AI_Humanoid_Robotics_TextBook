---
sidebar_position: 2
title: "2.2 Gazebo Basics"
description: Getting started with Gazebo simulation
keywords: [Gazebo, simulation, physics, ROS 2]
---

# Chapter 2.2: Gazebo Basics

## Learning Objectives

- Navigate the Gazebo interface
- Create and manipulate world files
- Add models and interact with physics
- Connect Gazebo to ROS 2

## Gazebo Interface

### Main Components

- **Scene** - 3D visualization of the world
- **Panel** - World tree, properties, layers
- **Toolbar** - Navigation, insertion, manipulation tools
- **Timeline** - Simulation time control

## Creating Worlds

### World File Structure

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

## ROS 2 Integration

### Gazebo Plugins

```xml
<plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
  <robotNamespace>/my_robot</robotNamespace>
</plugin>
```

### Spawning from ROS 2

```bash
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file robot.urdf
```

## Physics Configuration

```xml
<physics type="ode">
  <real_time_update_rate>1000</real_time_update_rate>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
</physics>
```

## Hands-on Lab

### Lab 2.2: Create a Custom World

Build a warehouse environment with:
- Shelving units
- Floor markings
- Proper lighting

## Summary

- Gazebo provides realistic physics simulation
- World files define environments in SDF format
- ROS 2 plugins enable seamless integration

[Continue to Chapter 2.3 →](/docs/module-2-simulation/chapter-3-urdf)


