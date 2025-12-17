---
sidebar_position: 3
title: "2.3 URDF and Robot Models"
description: Creating robot descriptions with URDF
keywords: [URDF, robot model, links, joints, xacro]
---

# Chapter 2.3: URDF and Robot Models

## Learning Objectives

- Understand URDF structure
- Create links and joints
- Add visual and collision geometries
- Use xacro for modular robot descriptions

## URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Wheel Joint -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel"/>
    <origin xyz="0.2 0.2 0" rpy="0 1.5708 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Wheel Link -->
  <link name="wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Joint Types

| Type | DOF | Description |
|------|-----|-------------|
| `fixed` | 0 | No movement |
| `revolute` | 1 | Rotation with limits |
| `continuous` | 1 | Unlimited rotation |
| `prismatic` | 1 | Linear motion |
| `floating` | 6 | Free movement |

## Using Xacro

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="my_robot">
  <xacro:property name="wheel_radius" value="0.1"/>

  <xacro:macro name="wheel" params="prefix side">
    <link name="${prefix}_${side}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="0.05"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <xacro:wheel prefix="front" side="left"/>
  <xacro:wheel prefix="front" side="right"/>
</robot>
```

## Visualizing in RViz

```bash
ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf
```

## Hands-on Lab

### Lab 2.3: Build a Mobile Robot

Create a 4-wheeled robot with:
- Differential drive base
- Lidar sensor mount
- Camera mount

## Summary

- URDF defines robot structure with links and joints
- Xacro enables modular, reusable descriptions
- Proper inertials are critical for physics

[Continue to Chapter 2.4 →](/ur/docs/module-2-simulation/chapter-4-unity)
