---
sidebar_position: 2
title: "3.2 Isaac Sim Fundamentals"
description: Deep dive into Isaac Sim capabilities
keywords: [Isaac Sim, Omniverse, simulation, USD]
---

# Chapter 3.2: Isaac Sim Fundamentals

## Learning Objectives

- Navigate the Isaac Sim interface
- Create and manipulate scenes
- Work with USD assets
- Configure physics settings

## Scene Creation

### Adding Objects

```python
from omni.isaac.core.utils.prims import create_prim
from pxr import UsdGeom, Gf

# Create a cube
cube_prim = create_prim(
    prim_path="/World/Cube",
    prim_type="Cube",
    position=Gf.Vec3d(0, 0, 0.5),
    scale=Gf.Vec3d(1, 1, 1)
)
```

### Loading USD Assets

```python
from omni.isaac.core.utils.stage import add_reference_to_stage

# Load robot from USD
add_reference_to_stage(
    usd_path="/path/to/robot.usd",
    prim_path="/World/Robot"
)
```

## Physics Configuration

### PhysX Settings

```python
from omni.physx import get_physx_scene_query_interface

physx_scene = get_physx_scene_query_interface()
physx_scene.set_gravity(Gf.Vec3f(0, 0, -9.81))
```

### Rigid Body Setup

```python
from pxr import UsdPhysics

rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(prim)
mass_api = UsdPhysics.MassAPI.Apply(prim)
mass_api.CreateMassAttr(10.0)
```

## Domain Randomization

```python
from omni.isaac.core.utils.random import random_position

for i in range(100):
    # Randomize object positions
    pos = random_position((-5, -5, 0), (5, 5, 2))

    # Randomize lighting
    light.GetIntensityAttr().Set(random.uniform(500, 2000))

    # Randomize textures
    material.GetDiffuseColorAttr().Set(random_color())
```

## ROS 2 Bridge

### Enable ROS 2 Bridge

```python
import omni.graph.core as og
from omni.isaac.ros2_bridge import ROSBridge

# Create ROS 2 bridge
og.Controller.edit(
    {"graph_path": "/ROS2Bridge"},
    {og.Controller.Keys.CREATE_NODES: [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
    ]}
)
```

## Hands-on Lab

### Lab 3.2: Build a Training Environment

Create a scene with:
- Random object placement
- Variable lighting
- Multiple camera views

## Summary

- Isaac Sim uses USD for scene composition
- PhysX provides accurate physics simulation
- Domain randomization improves AI training

[Continue to Chapter 3.3 →](/docs/module-3-isaac/chapter-3-perception)


