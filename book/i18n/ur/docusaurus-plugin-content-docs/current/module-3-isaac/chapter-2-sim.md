---
sidebar_position: 2
title: "3.2 ایزیک سیم Fundamentals"
description: "Deep dive into ایزیک سیم capabilities"
keywords: ["ایزیک سیم", "اومنی ورس", "simulation", "USD"]
---

# باب 3.2: ایزیک سیم Fundamentals

## سیکھنے کے مقاصد

- Navigate the ایزیک سیم interface
- تخلیق کریں and manipulate scenes
- Work with USD assets
- Configure physics settings

## Scene Creation

### Adding Objects

```python
from omni.isaac.core.utils.prims import create_prim
from pxr import UsdGeom, Gf

# تخلیق کریں a cube
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

## طبیعات Configuration

### PhysX Settings

```python
from omni.physx import get_physx_scene_query_interface

physx_scene = get_physx_scene_query_interface()
physx_scene.set_gravity(Gf.Vec3f(0, 0, -9.81))
```

### Rigid Body Setup

```python
from pxr import UsdPhysics

rigid_body_api = UsdPhysics.RigidBodyAPI.اپلائی کریں(prim)
mass_api = UsdPhysics.MassAPI.اپلائی کریں(prim)
mass_api.CreateMassAttr(10.0)
```

##  ڈومین رینڈمائزیشن

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

### فعال کریں ROS 2 Bridge

```python
import omni.graph.core as og
from omni.isaac.ros2_bridge import ROSBridge

# تخلیق کریں ROS 2 bridge
og.Controller.edit(
    {"graph_path": "/ROS2Bridge"},
    {og.Controller.Keys.CREATE_NODES: [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
    ]}
)
```

## ہاتھ سے کام کرنے والی لیب

### Lab 3.2: تعمیر کریں a  تربیت ماحول

تخلیق کریں a scene with:
- Random object placement
- Variable lighting
- Multiple camera views

## خلاصہ

- ایزیک سیم uses USD for scene composition
- PhysX provides accurate physics simulation
- Domain randomization improves AI training

[Continue to باب 3.3 →](/docs/module-3-isaac/chapter-3-perception)


