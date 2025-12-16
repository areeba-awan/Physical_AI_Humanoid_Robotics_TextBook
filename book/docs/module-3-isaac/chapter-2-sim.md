---
sidebar_position: 2
title: "3.2 Isaac Sim کے بنیادی اصول"
description: Isaac Sim کی صلاحیتوں میں گہرائی سے مطالعہ
keywords: [Isaac Sim, Omniverse, سمولیشن, USD]
---

# باب 3.2: Isaac Sim کے بنیادی اصول

## سیکھنے کے مقاصد

- Isaac Sim انٹرفیس میں نیویگیٹ کریں
- سینز بنائیں اور تبدیل کریں
- USD اثاثوں کے ساتھ کام کریں
- فزکس سیٹنگز کنفیگر کریں

## سین کی تخلیق

### آبجیکٹس شامل کرنا

```python
from omni.isaac.core.utils.prims import create_prim
from pxr import UsdGeom, Gf

# ایک کیوب بنائیں
cube_prim = create_prim(
    prim_path="/World/Cube",
    prim_type="Cube",
    position=Gf.Vec3d(0, 0, 0.5),
    scale=Gf.Vec3d(1, 1, 1)
)
```

### USD اثاثے لوڈ کرنا

```python
from omni.isaac.core.utils.stage import add_reference_to_stage

# USD سے روبوٹ لوڈ کریں
add_reference_to_stage(
    usd_path="/path/to/robot.usd",
    prim_path="/World/Robot"
)
```

## فزکس کنفیگریشن

### PhysX سیٹنگز

```python
from omni.physx import get_physx_scene_query_interface

physx_scene = get_physx_scene_query_interface()
physx_scene.set_gravity(Gf.Vec3f(0, 0, -9.81))
```

### رجڈ باڈی سیٹ اپ

```python
from pxr import UsdPhysics

rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(prim)
mass_api = UsdPhysics.MassAPI.Apply(prim)
mass_api.CreateMassAttr(10.0)
```

## ڈومین رینڈمائزیشن

```python
from omni.isaac.core.utils.random import random_position

for i in range(100):
    # آبجیکٹ پوزیشنز کو رینڈمائز کریں
    pos = random_position((-5, -5, 0), (5, 5, 2))

    # روشنی کو رینڈمائز کریں
    light.GetIntensityAttr().Set(random.uniform(500, 2000))

    # ٹیکسچرز کو رینڈمائز کریں
    material.GetDiffuseColorAttr().Set(random_color())
```

## ROS 2 برج

### ROS 2 برج فعال کریں

```python
import omni.graph.core as og
from omni.isaac.ros2_bridge import ROSBridge

# ROS 2 برج بنائیں
og.Controller.edit(
    {"graph_path": "/ROS2Bridge"},
    {og.Controller.Keys.CREATE_NODES: [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
    ]}
)
```

## عملی لیب

### لیب 3.2: ٹریننگ ماحول بنائیں

مندرجہ ذیل کے ساتھ ایک سین بنائیں:
- رینڈم آبجیکٹ پلیسمنٹ
- متغیر روشنی
- متعدد کیمرہ ویوز

## خلاصہ

- Isaac Sim سین کمپوزیشن کے لیے USD استعمال کرتا ہے
- PhysX درست فزکس سمولیشن فراہم کرتا ہے
- ڈومین رینڈمائزیشن AI ٹریننگ کو بہتر بناتی ہے

[باب 3.3 پر جائیں ←](/docs/module-3-isaac/chapter-3-perception)
