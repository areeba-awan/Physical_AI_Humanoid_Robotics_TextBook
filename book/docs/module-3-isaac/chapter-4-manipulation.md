---
sidebar_position: 4
title: "3.4 مینیپولیشن پلاننگ"
description: Isaac کے ساتھ روبوٹ آرم موشن پلاننگ
keywords: [مینیپولیشن, موشن پلاننگ, cuMotion, MoveIt]
---

# باب 3.4: مینیپولیشن پلاننگ

## سیکھنے کے مقاصد

- موشن پلاننگ کے تصورات سمجھیں
- GPU سے تیز پلاننگ کے لیے cuMotion استعمال کریں
- MoveIt 2 کے ساتھ انٹیگریٹ کریں
- پک اینڈ پلیس آپریشنز نافذ کریں

## cuMotion: GPU موشن پلاننگ

### آرکیٹیکچر

```
┌─────────────────────────────────────────────────────────────┐
│                      cuMotion پائپ لائن                      │
│                                                              │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌────────┐│
│  │  ہدف     │───>│  گراف   │───>│ ٹکراؤ    │───>│ ہموار  ││
│  │  پوز    │    │  سرچ    │    │  چیک     │    │ ٹریج   ││
│  └──────────┘    └──────────┘    └──────────┘    └────────┘│
│                       │               │                      │
│                       └───────────────┘                      │
│                        GPU متوازی                           │
└─────────────────────────────────────────────────────────────┘
```

### cuMotion کا استعمال

```python
from isaac_ros_cumotion import cuMotionMoveGroup

# موشن پلانر بنائیں
planner = cuMotionMoveGroup(
    robot_description="/robot_description",
    planning_group="manipulator"
)

# ہدف پوز کی طرف منصوبہ بنائیں
success, trajectory = planner.plan_to_pose(
    target_pose=Pose(
        position=Point(0.5, 0.0, 0.3),
        orientation=Quaternion(0, 0, 0, 1)
    )
)

# ٹریجیکٹری عمل میں لائیں
if success:
    planner.execute(trajectory)
```

## MoveIt 2 انٹیگریشن

### MoveIt لانچ کریں

```bash
ros2 launch moveit_config move_group.launch.py
```

### پلاننگ سین

```python
from moveit_msgs.msg import PlanningScene, CollisionObject

# ٹکراؤ آبجیکٹ شامل کریں
collision_object = CollisionObject()
collision_object.id = "table"
collision_object.header.frame_id = "world"
collision_object.operation = CollisionObject.ADD

# پلاننگ سین میں پبلش کریں
planning_scene_publisher.publish(scene)
```

## پک اینڈ پلیس

```python
class PickAndPlace:
    def __init__(self):
        self.move_group = MoveGroupInterface("manipulator")
        self.gripper = GripperInterface()

    def pick(self, object_pose):
        # گرفت سے پہلے اپروچ
        approach_pose = self.compute_approach(object_pose)
        self.move_group.set_pose_target(approach_pose)
        self.move_group.go()

        # گرفت کی طرف جائیں
        self.move_group.set_pose_target(object_pose)
        self.move_group.go()

        # گرپر بند کریں
        self.gripper.close()

    def place(self, place_pose):
        # رکھنے کی جگہ پر جائیں
        self.move_group.set_pose_target(place_pose)
        self.move_group.go()

        # گرپر کھولیں
        self.gripper.open()

        # واپس ہٹیں
        retreat_pose = self.compute_retreat(place_pose)
        self.move_group.set_pose_target(retreat_pose)
        self.move_group.go()
```

## عملی لیب

### لیب 3.4: چھانٹنے کا کام

ایک روبوٹ نافذ کریں جو:
1. میز پر آبجیکٹس کا پتہ لگائے
2. ٹکراؤ سے پاک راستے بنائے
3. رنگ کے لحاظ سے آبجیکٹس چھانٹے

## خلاصہ

- cuMotion GPU سے تیز موشن پلاننگ فراہم کرتا ہے
- MoveIt 2 جامع مینیپولیشن ٹولز پیش کرتا ہے
- پک اینڈ پلیس ایک بنیادی مینیپولیشن کام ہے

[باب 3.5 پر جائیں ←](/docs/module-3-isaac/chapter-5-ros-integration)
