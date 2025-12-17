---
sidebar_position: 4
title: "3.4 Manipulation Planning"
description: Robot arm motion planning with Isaac
keywords: [manipulation, motion planning, cuMotion, MoveIt]
---

# Chapter 3.4: Manipulation Planning

## Learning Objectives

- Understand motion planning concepts
- Use cuMotion for GPU-accelerated planning
- Integrate with MoveIt 2
- Implement pick-and-place operations

## cuMotion: GPU Motion Planning

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      cuMotion Pipeline                       │
│                                                              │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌────────┐│
│  │  Goal    │───>│  Graph   │───>│ Collision │───>│ Smooth ││
│  │  Pose    │    │  Search  │    │   Check   │    │ Traj   ││
│  └──────────┘    └──────────┘    └──────────┘    └────────┘│
│                       │               │                      │
│                       └───────────────┘                      │
│                        GPU Parallel                          │
└─────────────────────────────────────────────────────────────┘
```

### Using cuMotion

```python
from isaac_ros_cumotion import cuMotionMoveGroup

# Create motion planner
planner = cuMotionMoveGroup(
    robot_description="/robot_description",
    planning_group="manipulator"
)

# Plan to target pose
success, trajectory = planner.plan_to_pose(
    target_pose=Pose(
        position=Point(0.5, 0.0, 0.3),
        orientation=Quaternion(0, 0, 0, 1)
    )
)

# Execute trajectory
if success:
    planner.execute(trajectory)
```

## MoveIt 2 Integration

### Launch MoveIt

```bash
ros2 launch moveit_config move_group.launch.py
```

### Planning Scene

```python
from moveit_msgs.msg import PlanningScene, CollisionObject

# Add collision object
collision_object = CollisionObject()
collision_object.id = "table"
collision_object.header.frame_id = "world"
collision_object.operation = CollisionObject.ADD

# Publish to planning scene
planning_scene_publisher.publish(scene)
```

## Pick and Place

```python
class PickAndPlace:
    def __init__(self):
        self.move_group = MoveGroupInterface("manipulator")
        self.gripper = GripperInterface()

    def pick(self, object_pose):
        # Pre-grasp approach
        approach_pose = self.compute_approach(object_pose)
        self.move_group.set_pose_target(approach_pose)
        self.move_group.go()

        # Move to grasp
        self.move_group.set_pose_target(object_pose)
        self.move_group.go()

        # Close gripper
        self.gripper.close()

    def place(self, place_pose):
        # Move to place location
        self.move_group.set_pose_target(place_pose)
        self.move_group.go()

        # Open gripper
        self.gripper.open()

        # Retreat
        retreat_pose = self.compute_retreat(place_pose)
        self.move_group.set_pose_target(retreat_pose)
        self.move_group.go()
```

## Hands-on Lab

### Lab 3.4: Sorting Task

Implement a robot that:
1. Detects objects on a table
2. Plans collision-free paths
3. Sorts objects by color

## Summary

- cuMotion provides GPU-accelerated motion planning
- MoveIt 2 offers comprehensive manipulation tools
- Pick-and-place is a fundamental manipulation task

[Continue to Chapter 3.5 →](/ur/docs/module-3-isaac/chapter-5-ros-integration)
