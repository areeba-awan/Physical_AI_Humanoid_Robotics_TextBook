# Physical AI & Humanoid Robotics Research

**Feature**: Physical AI & Humanoid Robotics Book Content
**Purpose**: Document research sources, citations, and validation notes
**Created**: 2025-12-11
**Last Updated**: 2025-12-11
**Version**: 1.0.0

---

## 1. Overview

This document catalogs all research sources, citations, and validation notes for the Physical AI & Humanoid Robotics book. All technical claims must be traced to authoritative sources documented here.

**Research Methodology:**
1. Documentation-First: Prioritize official docs, standards, RFCs
2. Peer Review: Academic papers and industry publications
3. Validation: Test code examples against documented APIs
4. Currency: Prefer 2023-2025 sources; note version dependencies

---

## 2. Primary Sources (Official Documentation)

### 2.1 ROS 2 Documentation

| Resource | URL | Version | Last Accessed | Notes |
|----------|-----|---------|---------------|-------|
| ROS 2 Main Docs | https://docs.ros.org | Humble (LTS) | 2025-12-11 | Primary ROS 2 reference |
| ROS 2 Concepts | https://docs.ros.org/en/humble/Concepts.html | Humble | 2025-12-11 | Nodes, topics, services, actions |
| ROS 2 Tutorials | https://docs.ros.org/en/humble/Tutorials.html | Humble | 2025-12-11 | Code examples source |
| rclpy API Docs | https://docs.ros.org/en/humble/p/rclpy/ | Humble | 2025-12-11 | Python client library |
| ROS 2 Design Docs | https://design.ros2.org | Latest | 2025-12-11 | Architecture decisions |

**Citation Format (MLA):**
```
Open Robotics. "ROS 2 Concepts." ROS 2 Documentation, Humble, 2023, https://docs.ros.org/en/humble/Concepts.html. Accessed 11 Dec. 2025.
```

**Key Topics Covered:**
- Nodes and composition
- Topics and publishers/subscribers
- Services and clients
- Actions and action servers
- Launch systems
- Parameters and dynamic reconfiguration

### 2.2 Gazebo Simulation Documentation

| Resource | URL | Version | Last Accessed | Notes |
|----------|-----|---------|---------------|-------|
| Gazebo Main Docs | https://gazebosim.org/docs | Fortress | 2025-12-11 | Primary Gazebo reference |
| Gazebo Tutorials | https://gazebosim.org/docs/fortress/tutorials | Fortress | 2025-12-11 | Setup and usage |
| SDF Specification | http://sdformat.org/spec | 1.9 | 2025-12-11 | Robot description format |
| Gazebo Plugins | https://gazebosim.org/api/gazebo/6/namespacegz.html | Fortress | 2025-12-11 | Sensor and actuator plugins |

**Citation Format (MLA):**
```
Open Robotics. "Gazebo Tutorials." Gazebo Documentation, Fortress, 2023, https://gazebosim.org/docs/fortress/tutorials. Accessed 11 Dec. 2025.
```

**Key Topics Covered:**
- World and model creation
- Physics engine configuration (ODE, Bullet, DART)
- Sensor simulation (LiDAR, cameras, IMU)
- Plugin development
- ROS 2 integration (ros_gz_bridge)

### 2.3 NVIDIA Isaac Sim Documentation

| Resource | URL | Version | Last Accessed | Notes |
|----------|-----|---------|---------------|-------|
| Isaac Sim Main Docs | https://docs.omniverse.nvidia.com/isaacsim/latest/index.html | 2023.1+ | 2025-12-11 | Primary Isaac Sim reference |
| Isaac ROS Docs | https://nvidia-isaac-ros.github.io/index.html | Latest | 2025-12-11 | ROS 2 integration |
| USD Documentation | https://openusd.org/release/index.html | 23.11 | 2025-12-11 | Scene description format |
| Isaac Tutorials | https://docs.omniverse.nvidia.com/isaacsim/latest/introductory_tutorials/index.html | 2023.1+ | 2025-12-11 | Getting started |

**Citation Format (MLA):**
```
NVIDIA. "Isaac Sim Introduction." Isaac Sim Documentation, version 2023.1, 2023, https://docs.omniverse.nvidia.com/isaacsim/latest/index.html. Accessed 11 Dec. 2025.
```

**Key Topics Covered:**
- USD scene creation and manipulation
- Articulation and kinematics
- Sensor simulation (cameras, LiDAR, depth)
- Isaac ROS integration
- Reinforcement learning (Isaac Gym)

### 2.4 Nav2 Documentation

| Resource | URL | Version | Last Accessed | Notes |
|----------|-----|---------|---------------|-------|
| Nav2 Main Docs | https://navigation.ros.org | Humble | 2025-12-11 | Navigation stack reference |
| Nav2 Tutorials | https://navigation.ros.org/tutorials/index.html | Humble | 2025-12-11 | Setup and configuration |
| Nav2 Plugins | https://navigation.ros.org/plugins/index.html | Humble | 2025-12-11 | Controller and planner options |
| Nav2 Behavior Trees | https://navigation.ros.org/behavior_trees/index.html | Humble | 2025-12-11 | BT-based navigation |

**Citation Format (MLA):**
```
Open Navigation. "Nav2 Overview." Nav2 Documentation, Humble, 2023, https://navigation.ros.org. Accessed 11 Dec. 2025.
```

**Key Topics Covered:**
- SLAM and localization
- Global and local planning
- Costmap configuration
- Behavior trees for navigation
- Recovery behaviors

### 2.5 MoveIt 2 Documentation

| Resource | URL | Version | Last Accessed | Notes |
|----------|-----|---------|---------------|-------|
| MoveIt 2 Main Docs | https://moveit.ros.org | Humble | 2025-12-11 | Manipulation stack reference |
| MoveIt 2 Tutorials | https://moveit.picknik.ai/humble/index.html | Humble | 2025-12-11 | Setup and usage |
| OMPL Documentation | https://ompl.kavrakilab.org | 1.6+ | 2025-12-11 | Motion planning library |

**Citation Format (MLA):**
```
PickNik Robotics. "MoveIt 2 Overview." MoveIt 2 Documentation, Humble, 2023, https://moveit.ros.org. Accessed 11 Dec. 2025.
```

**Key Topics Covered:**
- Motion planning
- Kinematics and IK solvers
- Collision detection
- Grasp planning
- Trajectory execution

---

## 3. Secondary Sources (Academic & Industry)

### 3.1 SLAM and Localization

| Paper/Resource | Authors | Year | Venue | URL | Notes |
|----------------|---------|------|-------|-----|-------|
| Cartographer Algorithm | Hess et al. | 2016 | IEEE ICRA | [Link](https://ieeexplore.ieee.org/document/7487258) | Google's SLAM system |
| ORB-SLAM3 | Campos et al. | 2021 | IEEE TRO | [Link](https://ieeexplore.ieee.org/document/9440682) | Visual-inertial SLAM |
| SLAM Toolbox | Macenski | 2021 | JOSS | [Link](https://joss.theoj.org/papers/10.21105/joss.02783) | ROS 2 SLAM |

**Citation Example (MLA):**
```
Hess, Wolfgang, et al. "Real-Time Loop Closure in 2D LIDAR SLAM." 2016 IEEE International Conference on Robotics and Automation (ICRA), 2016, pp. 1271-1278. DOI: 10.1109/ICRA.2016.7487258.
```

### 3.2 Vision-Language-Action Models

| Paper/Resource | Authors | Year | Venue | URL | Notes |
|----------------|---------|------|-------|-----|-------|
| RT-1 | Brohan et al. | 2022 | arXiv | [Link](https://arxiv.org/abs/2212.06817) | Google's robotics transformer |
| RT-2 | Brohan et al. | 2023 | arXiv | [Link](https://arxiv.org/abs/2307.15818) | Vision-language-action model |
| PaLM-E | Driess et al. | 2023 | ICML | [Link](https://arxiv.org/abs/2303.03378) | Embodied multimodal LLM |
| OpenVLA | Kim et al. | 2024 | arXiv | [Link](https://arxiv.org/abs/2406.09246) | Open-source VLA |

**Citation Example (MLA):**
```
Brohan, Anthony, et al. "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control." arXiv preprint arXiv:2307.15818, 2023.
```

### 3.3 Object Detection and Computer Vision

| Resource | Authors | Year | Venue | URL | Notes |
|----------|---------|------|-------|-----|-------|
| YOLOv8 | Jocher et al. | 2023 | Ultralytics | [Link](https://github.com/ultralytics/ultralytics) | Real-time object detection |
| SAM (Segment Anything) | Kirillov et al. | 2023 | ICCV | [Link](https://arxiv.org/abs/2304.02643) | Universal segmentation |
| DINO | Zhang et al. | 2022 | ICLR | [Link](https://arxiv.org/abs/2203.03605) | Vision transformer detection |

**Citation Example (MLA):**
```
Kirillov, Alexander, et al. "Segment Anything." Proceedings of the IEEE/CVF International Conference on Computer Vision (ICCV), 2023, pp. 4015-4026.
```

### 3.4 Humanoid Robotics

| Resource | Authors/Organization | Year | Type | URL | Notes |
|----------|----------------------|------|------|-----|-------|
| Unitree H1 Specs | Unitree Robotics | 2024 | Datasheet | [Link](https://www.unitree.com/h1) | 175cm humanoid |
| Atlas Technical Paper | Boston Dynamics | 2023 | Whitepaper | [Link](https://bostondynamics.com/atlas/) | Advanced humanoid |
| Digit Specs | Agility Robotics | 2024 | Datasheet | [Link](https://agilityrobotics.com/digit) | Commercial humanoid |

**Citation Example (MLA):**
```
Unitree Robotics. "H1 Humanoid Robot Specifications." Unitree, 2024, https://www.unitree.com/h1. Accessed 11 Dec. 2025.
```

### 3.5 Speech Recognition

| Resource | Authors | Year | Venue | URL | Notes |
|----------|---------|------|-------|-----|-------|
| Whisper | Radford et al. | 2022 | OpenAI | [Link](https://arxiv.org/abs/2212.04356) | Robust speech recognition |
| Whisper GitHub | OpenAI | 2023 | GitHub | [Link](https://github.com/openai/whisper) | Official implementation |

**Citation Example (MLA):**
```
Radford, Alec, et al. "Robust Speech Recognition via Large-Scale Weak Supervision." arXiv preprint arXiv:2212.04356, 2022.
```

---

## 4. Code Examples Validation

### 4.1 ROS 2 Examples

| Example | Source | Tested | ROS Version | OS | Status |
|---------|--------|--------|-------------|-----|--------|
| Publisher/Subscriber | ROS 2 Tutorials | ✅ | Humble | Ubuntu 22.04 | Verified |
| Service/Client | ROS 2 Tutorials | ✅ | Humble | Ubuntu 22.04 | Verified |
| Action Server | ROS 2 Tutorials | ✅ | Humble | Ubuntu 22.04 | Verified |
| Launch Files | ROS 2 Tutorials | ✅ | Humble | Ubuntu 22.04 | Verified |
| Parameters | ROS 2 Tutorials | ⬜ | Humble | Ubuntu 22.04 | To Do |

**Validation Notes:**
- All examples tested in clean ROS 2 Humble installation
- Ubuntu 22.04 LTS used for consistency
- Code linted with pylint and black

### 4.2 Gazebo Examples

| Example | Source | Tested | Gazebo Version | Status |
|---------|--------|--------|----------------|--------|
| World Creation | Gazebo Tutorials | ⬜ | Fortress | To Do |
| Robot Spawning | Gazebo Tutorials | ⬜ | Fortress | To Do |
| Sensor Plugins | Gazebo Tutorials | ⬜ | Fortress | To Do |
| ROS 2 Bridge | ros_gz Docs | ⬜ | Fortress | To Do |

### 4.3 Isaac Sim Examples

| Example | Source | Tested | Isaac Sim Version | Status |
|---------|--------|--------|-------------------|--------|
| USD Scene | Isaac Sim Docs | ⬜ | 2023.1 | To Do |
| Robot Import | Isaac Sim Docs | ⬜ | 2023.1 | To Do |
| ROS 2 Integration | Isaac ROS Docs | ⬜ | 2023.1 | To Do |

---

## 5. Terminology Glossary

Consistent terminology across all chapters:

| Term | Definition | First Use Chapter | Related Terms |
|------|------------|-------------------|---------------|
| Physical AI | AI systems embodied in physical agents (robots) | Chapter 1 | Embodied AI |
| VLA | Vision-Language-Action model architecture | Chapter 5 | Multimodal AI |
| SLAM | Simultaneous Localization and Mapping | Chapter 4 | Mapping, Localization |
| Nav2 | ROS 2 Navigation Stack (version 2) | Chapter 4 | Navigation |
| MoveIt 2 | ROS 2 Manipulation Stack | Chapter 6 | Motion Planning |
| URDF | Unified Robot Description Format | Chapter 2 | SDF, USD |
| USD | Universal Scene Description (Pixar) | Chapter 3 | Isaac Sim |
| DOF | Degrees of Freedom | Chapter 2 | Joints |
| IK | Inverse Kinematics | Chapter 6 | FK, Kinematics |

---

## 6. Research Checklist

### 6.1 Before Writing a Chapter

- [ ] Identify 3-5 authoritative sources for technical claims
- [ ] Verify current versions of all tools/frameworks mentioned
- [ ] Test all code examples in clean environment
- [ ] Document citations in this file
- [ ] Check for recent updates to official documentation

### 6.2 During Writing

- [ ] Cite sources inline for all technical claims
- [ ] Use consistent terminology from glossary
- [ ] Link to official documentation for APIs
- [ ] Note version dependencies explicitly

### 6.3 After Writing

- [ ] Verify all citations formatted correctly (MLA or APA)
- [ ] Ensure all sources dated and accessible
- [ ] Add new terms to glossary if needed
- [ ] Update research notes with findings

---

## 7. Open Research Questions

### 7.1 Pending Verification

| Question | Status | Priority | Notes |
|----------|--------|----------|-------|
| Isaac Sim installation on non-NVIDIA GPUs | Research | Medium | Check official docs for AMD/Intel support |
| VLA model latency for real-time control | Research | High | Need benchmarks for RT-2/OpenVLA |
| MoveIt 2 Humble vs Iron differences | Research | Low | Check if examples need updates |

### 7.2 Future Topics (Out of Scope for v1.0)

- Deep reinforcement learning with Isaac Gym
- Multi-robot coordination
- Sim-to-real transfer techniques
- Custom sensor plugin development
- ROS 2 real-time performance tuning

---

## 8. Source Update Policy

### 8.1 Monitoring

- **Frequency**: Monthly check for major documentation updates
- **Trigger**: New ROS 2 distribution release, Gazebo version bump
- **Process**: Review release notes, test breaking changes, update chapters

### 8.2 Version Pinning

- **ROS 2**: Pin to Humble (LTS) until 2027 EOL
- **Gazebo**: Pin to Fortress (stable), note Garden availability
- **Isaac Sim**: Update with each major release (2024.x → 2025.x)
- **Python Libraries**: Specify minimum versions, test against latest

---

## 9. Governance

This research document is governed by:
- **Constitution**: `.specify/memory/constitution.md`
- **Specification**: `specs/physical-ai-robotics/spec.md`
- **Plan**: `specs/physical-ai-robotics/plan.md`

**Research Updates:**
- New sources added as chapters are written
- Validation status updated as code is tested
- Citations added to chapters as content is created
- PHR created for significant research findings

**Version History:**
- v1.0.0 (2025-12-11): Initial research structure created

---

**End of Research Document**
