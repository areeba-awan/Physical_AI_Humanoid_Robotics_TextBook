---
sidebar_position: 4
title: "2.4 Unity Robotics Hub"
description: High-fidelity simulation with Unity
keywords: [Unity, robotics, simulation, perception]
---

# Chapter 2.4: Unity Robotics Hub

## Learning Objectives

- Set up Unity Robotics Hub
- Import URDF robots into Unity
- Connect Unity to ROS 2
- Create perception training scenarios

## Why Unity?

- **Photorealistic graphics** for vision AI training
- **ML-Agents** for reinforcement learning
- **Cross-platform** deployment
- **Large asset ecosystem**

## Setup

### Install Unity Robotics Hub

1. Open Unity Hub
2. Create new 3D project
3. Window > Package Manager
4. Add package from git URL:
   ```
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git
   ```

### ROS 2 Connector

```bash
# Install ROS TCP Endpoint
sudo apt install ros-humble-ros-tcp-endpoint

# Launch the endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

## Importing Robots

### URDF Importer

1. Assets > Import Robot from URDF
2. Select your URDF file
3. Configure import settings
4. Click Import

### Articulation Bodies

Unity uses Articulation Bodies for realistic physics:
- Reduced coordinates
- Stable joint simulation
- Direct force application

## ROS 2 Communication

### Publisher (Unity to ROS)

```csharp
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;

public class OdometryPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "odom";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
    }

    void Update()
    {
        TwistMsg msg = new TwistMsg();
        msg.linear.x = velocity.x;
        ros.Publish(topicName, msg);
    }
}
```

### Subscriber (ROS to Unity)

```csharp
public class CmdVelSubscriber : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance()
            .Subscribe<TwistMsg>("cmd_vel", OnCmdVel);
    }

    void OnCmdVel(TwistMsg msg)
    {
        // Apply velocity to robot
        targetVelocity = (float)msg.linear.x;
    }
}
```

## Synthetic Data Generation

### Camera Setup for Training

```csharp
public class SyntheticDataCapture : MonoBehaviour
{
    public Camera captureCamera;
    public int width = 640;
    public int height = 480;

    public void CaptureImage()
    {
        RenderTexture rt = new RenderTexture(width, height, 24);
        captureCamera.targetTexture = rt;
        Texture2D image = new Texture2D(width, height);
        captureCamera.Render();
        RenderTexture.active = rt;
        image.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        // Save or process image
    }
}
```

## Hands-on Lab

### Lab 2.4: Vision Training Environment

Create a Unity scene with:
- Random object placement
- Varied lighting conditions
- Camera capture for dataset generation

## Summary

- Unity provides photorealistic simulation
- ROS TCP Connector enables ROS 2 communication
- Ideal for vision AI and ML training

[Continue to Chapter 2.5 →](/ur/docs/module-2-simulation/chapter-5-sensors)
