---
sidebar_position: 4
title: "2.4 یونٹی روبوٹکس ہب"
description: "High-fidelity simulation with یونٹی"
keywords: ["یونٹی", "robotics", "simulation", "perception"]
---

# باب 2.4: یونٹی روبوٹکس ہب

## سیکھنے کے مقاصد

-  تیار کریں یونٹی روبوٹکس ہب
- Import URDF robots into یونٹی
- Connect یونٹی کا تعارف
- تخلیق کریں perception training scenarios

## Why یونٹی?

- **فوٹو ریئلیسٹک graphics** for vision AI training
- **ML-Agents** for reinforcement learning
- **Cross-platform** deployment
- **Large asset ecosystem**

## Setup

### انسٹال کریں یونٹی روبوٹکس ہب

1. Open یونٹی Hub
2. تخلیق کریں new 3D project
3. Window > Package Manager
4. شامل کریں package from git URL:
   ```
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git
   ```

### ROS 2 Connector

```bash
# انسٹال کریں ROS TCP Endpoint
sudo apt install ros-humble-ros-tcp-endpoint

# لانچ کریں the endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

## Importing Robots

### URDF Importer

1. Assets > Import Robot from URDF
2. Select your URDF file
3. Configure import settings
4. Click Import

### Articulation Bodies

یونٹی uses Articulation Bodies for realistic physics:
- Reduced coordinates
- Stable joint simulation
- Direct force application

## ROS 2 Communication

### شائع کنندہ (یونٹی to ROS)

```csharp
using RosMessageTypes.Geometry;
using یونٹی.Robotics.ROSTCPConnector;

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

###  مسیحین (ROS to یونٹی)

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

##  مصنوعی ڈیٹا Generation

### Camera Setup for  تربیت

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

## ہاتھ سے کام کرنے والی لیب

### Lab 2.4: Vision  تربیت ماحول

تخلیق کریں a یونٹی scene with:
- Random object placement
- Varied lighting conditions
- Camera capture for dataset generation

## خلاصہ

- یونٹی provides photorealistic simulation
- ROS TCP Connector enables ROS 2 communication
- Ideal for vision AI and ML training

[Continue to باب 2.5 →](/docs/module-2-simulation/chapter-5-sensors)


