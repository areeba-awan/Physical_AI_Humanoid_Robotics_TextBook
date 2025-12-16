---
sidebar_position: 4
title: "2.4 یونٹی روبوٹکس ہب"
description: یونٹی کے ساتھ اعلیٰ درجے کی سمولیشن
keywords: [یونٹی, روبوٹکس, سمولیشن, پرسیپشن]
---

# باب 2.4: یونٹی روبوٹکس ہب

## سیکھنے کے مقاصد

- یونٹی روبوٹکس ہب ترتیب دیں
- URDF روبوٹس کو یونٹی میں درآمد کریں
- یونٹی کو ROS 2 سے جوڑیں
- پرسیپشن ٹریننگ کے منظرنامے بنائیں

## یونٹی کیوں؟

- **فوٹو ریئلسٹک گرافکس** - ویژن AI ٹریننگ کے لیے
- **ML-Agents** - ریانفورسمنٹ لرننگ کے لیے
- **کراس پلیٹ فارم** - مختلف پلیٹ فارمز پر ڈیپلائے
- **بڑا ایسٹ ایکو سسٹم** - بہت سے ریڈی میڈ ایسٹس

## سیٹ اپ

### یونٹی روبوٹکس ہب انسٹال کریں

1. یونٹی ہب کھولیں
2. نیا 3D پروجیکٹ بنائیں
3. Window > Package Manager
4. گٹ URL سے پیکیج شامل کریں:
   ```
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git
   ```

### ROS 2 کنیکٹر

```bash
# ROS TCP Endpoint انسٹال کریں
sudo apt install ros-humble-ros-tcp-endpoint

# اینڈ پوائنٹ شروع کریں
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

## روبوٹس درآمد کرنا

### URDF امپورٹر

1. Assets > Import Robot from URDF
2. اپنی URDF فائل منتخب کریں
3. امپورٹ سیٹنگز ترتیب دیں
4. Import پر کلک کریں

### آرٹیکیولیشن باڈیز

یونٹی حقیقت پسندانہ فزکس کے لیے آرٹیکیولیشن باڈیز استعمال کرتا ہے:
- ریڈیوسڈ کوآرڈینیٹس
- مستحکم جوائنٹ سمولیشن
- براہ راست فورس اپلیکیشن

## ROS 2 کمیونیکیشن

### پبلشر (یونٹی سے ROS)

```csharp
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;

public class OdometryPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "odom";

    void Start()
    {
        // ROS کنکشن حاصل کریں
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
    }

    void Update()
    {
        // پیغام بنائیں اور بھیجیں
        TwistMsg msg = new TwistMsg();
        msg.linear.x = velocity.x;
        ros.Publish(topicName, msg);
    }
}
```

### سبسکرائبر (ROS سے یونٹی)

```csharp
public class CmdVelSubscriber : MonoBehaviour
{
    void Start()
    {
        // ٹاپک پر سبسکرائب کریں
        ROSConnection.GetOrCreateInstance()
            .Subscribe<TwistMsg>("cmd_vel", OnCmdVel);
    }

    void OnCmdVel(TwistMsg msg)
    {
        // روبوٹ پر ویلاسٹی لگائیں
        targetVelocity = (float)msg.linear.x;
    }
}
```

## مصنوعی ڈیٹا جنریشن

### ٹریننگ کے لیے کیمرہ سیٹ اپ

```csharp
public class SyntheticDataCapture : MonoBehaviour
{
    public Camera captureCamera;
    public int width = 640;
    public int height = 480;

    public void CaptureImage()
    {
        // رینڈر ٹیکسچر بنائیں
        RenderTexture rt = new RenderTexture(width, height, 24);
        captureCamera.targetTexture = rt;
        Texture2D image = new Texture2D(width, height);
        captureCamera.Render();
        RenderTexture.active = rt;
        image.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        // تصویر محفوظ یا پراسیس کریں
    }
}
```

## عملی لیب

### لیب 2.4: ویژن ٹریننگ ماحول

یونٹی سین بنائیں جس میں:
- رینڈم آبجیکٹ پلیسمنٹ
- مختلف روشنی کے حالات
- ڈیٹاسیٹ جنریشن کے لیے کیمرہ کیپچر

## خلاصہ

- یونٹی فوٹو ریئلسٹک سمولیشن فراہم کرتا ہے
- ROS TCP کنیکٹر ROS 2 کمیونیکیشن کو ممکن بناتا ہے
- ویژن AI اور ML ٹریننگ کے لیے مثالی

[باب 2.5 جاری رکھیں ←](/docs/module-2-simulation/chapter-5-sensors)
