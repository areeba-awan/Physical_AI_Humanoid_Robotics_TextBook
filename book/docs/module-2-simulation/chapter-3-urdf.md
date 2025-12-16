---
sidebar_position: 3
title: "2.3 URDF اور روبوٹ ماڈلز"
description: URDF کے ساتھ روبوٹ ڈسکرپشنز بنانا
keywords: [URDF, روبوٹ ماڈل, لنکس, جوائنٹس, xacro]
---

# باب 2.3: URDF اور روبوٹ ماڈلز

## سیکھنے کے مقاصد

- URDF کی ساخت کو سمجھیں
- لنکس اور جوائنٹس بنائیں
- بصری اور ٹکراؤ کی جیومیٹریز شامل کریں
- ماڈیولر روبوٹ ڈسکرپشنز کے لیے xacro استعمال کریں

## URDF کی ساخت

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- بیس لنک -->
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

  <!-- پہیے کا جوائنٹ -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel"/>
    <origin xyz="0.2 0.2 0" rpy="0 1.5708 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- پہیے کا لنک -->
  <link name="wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## جوائنٹ کی اقسام

| قسم | DOF | تفصیل |
|------|-----|-------------|
| `fixed` | 0 | کوئی حرکت نہیں |
| `revolute` | 1 | حدود کے ساتھ گردش |
| `continuous` | 1 | لامحدود گردش |
| `prismatic` | 1 | خطی حرکت |
| `floating` | 6 | آزاد حرکت |

## Xacro کا استعمال

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="my_robot">
  <!-- پہیے کا ریڈیئس متغیر -->
  <xacro:property name="wheel_radius" value="0.1"/>

  <!-- پہیے کا میکرو -->
  <xacro:macro name="wheel" params="prefix side">
    <link name="${prefix}_${side}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="0.05"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <!-- میکرو کا استعمال -->
  <xacro:wheel prefix="front" side="left"/>
  <xacro:wheel prefix="front" side="right"/>
</robot>
```

## RViz میں ویژولائزیشن

```bash
ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf
```

## عملی لیب

### لیب 2.3: موبائل روبوٹ بنائیں

4 پہیوں والا روبوٹ بنائیں جس میں:
- ڈفرینشل ڈرائیو بیس
- لائیڈار سینسر ماؤنٹ
- کیمرہ ماؤنٹ

## خلاصہ

- URDF لنکس اور جوائنٹس کے ساتھ روبوٹ کی ساخت کی تعریف کرتا ہے
- Xacro ماڈیولر، دوبارہ قابل استعمال ڈسکرپشنز کو ممکن بناتا ہے
- فزکس کے لیے مناسب اینرشیلز بہت اہم ہیں

[باب 2.4 جاری رکھیں ←](/docs/module-2-simulation/chapter-4-unity)
