---
sidebar_position: 6
title: "4.6 کیپسٹون: VLA روبوٹ پروجیکٹ"
description: "آواز سے کنٹرول مکمل ہدایات پر عمل کرنے والا روبوٹ بنائیں"
keywords: ["کیپسٹون", "پروجیکٹ", "VLA", "روبوٹ", "ہدایات کی پیروی", "آواز کنٹرول", "Whisper"]
---

# باب 4.6: کیپسٹون - VLA روبوٹ پروجیکٹ

## پروجیکٹ کا جائزہ

اس کیپسٹون میں، آپ ایک مکمل VLA سے چلنے والا روبوٹ بنائیں گے جو:

- قدرتی زبان کی ہدایات سمجھ سکے
- ماحول میں اشیاء کو دیکھ سکے
- ہینڈلنگ کے کاموں کی منصوبہ بندی اور عمل کر سکے
- مظاہروں سے سیکھ سکے
- **ویک ورڈ ایکٹیویشن کے ساتھ آواز کے احکامات کا جواب دے**
- **آواز فیڈبیک اور تصدیقات فراہم کرے**

## پروجیکٹ: آواز سے کنٹرول کچن اسسٹنٹ روبوٹ

### سسٹم آرکیٹیکچر

```
┌──────────────────────────────────────────────────────────────────┐
│           آواز سے کنٹرول کچن اسسٹنٹ VLA سسٹم                    │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────┐    │
│  │                    صارف انٹرفیس                           │    │
│  │                                                            │    │
│  │   🎤 آواز ان پٹ ─────────────┐                           │    │
│  │   "ارے شیف، سیب کو پیالے    │                           │    │
│  │    میں رکھو"                 │                           │    │
│  │                               │                           │    │
│  │   🔊 آواز آؤٹ پٹ ◄───────────┼────────────────────┐     │    │
│  │   "سیب اٹھا رہا ہوں..."      │                    │     │    │
│  │                               │                    │     │    │
│  └───────────────────────────────┼────────────────────┼─────┘    │
│                                  │                    │           │
│  ┌───────────────────────────────┴────────────────────┴─────┐    │
│  │                    آواز پروسیسنگ                          │    │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────────────┐  │    │
│  │  │  ویک ورڈ  │  │  Whisper   │  │   ٹیکسٹ ٹو اسپیچ  │  │    │
│  │  │   ڈیٹیکشن │──│    STT     │  │      (gTTS)        │  │    │
│  │  │(Porcupine) │  │            │  │                    │  │    │
│  │  └────────────┘  └──────┬─────┘  └────────────────────┘  │    │
│  └─────────────────────────┼────────────────────────────────┘    │
│                            │                                      │
│  ┌─────────────────────────┴────────────────────────────────┐    │
│  │                    VLA دماغ                               │    │
│  │                                                            │    │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────┐          │    │
│  │  │   ویژن    │  │   زبان    │  │   ایکشن   │          │    │
│  │  │  انکوڈر   │  │  انکوڈر   │  │  ڈیکوڈر   │          │    │
│  │  │   (ViT)    │  │   (T5)     │  │   (MLP)    │          │    │
│  │  └──────┬─────┘  └──────┬─────┘  └──────┬─────┘          │    │
│  │         └───────────────┼───────────────┘                 │    │
│  │                         │                                  │    │
│  │                  فیوژن + ایکشن                            │    │
│  └─────────────────────────┬────────────────────────────────┘    │
│                            │                                      │
│  ┌─────────────────────────┴────────────────────────────────┐    │
│  │                    روبوٹ ہارڈویئر                         │    │
│  │  ┌────────┐  ┌────────┐  ┌────────┐  ┌────────────────┐  │    │
│  │  │ کیمرہ  │  │  بازو  │  │ گرپر  │  │  مائیکروفون    │  │    │
│  │  │  x2    │  │ 7-DOF  │  │ 2-جبڑا │  │  + اسپیکر     │  │    │
│  │  └────────┘  └────────┘  └────────┘  └────────────────┘  │    │
│  └──────────────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────────────┘
```

## مرحلہ 1: انوائرنمنٹ سیٹ اپ

### ایزیک سیم سین

```python
# capstone/setup_kitchen.py
from omni.isaac.core import دنیا
from omni.isaac.manipulators import SingleManipulator

def setup_kitchen():
    world = World()

    # روبوٹ شامل کریں
    robot = world.scene.add(
        SingleManipulator(
            prim_path="/World/FrankaKitchen",
            usd_path="/Isaac/Robots/Franka/franka.usd"
        )
    )

    # کچن کی اشیاء شامل کریں
    items = [
        ("apple", "/assets/apple.usd", [0.3, 0.0, 0.02]),    # سیب
        ("bowl", "/assets/bowl.usd", [0.4, 0.2, 0.0]),       # پیالہ
        ("cup", "/assets/cup.usd", [0.2, -0.2, 0.0]),        # کپ
        ("plate", "/assets/plate.usd", [0.5, 0.0, 0.0]),     # پلیٹ
    ]

    for name, usd, pos in items:
        world.scene.add_usd_to_stage(usd, f"/World/{name}")

    return world, robot
```

## مرحلہ 2: VLA ماڈل

### ماڈل آرکیٹیکچر

```python
# capstone/vla_model.py
import torch
import torch.nn as nn
from transformers import ViTModel, T5EncoderModel

class KitchenVLA(nn.مودیول):
    def __init__(self):
        super().__init__()

        # ویژن انکوڈر
        self.vision = ViTModel.from_pretrained('google/vit-base-patch16-224')

        # زبان انکوڈر
        self.language = T5EncoderModel.from_pretrained('t5-base')

        # فیوژن لیئر
        self.fusion = nn.MultiheadAttention(embed_dim=768, num_heads=8)

        # ایکشن ڈیکوڈر
        self.action_head = nn.Sequential(
            nn.Linear(768, 256),
            nn.ReLU(),
            nn.Linear(256, 7),  # 7-DOF ایکشن
            nn.Tanh()
        )

    def forward(self, image, instruction_tokens):
        # ویژن انکوڈ کریں
        vis_features = self.vision(image).last_hidden_state

        # زبان انکوڈ کریں
        lang_features = self.language(instruction_tokens).last_hidden_state

        # کراس-اٹینشن فیوژن
        fused, _ = self.fusion(
            query=vis_features,
            key=lang_features,
            value=lang_features
        )

        # ایکشن ڈیکوڈ کریں
        pooled = fused.mean(dim=1)
        action = self.action_head(pooled)

        return action
```

## مرحلہ 3: تربیت

### ڈیٹا کلیکشن

```python
# capstone/collect_demos.py
class DemoCollector:
    def __init__(self, robot):
        self.robot = robot
        self.demos = []

    def record_demo(self, instruction):
        demo = {
            'instruction': instruction,
            'observations': [],
            'actions': []
        }

        print(f"ڈیمو ریکارڈ ہو رہا ہے: {instruction}")
        print("کام مکمل کرنے کے لیے روبوٹ کو حرکت دیں...")

        while not self.task_complete():
            obs = self.get_observation()
            action = self.get_human_action()

            demo['observations'].append(obs)
            demo['actions'].append(action)

        self.demos.append(demo)
        return demo
```

### تربیتی لوپ

```python
# capstone/train.py
def train_vla(model, dataloader, epochs=100):
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-4)
    criterion = nn.MSELoss()

    for epoch in range(epochs):
        for batch in dataloader:
            images = batch['images']
            instructions = batch['instructions']
            target_actions = batch['actions']

            # فارورڈ پاس
            pred_actions = model(images, instructions)

            # لاس
            loss = criterion(pred_actions, target_actions)

            # بیکورڈ
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

        print(f"ایپاک {epoch}: لاس = {loss.item():.4f}")
```

## مرحلہ 4: آواز کنٹرول انٹیگریشن

### آواز کنٹرول سسٹم

```python
# capstone/voice_control.py
"""کچن اسسٹنٹ کے لیے آواز کنٹرول سسٹم"""

import whisper
import numpy as np
import sounddevice as sd
from gtts import gTTS
import pygame
import io
import threading
import queue

class VoiceController:
    """کچن روبوٹ کے لیے آواز کنٹرول"""

    def __init__(self, wake_word="ارے شیف"):
        # اسپیچ ریکگنیشن
        print("Whisper ماڈل لوڈ ہو رہا ہے...")
        self.whisper = whisper.load_model("small")
        self.wake_word = wake_word.lower()

        # آڈیو سیٹنگز
        self.sample_rate = 16000
        self.audio_queue = queue.Queue()
        self.is_listening = True

        # حالت
        self.is_awake = False
        self.awake_timeout = 10.0  # سیکنڈز

        # آڈیو پلے بیک کے لیے pygame شروع کریں
        pygame.mixer.init()

    def start(self):
        """آواز کنٹرول سننا شروع کریں"""
        self.listen_thread = threading.Thread(target=self._listen_loop)
        self.listen_thread.daemon = True
        self.listen_thread.start()
        print(f"آواز کنٹرول فعال۔ شروع کرنے کے لیے '{self.wake_word}' کہیں۔")

    def _listen_loop(self):
        """مسلسل سننے کا لوپ"""
        buffer = []
        silence_frames = 0

        with sd.InputStream(samplerate=self.sample_rate,
                           channels=1, dtype=np.int16,
                           blocksize=1024) as stream:
            while self.is_listening:
                audio_chunk, _ = stream.read(1024)
                audio_chunk = audio_chunk.flatten()

                # آواز کی سرگرمی کا پتہ لگائیں
                energy = np.abs(audio_chunk).mean()

                if energy > 300:  # آواز ملی
                    buffer.extend(audio_chunk)
                    silence_frames = 0
                else:
                    silence_frames += 1

                # خاموشی کے بعد پروسیس کریں
                if silence_frames > 10 and len(buffer) > self.sample_rate:
                    audio = np.array(buffer, dtype=np.float32) / 32768.0
                    self.audio_queue.put(audio)
                    buffer = []

    def get_command(self, timeout=5.0):
        """آواز کمانڈ حاصل کریں (بلاکنگ)"""
        try:
            audio = self.audio_queue.get(timeout=timeout)
            return self._transcribe(audio)
        except queue.Empty:
            return None

    def _transcribe(self, audio):
        """آڈیو کو ٹیکسٹ میں تبدیل کریں"""
        result = self.whisper.transcribe(audio, language="ur")
        text = result["text"].strip().lower()

        # ویک ورڈ کی جانچ کریں
        if not self.is_awake:
            if self.wake_word in text:
                self.is_awake = True
                # ویک ورڈ کے بعد کمانڈ نکالیں
                parts = text.split(self.wake_word)
                if len(parts) > 1:
                    return parts[-1].strip()
                return ""  # صرف ویک ورڈ، کمانڈ کا انتظار
            return None
        else:
            return text

    def speak(self, text):
        """ٹیکسٹ سے آواز آؤٹ پٹ"""
        print(f"روبوٹ: {text}")
        tts = gTTS(text=text, lang='ur')
        fp = io.BytesIO()
        tts.write_to_fp(fp)
        fp.seek(0)

        pygame.mixer.music.load(fp)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pygame.time.wait(100)

    def confirm_action(self, action_description):
        """ایکشن کی تصدیق بولیں"""
        self.speak(action_description)

    def ask_clarification(self, question):
        """صارف سے وضاحت مانگیں اور جواب لیں"""
        self.speak(question)
        return self.get_command(timeout=10.0)

    def stop(self):
        """آواز کنٹرول بند کریں"""
        self.is_listening = False


class VoiceCommandParser:
    """آواز کے احکامات کو روبوٹ ایکشنز میں پارس کریں"""

    COMMANDS = {
        'pick': ['اٹھاؤ', 'پکڑو', 'لو', 'لے لو'],
        'place': ['رکھو', 'ڈالو', 'رکھ دو'],
        'move': ['جاؤ', 'ہلو', 'چلو'],
        'pour': ['ڈالو', 'بھرو'],
        'stop': ['رکو', 'بند کرو', 'وقفہ'],
        'status': ['کیا', 'کہاں', 'دکھاؤ', 'بتاؤ'],
    }

    OBJECTS = ['سیب', 'پیالہ', 'کپ', 'پلیٹ', 'بوتل', 'گلاس', 'کانٹا', 'چھری', 'چمچ']
    LOCATIONS = ['میز', 'کاؤنٹر', 'سنک', 'فریج', 'الماری', 'بائیں', 'دائیں', 'یہاں']

    def parse(self, command):
        """آواز کمانڈ کو سٹرکچرڈ ایکشن میں پارس کریں"""
        if not command:
            return None

        command = command.lower()

        # ایکشن کی قسم کا پتہ لگائیں
        action_type = None
        for action, keywords in self.COMMANDS.items():
            if any(kw in command for kw in keywords):
                action_type = action
                break

        if not action_type:
            return {'type': 'unknown', 'raw': command}

        # اشیاء اور جگہیں نکالیں
        objects = [obj for obj in self.OBJECTS if obj in command]
        locations = [loc for loc in self.LOCATIONS if loc in command]

        return {
            'type': action_type,
            'objects': objects,
            'locations': locations,
            'raw': command
        }
```

### آواز سے چلنے والا ROS 2 نوڈ

```python
# capstone/voice_vla_node.py
"""آواز سے کنٹرول VLA ROS 2 نوڈ"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from cv_bridge import CvBridge
import torch

class VoiceVLANode(Node):
    """کچن روبوٹ کے لیے آواز کنٹرول والا ROS 2 نوڈ"""

    def __init__(self):
        super().__init__('voice_vla_node')

        # آواز کنٹرول
        self.voice = VoiceController(wake_word="ارے شیف")
        self.parser = VoiceCommandParser()

        # VLA ماڈل
        self.model = KitchenVLA()
        self.model.load_state_dict(torch.load('kitchen_vla.pt'))
        self.model.eval()

        # ROS سیٹ اپ
        self.bridge = CvBridge()
        self.current_image = None
        self.task_active = False

        # سبسکرائبرز
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # پبلشرز
        self.action_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory', 10
        )
        self.status_pub = self.create_publisher(
            String, '/robot_status', 10
        )

        # آواز کنٹرول شروع کریں
        self.voice.start()
        self.voice.speak("کچن اسسٹنٹ تیار ہے۔ حکم دینے کے لیے 'ارے شیف' کہیں۔")

        # آواز پروسیسنگ ٹائمر
        self.create_timer(0.1, self.process_voice)

        self.get_logger().info('Voice VLA نوڈ شروع ہو گیا')

    def image_callback(self, msg):
        """موجودہ کیمرہ تصویر محفوظ کریں"""
        self.current_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def process_voice(self):
        """آنے والے آواز کے احکامات پروسیس کریں"""
        command_text = self.voice.get_command(timeout=0.05)

        if command_text is None:
            return

        if command_text == "":
            # ویک ورڈ ملا، کمانڈ کا انتظار
            self.voice.speak("جی ہاں؟ کیا کروں؟")
            return

        self.get_logger().info(f'آواز کمانڈ: {command_text}')

        # کمانڈ پارس کریں
        parsed = self.parser.parse(command_text)
        self.handle_command(parsed)

    def handle_command(self, parsed):
        """پارس شدہ آواز کمانڈ سنبھالیں"""
        if parsed['type'] == 'stop':
            self.task_active = False
            self.voice.speak("رک رہا ہوں")
            return

        if parsed['type'] == 'status':
            self.report_status()
            return

        if parsed['type'] == 'unknown':
            self.voice.speak("معذرت، سمجھ نہیں آیا۔ 'سیب اٹھاؤ' یا 'پیالے میں رکھو' کہیں۔")
            return

        # اٹھانے/رکھنے/ہلنے کے احکامات
        if parsed['type'] == 'pick':
            if not parsed['objects']:
                self.voice.speak("کیا اٹھاؤں؟")
                return
            obj = parsed['objects'][0]
            self.voice.confirm_action(f"{obj} اٹھا رہا ہوں")
            self.execute_vla_task(f"{obj} اٹھاؤ")

        elif parsed['type'] == 'place':
            if not parsed['locations']:
                self.voice.speak("کہاں رکھوں؟")
                return
            loc = parsed['locations'][0]
            self.voice.confirm_action(f"{loc} میں رکھ رہا ہوں")
            self.execute_vla_task(f"{loc} میں رکھو")

        elif parsed['type'] == 'pour':
            self.voice.confirm_action("ڈال رہا ہوں")
            self.execute_vla_task("برتن میں ڈالو")

    def execute_vla_task(self, instruction):
        """آواز فیڈبیک کے ساتھ VLA کام انجام دیں"""
        self.task_active = True

        while self.task_active:
            if self.current_image is None:
                continue

            # VLA سے ایکشن حاصل کریں
            action = self.model(
                self.preprocess_image(self.current_image),
                instruction
            )

            # ایکشن پبلش کریں
            traj_msg = self.action_to_trajectory(action)
            self.action_pub.publish(traj_msg)

            # تکمیل کی جانچ
            if self.detect_task_complete():
                self.task_active = False
                self.voice.speak("ہو گیا!")
                break

            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

    def report_status(self):
        """آواز سے روبوٹ کی حالت بتائیں"""
        status = "میں حکم کے لیے تیار ہوں۔"
        if self.task_active:
            status = "میں ابھی کام کر رہا ہوں۔"
        self.voice.speak(status)

    def preprocess_image(self, image):
        """ماڈل کے لیے تصویر پری پروسیس کریں"""
        # سائز بدلیں، نارملائز کریں، ٹینسر میں تبدیل کریں
        import torchvision.transforms as T
        transform = T.Compose([
            T.ToPILImage(),
            T.Resize((224, 224)),
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406],
                       std=[0.229, 0.224, 0.225])
        ])
        return transform(image).unsqueeze(0)

    def action_to_trajectory(self, action):
        """ماڈل آؤٹ پٹ کو ROS ٹریجیکٹری میسج میں تبدیل کریں"""
        msg = JointTrajectory()
        # ... تبدیلی کی منطق
        return msg

    def detect_task_complete(self):
        """جانچیں کہ موجودہ کام مکمل ہے"""
        # نفاذ کام کی قسم پر منحصر
        return False


def main(args=None):
    rclpy.init(args=args)
    node = VoiceVLANode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.voice.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### کثیر زبان سپورٹ (انگریزی/اردو)

```python
# capstone/multilingual_voice.py
"""اردو سپورٹ کے ساتھ کثیر زبان آواز کنٹرول"""

class MultilingualVoiceController(VoiceController):
    """انگریزی اور اردو سپورٹ کرنے والا آواز کنٹرولر"""

    WAKE_WORDS = {
        'en': 'hey chef',
        'ur': 'ارے شیف',
    }

    RESPONSES = {
        'en': {
            'ready': "Kitchen assistant ready.",
            'listening': "Yes? What would you like me to do?",
            'picking': "Picking up the {}",
            'placing': "Placing in the {}",
            'done': "Done!",
            'unknown': "Sorry, I didn't understand that.",
        },
        'ur': {
            'ready': "کچن اسسٹنٹ تیار ہے۔",
            'listening': "جی ہاں؟ کیا کروں؟",
            'picking': "{} اٹھا رہا ہوں",
            'placing': "{} میں رکھ رہا ہوں",
            'done': "ہو گیا!",
            'unknown': "معذرت، سمجھ نہیں آیا۔",
        }
    }

    def __init__(self, default_lang='ur'):
        super().__init__()
        self.current_lang = default_lang
        self.whisper = whisper.load_model("medium")  # بہتر کثیر زبان

    def _transcribe(self, audio):
        """زبان کی شناخت کے ساتھ ٹرانسکرائب کریں"""
        result = self.whisper.transcribe(audio)
        detected_lang = result.get("language", "ur")

        # سپورٹڈ زبانوں میں میپ کریں
        if detected_lang in ['ur', 'hi']:  # اردو/ہندی
            self.current_lang = 'ur'
        else:
            self.current_lang = 'en'

        text = result["text"].strip().lower()

        # موجودہ زبان میں ویک ورڈ کی جانچ
        wake_word = self.WAKE_WORDS.get(self.current_lang, 'ارے شیف')
        if wake_word in text:
            self.is_awake = True
            parts = text.split(wake_word)
            return parts[-1].strip() if len(parts) > 1 else ""

        return text if self.is_awake else None

    def speak(self, key, *args):
        """موجودہ زبان میں بولیں"""
        responses = self.RESPONSES.get(self.current_lang, self.RESPONSES['ur'])
        text = responses.get(key, key)
        if args:
            text = text.format(*args)

        print(f"روبوٹ ({self.current_lang}): {text}")
        tts = gTTS(text=text, lang=self.current_lang)
        # ... آڈیو چلائیں
```

---

## مرحلہ 5: ROS 2 ڈیپلائمنٹ

### معیاری ROS 2 نوڈ (ٹیکسٹ ان پٹ)

```python
# capstone/vla_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

class VLANode(Node):
    def __init__(self):
        super().__init__('vla_node')

        # ماڈل لوڈ کریں
        self.model = KitchenVLA()
        self.model.load_state_dict(torch.load('kitchen_vla.pt'))
        self.model.eval()

        # سبسکرائبرز
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.instruction_sub = self.create_subscription(
            String, '/instruction', self.instruction_callback, 10
        )

        # پبلشر
        self.action_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory', 10
        )

        self.current_instruction = None
        self.current_image = None

    def instruction_callback(self, msg):
        self.current_instruction = msg.data
        self.execute_task()

    def execute_task(self):
        while not self.task_complete():
            # ماڈل سے ایکشن حاصل کریں
            action = self.model(self.current_image, self.current_instruction)

            # ایکشن پبلش کریں
            traj_msg = self.action_to_trajectory(action)
            self.action_pub.publish(traj_msg)

            time.sleep(0.1)
```

## مرحلہ 6: ٹیسٹنگ

### ٹیسٹ منظرنامے

```yaml
# capstone/test_scenarios.yaml
scenarios:
  # ٹیکسٹ پر مبنی منظرنامے
  - name: "سادہ پک اینڈ پلیس"
    instruction: "سیب کو پیالے میں رکھو"
    expected_outcome: apple_in_bowl

  - name: "مقامی استدلال"
    instruction: "کپ کو پلیٹ کے قریب رکھو"
    expected_outcome: cup_near_plate

  - name: "ملٹی سٹیپ ٹاسک"
    instruction: "کپوں کو اوپر رکھو"
    expected_outcome: cups_stacked

  - name: "مبہم حوالہ"
    instruction: "پھل اٹھاؤ"
    expected_outcome: clarification_requested

  # آواز پر مبنی منظرنامے
  - name: "آواز ویک ورڈ"
    voice_input: "ارے شیف"
    expected_outcome: robot_listening

  - name: "آواز پک کمانڈ"
    voice_input: "ارے شیف، سرخ سیب اٹھاؤ"
    expected_outcome: apple_picked
    voice_confirmation: "سرخ سیب اٹھا رہا ہوں"

  - name: "آواز پلیس کمانڈ"
    voice_input: "پیالے میں رکھو"
    expected_outcome: apple_in_bowl
    voice_confirmation: "پیالے میں رکھ رہا ہوں"

  - name: "آواز سٹاپ کمانڈ"
    voice_input: "رکو"
    expected_outcome: robot_stopped
    voice_confirmation: "رک رہا ہوں"

  - name: "آواز سٹیٹس سوال"
    voice_input: "کیا نظر آتا ہے؟"
    expected_outcome: scene_description
    voice_confirmation: "مجھے نظر آ رہا ہے..."

  # کثیر زبان منظرنامے (انگریزی)
  - name: "انگریزی ویک ورڈ"
    voice_input: "Hey Chef"
    expected_outcome: robot_listening_english

  - name: "انگریزی پک کمانڈ"
    voice_input: "Pick up the apple"
    expected_outcome: apple_picked
    voice_confirmation: "Picking up the apple"
```

### تشخیص

```python
def evaluate_system(vla_node, scenarios):
    results = []

    for scenario in scenarios:
        # ماحول ری سیٹ کریں
        reset_scene()

        # ہدایات عمل کریں (آواز یا ٹیکسٹ)
        if 'voice_input' in scenario:
            success = vla_node.process_voice_test(scenario['voice_input'])
            # آواز تصدیق کی تصدیق کریں
            if 'voice_confirmation' in scenario:
                confirmed = verify_voice_output(scenario['voice_confirmation'])
                success = success and confirmed
        else:
            success = vla_node.execute(scenario['instruction'])

        # نتیجہ چیک کریں
        outcome = check_outcome(scenario['expected_outcome'])

        results.append({
            'scenario': scenario['name'],
            'success': success and outcome,
        })

    # رپورٹ
    success_rate = sum(r['success'] for r in results) / len(results)
    print(f"کامیابی کی شرح: {success_rate:.1%}")

    # زمرے کے لحاظ سے تفصیل
    voice_tests = [r for r in results if 'آواز' in r['scenario'].lower()]
    text_tests = [r for r in results if 'آواز' not in r['scenario'].lower()]

    print(f"آواز کے احکامات: {sum(r['success'] for r in voice_tests)}/{len(voice_tests)}")
    print(f"ٹیکسٹ کے احکامات: {sum(r['success'] for r in text_tests)}/{len(text_tests)}")
```

## ڈیلیوریبلز چیک لسٹ

### بنیادی تقاضے
- [ ] ایزیک سیم کچن انوائرنمنٹ اشیاء کے ساتھ
- [ ] تربیت یافتہ VLA ماڈل (کم از کم 80% درستگی)
- [ ] ROS 2 ڈیپلائمنٹ نوڈ

### آواز کنٹرول تقاضے
- [ ] Whisper پر مبنی اسپیچ ریکگنیشن
- [ ] ویک ورڈ ایکٹیویشن ("ارے شیف")
- [ ] آواز کمانڈ پارسنگ
- [ ] ٹیکسٹ سے آواز فیڈبیک
- [ ] آواز کنٹرول ROS 2 نوڈ

### ڈیمو تقاضے
- [ ] ڈیمو ویڈیو: 5+ ٹیکسٹ پر مبنی کام
- [ ] ڈیمو ویڈیو: 5+ آواز سے کنٹرول کام
- [ ] کثیر زبان ڈیمو (اردو + انگریزی)
- [ ] غلطی ہینڈلنگ اور وضاحت ڈیمو

### دستاویزات
- [ ] تکنیکی رپورٹ
- [ ] آواز کمانڈ حوالہ گائیڈ
- [ ] تنصیب اور سیٹ اپ ہدایات

## مثالی ڈیمو اسکرپٹ

```
ڈیمو: آواز سے کنٹرول کچن اسسٹنٹ روبوٹ
================================================

[روبوٹ شروع ہوتا ہے]
روبوٹ: "کچن اسسٹنٹ تیار ہے۔ حکم دینے کے لیے 'ارے شیف' کہیں۔"

[صارف بولتا ہے]
صارف: "ارے شیف"
روبوٹ: "جی ہاں؟ کیا کروں؟"

[صارف حکم دیتا ہے]
صارف: "سرخ سیب اٹھاؤ"
روبوٹ: "سرخ سیب اٹھا رہا ہوں"
[روبوٹ سیب کی طرف جاتا ہے، اٹھاتا ہے]
روبوٹ: "لے لیا!"

[صارف اگلا حکم دیتا ہے]
صارف: "پیالے میں رکھو"
روبوٹ: "پیالے میں رکھ رہا ہوں"
[روبوٹ پیالے کی طرف جاتا ہے، سیب رکھتا ہے]
روبوٹ: "ہو گیا!"

[صارف سوال پوچھتا ہے]
صارف: "کیا نظر آتا ہے؟"
روبوٹ: "مجھے نظر آ رہا ہے: سیب والا پیالہ، کپ، اور پلیٹ۔"

[صارف انگریزی میں حکم دیتا ہے]
صارف: "Hey Chef, pick up the cup"
روبوٹ: "Picking up the cup"
[روبوٹ کپ اٹھاتا ہے]
روبوٹ: "Got it!"

[صارف روبوٹ روکتا ہے]
صارف: "رکو"
روبوٹ: "رک رہا ہوں"
[روبوٹ تمام حرکت روک دیتا ہے]

[ڈیمو مکمل]
روبوٹ: "کچن اسسٹنٹ استعمال کرنے کا شکریہ!"
```

## مبارک ہو!

آپ نے فزیکل AI اور ہیومینائڈ روبوٹکس کورس مکمل کر لیا! اب آپ کے پاس یہ مہارتیں ہیں:

- ROS 2 روبوٹ سسٹمز بنانا
- سمیولیشن میں ڈیجیٹل ٹوئنز بنانا
- AI سے چلنے والا پرسیپشن ڈیپلائی کرنا
- ویژن-لینگویج-ایکشن روبوٹس لاگو کرنا
- **Whisper کے ساتھ آواز سے کنٹرول روبوٹس بنانا**
- **کثیر زبان روبوٹ انٹرفیسز بنانا**

### آگے کیا ہے؟

- اوپن سورس روبوٹکس پروجیکٹس میں حصہ لیں
- ROS کمیونٹی میں شامل ہوں
- امبوڈیڈ AI پر تحقیقی مقالے دریافت کریں
- اپنا آواز سے کنٹرول روبوٹ بنائیں!
- دوسری زبانوں اور بولیوں کے ساتھ تجربہ کریں

### وسائل

- [OpenAI Whisper](https://github.com/openai/whisper) - اسپیچ ریکگنیشن
- [OpenVLA](https://openvla.github.io/) - ویژن-لینگویج-ایکشن ماڈلز
- [ROS 2 Documentation](https://docs.ros.org/en/humble/) - روبوٹ آپریٹنگ سسٹم
- [این ویڈیا ایزیک](https://developer.nvidia.com/isaac-sim) - روبوٹ سمیولیشن

---

**ہمارے ساتھ سیکھنے کا شکریہ!** 🤖🎓

*اب جائیں اور حیرت انگیز آواز سے کنٹرول روبوٹس بنائیں!*

