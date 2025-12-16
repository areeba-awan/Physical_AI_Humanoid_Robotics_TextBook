---
sidebar_position: 3
title: "4.3 روبوٹکس کے لیے لینگویج ماڈلز"
description: روبوٹ ہدایات کے لیے قدرتی زبان کی سمجھ اور آواز کنٹرول
keywords: [LLM, لینگویج ماڈل, ہدایات کی پیروی, روبوٹکس, آواز کنٹرول, Whisper, اسپیچ ٹو ٹیکسٹ]
---

# باب 4.3: روبوٹکس کے لیے لینگویج ماڈلز

## سیکھنے کے مقاصد

- ہدایات کی پارسنگ کے لیے LLMs استعمال کریں
- ٹاسک ڈیکمپوزیشن لاگو کریں
- زبان کو روبوٹ ایکشنز سے جوڑیں
- ابہام والی ہدایات کو سنبھالیں
- **Whisper اسپیچ ٹو ٹیکسٹ کے ساتھ آواز کنٹرول**
- **ریئل ٹائم آواز کمانڈ پائپ لائنز بنائیں**

## روبوٹ ہدایات کے لیے LLMs

```
┌─────────────────────────────────────────────────────────────┐
│               زبان → روبوٹ پائپ لائن                        │
│                                                              │
│  "میرے لیے سینڈوچ بناؤ"                                     │
│          │                                                   │
│          ▼                                                   │
│  ┌───────────────────────────────────────┐                  │
│  │           LLM پلانر                    │                  │
│  │   1. کچن میں جاؤ                       │                  │
│  │   2. فریج کھولو                        │                  │
│  │   3. روٹی، پنیر، سلاد لو              │                  │
│  │   4. سینڈوچ بناؤ                       │                  │
│  │   5. صارف کو دو                        │                  │
│  └───────────────────────────────────────┘                  │
│          │                                                   │
│          ▼                                                   │
│  ┌───────────────────────────────────────┐                  │
│  │        ایکشن پرائمٹوز                  │                  │
│  │   navigate(kitchen)                    │                  │
│  │   open(fridge)                         │                  │
│  │   pick(bread)                          │                  │
│  │   ...                                  │                  │
│  └───────────────────────────────────────┘                  │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## LLMs کے ساتھ ٹاسک ڈیکمپوزیشن

```python
from openai import OpenAI

client = OpenAI()

def decompose_task(instruction):
    response = client.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": """آپ روبوٹ ٹاسک پلانر ہیں۔
            اعلیٰ سطحی ہدایات کو ایٹامک ایکشنز میں توڑیں۔
            دستیاب ایکشنز: navigate(location), pick(object), place(object, location),
            open(container), close(container), pour(from, to)"""},
            {"role": "user", "content": instruction}
        ]
    )
    return response.choices[0].message.content

# مثال
tasks = decompose_task("میز پر گلاس میں پانی ڈالو")
# آؤٹ پٹ:
# 1. navigate(kitchen)
# 2. pick(water_bottle)
# 3. navigate(table)
# 4. pour(water_bottle, glass)
# 5. place(water_bottle, table)
```

## ہدایات کی پارسنگ

```python
import re

def parse_instruction(text):
    """ہدایات سے ایکشن، آبجیکٹ، اور لوکیشن نکالیں"""

    patterns = {
        'pick': r'pick up (?:the )?(\w+)',
        'place': r'put (?:the )?(\w+) (?:on|in|near) (?:the )?(\w+)',
        'move': r'go to (?:the )?(\w+)',
    }

    for action, pattern in patterns.items():
        match = re.search(pattern, text.lower())
        if match:
            return {
                'action': action,
                'params': match.groups()
            }

    return None

# مثال
result = parse_instruction("Pick up the red apple")
# {'action': 'pick', 'params': ('red apple',)}
```

## زبان کو آبجیکٹس سے جوڑنا

```python
class LanguageGrounder:
    def __init__(self):
        self.clip_encoder = CLIPEncoder()

    def ground_object(self, description, detections):
        """تفصیل سے ملتا آبجیکٹ تلاش کریں"""
        text_features = self.clip_encoder.encode_text(description)

        best_match = None
        best_score = -1

        for det in detections:
            img_features = self.clip_encoder.encode_image(det.crop)
            score = torch.cosine_similarity(text_features, img_features)

            if score > best_score:
                best_score = score
                best_match = det

        return best_match

# مثال
grounder = LanguageGrounder()
target = grounder.ground_object("سرخ سیب", detected_objects)
```

## ابہام کو سنبھالنا

```python
def clarify_instruction(instruction, scene_objects):
    """جب ہدایات مبہم ہوں تو وضاحت طلب کریں"""

    # مبہم حوالوں کی جانچ کریں
    referenced_objects = extract_objects(instruction)

    for ref in referenced_objects:
        matches = find_matches(ref, scene_objects)

        if len(matches) > 1:
            return {
                'needs_clarification': True,
                'question': f"کون سا {ref}؟ میں دیکھ رہا ہوں {describe_options(matches)}"
            }

    return {'needs_clarification': False}
```

---

## آواز کنٹرول انٹیگریشن

آواز کنٹرول ہینڈز فری روبوٹ آپریشن ممکن بناتا ہے، جو حقیقی دنیا کی ایپلیکیشنز کے لیے ضروری ہے جہاں آپریٹرز دوسرے کاموں میں مصروف ہو سکتے ہیں۔

```
┌─────────────────────────────────────────────────────────────┐
│              آواز → روبوٹ کنٹرول پائپ لائن                  │
│                                                              │
│  ┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────┐    │
│  │  مائک   │──▶│ Whisper │──▶│   LLM   │──▶│ روبوٹ  │    │
│  │ ان پٹ  │   │  STT    │   │ پلانر  │   │ کنٹرول │    │
│  └─────────┘   └─────────┘   └─────────┘   └─────────┘    │
│       │             │             │             │          │
│       ▼             ▼             ▼             ▼          │
│   آڈیو سٹریم   "سیب اٹھاؤ"   ٹاسک پلان    ایکشنز        │
│   16kHz مونو                [pick,move]   عمل میں        │
│                                                            │
└─────────────────────────────────────────────────────────────┘
```

### Whisper کے ساتھ اسپیچ ٹو ٹیکسٹ

OpenAI کا Whisper روبوٹ آواز کنٹرول کے لیے جدید ترین اسپیچ ریکگنیشن فراہم کرتا ہے:

```python
import whisper
import numpy as np
import sounddevice as sd

class WhisperSTT:
    """OpenAI Whisper استعمال کرتے ہوئے اسپیچ ٹو ٹیکسٹ"""

    def __init__(self, model_size="base"):
        # ماڈل سائز: tiny, base, small, medium, large
        self.model = whisper.load_model(model_size)
        self.sample_rate = 16000

    def transcribe_audio(self, audio_data):
        """آڈیو numpy array کو ٹیکسٹ میں تبدیل کریں"""
        # Whisper float32 آڈیو چاہتا ہے [-1, 1] میں نارملائز
        audio = audio_data.astype(np.float32) / 32768.0

        result = self.model.transcribe(
            audio,
            language="en",
            task="transcribe"
        )
        return result["text"].strip()

    def record_and_transcribe(self, duration=5):
        """مائیکروفون سے ریکارڈ کریں اور ٹرانسکرائب کریں"""
        print(f"{duration} سیکنڈ کے لیے ریکارڈنگ...")

        audio = sd.rec(
            int(duration * self.sample_rate),
            samplerate=self.sample_rate,
            channels=1,
            dtype=np.int16
        )
        sd.wait()

        return self.transcribe_audio(audio.flatten())

# استعمال
stt = WhisperSTT(model_size="base")
command = stt.record_and_transcribe(duration=3)
print(f"آپ نے کہا: {command}")
# آؤٹ پٹ: "آپ نے کہا: pick up the red apple"
```

### ریئل ٹائم سٹریمنگ آواز ریکگنیشن

جوابدہ روبوٹ کنٹرول کے لیے، سٹریمنگ ریکگنیشن استعمال کریں:

```python
import threading
import queue
from collections import deque

class StreamingVoiceControl:
    """سٹریمنگ ریکگنیشن کے ساتھ ریئل ٹائم آواز کنٹرول"""

    def __init__(self):
        self.stt = WhisperSTT(model_size="tiny")  # ریئل ٹائم کے لیے تیز ماڈل
        self.audio_queue = queue.Queue()
        self.is_listening = False
        self.buffer = deque(maxlen=int(16000 * 3))  # 3 سیکنڈ بفر

    def audio_callback(self, indata, frames, time, status):
        """آڈیو سٹریم کے لیے کال بیک"""
        if self.is_listening:
            self.audio_queue.put(indata.copy())

    def start_listening(self):
        """آڈیو سٹریم شروع کریں"""
        self.is_listening = True
        self.stream = sd.InputStream(
            samplerate=16000,
            channels=1,
            dtype=np.int16,
            callback=self.audio_callback,
            blocksize=1024
        )
        self.stream.start()

        # پروسیسنگ تھریڈ شروع کریں
        self.process_thread = threading.Thread(target=self._process_audio)
        self.process_thread.start()

    def _process_audio(self):
        """آڈیو چنکس پروسیس کریں اور کمانڈز کا پتہ لگائیں"""
        while self.is_listening:
            try:
                chunk = self.audio_queue.get(timeout=0.1)
                self.buffer.extend(chunk.flatten())

                # آواز سرگرمی کی جانچ کریں
                if self._detect_voice_activity(chunk):
                    # بفر ٹرانسکرائب کریں
                    audio = np.array(self.buffer)
                    text = self.stt.transcribe_audio(audio)

                    if text:
                        self._handle_command(text)
                        self.buffer.clear()

            except queue.Empty:
                continue

    def _detect_voice_activity(self, audio, threshold=500):
        """سادہ آواز سرگرمی کا پتہ لگانا"""
        return np.abs(audio).mean() > threshold

    def _handle_command(self, text):
        """پہچانی گئی کمانڈ پروسیس کریں"""
        print(f"کمانڈ موصول: {text}")
        # روبوٹ کنٹرولر کو بھیجیں

    def stop_listening(self):
        """آڈیو سٹریم بند کریں"""
        self.is_listening = False
        self.stream.stop()
```

### ویک ورڈ ڈیٹیکشن

روبوٹ کو صرف اس وقت ایکٹیویٹ کرنے کے لیے ویک ورڈ استعمال کریں جب مخاطب کیا جائے:

```python
import pvporcupine
import struct

class WakeWordDetector:
    """Porcupine استعمال کرتے ہوئے ویک ورڈ ڈیٹیکشن"""

    def __init__(self, wake_words=["jarvis", "robot"]):
        self.porcupine = pvporcupine.create(
            keywords=wake_words
        )
        self.sample_rate = self.porcupine.sample_rate
        self.frame_length = self.porcupine.frame_length

    def process_audio(self, audio_frame):
        """چیک کریں کہ ویک ورڈ کا پتہ چلا"""
        pcm = struct.unpack_from(
            "h" * self.frame_length,
            audio_frame
        )

        keyword_index = self.porcupine.process(pcm)

        if keyword_index >= 0:
            return True  # ویک ورڈ کا پتہ چلا
        return False

    def cleanup(self):
        self.porcupine.delete()


class VoiceControlledRobot:
    """مکمل آواز کنٹرول روبوٹ سسٹم"""

    def __init__(self, robot_interface):
        self.robot = robot_interface
        self.wake_detector = WakeWordDetector(["ارے روبوٹ"])
        self.stt = WhisperSTT(model_size="small")
        self.is_awake = False
        self.awake_timeout = 10  # سیکنڈز

    def run(self):
        """آواز کنٹرول کا مین لوپ"""
        print("ایکٹیویٹ کرنے کے لیے 'ارے روبوٹ' کہیں...")

        with sd.InputStream(
            samplerate=16000,
            channels=1,
            dtype=np.int16,
            blocksize=512
        ) as stream:
            while True:
                audio, _ = stream.read(512)

                if not self.is_awake:
                    # ویک ورڈ کے لیے سنیں
                    if self.wake_detector.process_audio(audio.tobytes()):
                        print("روبوٹ ایکٹیویٹ! کمانڈ کے لیے سن رہا ہوں...")
                        self.is_awake = True
                        self._play_activation_sound()
                else:
                    # کمانڈ ریکارڈ کریں
                    command = self.stt.record_and_transcribe(duration=5)
                    print(f"کمانڈ: {command}")

                    # کمانڈ عمل میں لائیں
                    self._execute_voice_command(command)
                    self.is_awake = False

    def _execute_voice_command(self, command):
        """آواز کمانڈ پارس اور عمل کریں"""
        # ٹاسک توڑنے کے لیے LLM استعمال کریں
        tasks = decompose_task(command)

        for task in tasks:
            action = parse_instruction(task)
            if action:
                self.robot.execute(action)

    def _play_activation_sound(self):
        """یہ بتانے کے لیے آواز چلائیں کہ روبوٹ سن رہا ہے"""
        # ایک مختصر بیپ چلائیں
        pass
```

### آواز فیڈبیک اور تصدیق

دو طرفہ آواز مواصلات فعال کریں:

```python
from gtts import gTTS
import pygame
import io

class VoiceFeedback:
    """روبوٹ جوابات کے لیے ٹیکسٹ ٹو اسپیچ"""

    def __init__(self):
        pygame.mixer.init()

    def speak(self, text):
        """ٹیکسٹ کو اسپیچ میں تبدیل کریں اور چلائیں"""
        tts = gTTS(text=text, lang='ur')

        # میموری بفر میں محفوظ کریں
        fp = io.BytesIO()
        tts.write_to_fp(fp)
        fp.seek(0)

        # آڈیو چلائیں
        pygame.mixer.music.load(fp)
        pygame.mixer.music.play()

        while pygame.mixer.music.get_busy():
            pygame.time.wait(100)

    def confirm_action(self, action):
        """ایکشن کی تصدیق بولیں"""
        confirmations = {
            'pick': f"{action['params'][0]} اٹھا رہا ہوں",
            'place': f"{action['params'][0]} کو {action['params'][1]} پر رکھ رہا ہوں",
            'move': f"{action['params'][0]} کی طرف جا رہا ہوں"
        }

        message = confirmations.get(
            action['action'],
            f"{action['action']} عمل میں لا رہا ہوں"
        )
        self.speak(message)


class InteractiveVoiceRobot:
    """آواز ان پٹ اور آؤٹ پٹ والا روبوٹ"""

    def __init__(self, robot):
        self.robot = robot
        self.stt = WhisperSTT()
        self.tts = VoiceFeedback()
        self.grounder = LanguageGrounder()

    def process_command(self, audio):
        """فیڈبیک کے ساتھ مکمل آواز کمانڈ پائپ لائن"""
        # 1. ٹرانسکرائب کریں
        command = self.stt.transcribe_audio(audio)
        self.tts.speak(f"میں نے سنا: {command}")

        # 2. ہدایات پارس کریں
        action = parse_instruction(command)

        if action is None:
            self.tts.speak("معذرت، میں وہ کمانڈ نہیں سمجھا")
            return

        # 3. ابہام کی جانچ کریں
        scene_objects = self.robot.get_detected_objects()
        clarity = clarify_instruction(command, scene_objects)

        if clarity['needs_clarification']:
            self.tts.speak(clarity['question'])
            # وضاحت کا انتظار کریں
            clarification = self.stt.record_and_transcribe(duration=5)
            command = f"{command}، خاص طور پر {clarification}"
            action = parse_instruction(command)

        # 4. تصدیق کریں اور عمل میں لائیں
        self.tts.confirm_action(action)
        self.robot.execute(action)
        self.tts.speak("ہو گیا!")
```

### ROS 2 آواز کنٹرول نوڈ

آواز کنٹرول کو ROS 2 کے ساتھ یکجا کریں:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np

class VoiceControlNode(Node):
    """آواز کنٹرول روبوٹ کے لیے ROS 2 نوڈ"""

    def __init__(self):
        super().__init__('voice_control_node')

        # پبلشرز
        self.cmd_pub = self.create_publisher(String, '/robot_command', 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # آواز کنٹرول کمپوننٹس
        self.stt = WhisperSTT(model_size="small")
        self.wake_detector = WakeWordDetector(["ارے روبوٹ"])

        # پیرامیٹرز
        self.declare_parameter('wake_word', 'ارے روبوٹ')
        self.declare_parameter('listen_duration', 5.0)

        # آڈیو پروسیسنگ کے لیے ٹائمر
        self.create_timer(0.1, self.process_audio)

        self.get_logger().info('آواز کنٹرول نوڈ شروع ہو گیا')

    def process_audio(self):
        """آنے والی آڈیو کو کمانڈز کے لیے پروسیس کریں"""
        # مختصر آڈیو سیگمنٹ ریکارڈ کریں
        audio = self.record_audio(duration=0.5)

        # ویک ورڈ کی جانچ کریں
        if self.wake_detector.process_audio(audio.tobytes()):
            self.get_logger().info('ویک ورڈ کا پتہ چلا!')

            # مکمل کمانڈ ریکارڈ کریں
            command_audio = self.record_audio(duration=5.0)
            command = self.stt.transcribe_audio(command_audio)

            self.get_logger().info(f'کمانڈ: {command}')
            self.process_voice_command(command)

    def process_voice_command(self, command):
        """آواز کمانڈ کو روبوٹ ایکشن میں تبدیل کریں"""
        command_lower = command.lower()

        # براہ راست حرکت کی کمانڈز
        if 'آگے' in command_lower or 'forward' in command_lower:
            self.send_velocity(linear=0.5)
        elif 'پیچھے' in command_lower or 'back' in command_lower:
            self.send_velocity(linear=-0.5)
        elif 'بائیں' in command_lower or 'left' in command_lower:
            self.send_velocity(angular=0.5)
        elif 'دائیں' in command_lower or 'right' in command_lower:
            self.send_velocity(angular=-0.5)
        elif 'رکو' in command_lower or 'stop' in command_lower:
            self.send_velocity(linear=0.0, angular=0.0)
        else:
            # پیچیدہ کمانڈ LLM پروسیسنگ کے لیے بھیجیں
            msg = String()
            msg.data = command
            self.cmd_pub.publish(msg)

    def send_velocity(self, linear=0.0, angular=0.0):
        """ولاسٹی کمانڈ پبلش کریں"""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.vel_pub.publish(twist)

    def record_audio(self, duration):
        """مائیکروفون سے آڈیو ریکارڈ کریں"""
        audio = sd.rec(
            int(duration * 16000),
            samplerate=16000,
            channels=1,
            dtype=np.int16
        )
        sd.wait()
        return audio.flatten()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## عملی لیب

### لیب 4.3A: ہدایات پارسر بنائیں

ایک سسٹم بنائیں جو:
1. قدرتی زبان کی ہدایات پارس کرے
2. پیچیدہ کاموں کو توڑے
3. آبجیکٹ حوالوں کو جوڑے
4. ضرورت پڑنے پر وضاحت طلب کرے

### لیب 4.3B: آواز کنٹرول روبوٹ

مکمل آواز کنٹرول سسٹم بنائیں:

```python
# lab_voice_control.py
"""
لیب 4.3B: آواز کنٹرول روبوٹ
Whisper اور ROS 2 استعمال کرتے ہوئے آواز کنٹرول روبوٹ بنائیں
"""

# مرحلہ 1: ڈیپینڈنسیز انسٹال کریں
# pip install openai-whisper sounddevice numpy pvporcupine gtts pygame

# مرحلہ 2: VoiceRobotLab کلاس لاگو کریں
class VoiceRobotLab:
    def __init__(self):
        # کمپوننٹس شروع کریں
        self.stt = WhisperSTT(model_size="base")
        self.tts = VoiceFeedback()

        # کمانڈ الفاظ کی فہرست
        self.commands = {
            'pick': self.handle_pick,
            'place': self.handle_place,
            'move': self.handle_move,
            'stop': self.handle_stop,
        }

    def run_demo(self):
        """انٹرایکٹو آواز کنٹرول ڈیمو چلائیں"""
        self.tts.speak("آواز کنٹرول تیار۔ کمانڈ دیں۔")

        while True:
            # کمانڈ ریکارڈ کریں
            print("\nسن رہا ہوں...")
            command = self.stt.record_and_transcribe(duration=5)
            print(f"آپ نے کہا: {command}")

            if 'باہر' in command.lower() or 'exit' in command.lower():
                self.tts.speak("خدا حافظ!")
                break

            # کمانڈ پروسیس کریں
            self.process_command(command)

    def process_command(self, command):
        """آواز کمانڈ پارس اور عمل کریں"""
        action = parse_instruction(command)

        if action and action['action'] in self.commands:
            self.tts.speak(f"{action['action']} عمل میں لا رہا ہوں")
            self.commands[action['action']](action['params'])
        else:
            self.tts.speak("کمانڈ نہیں پہچانی گئی۔ دوبارہ کوشش کریں۔")

    def handle_pick(self, params):
        print(f"اٹھا رہا ہوں: {params[0]}")

    def handle_place(self, params):
        print(f"{params[0]} کو {params[1]} پر رکھ رہا ہوں")

    def handle_move(self, params):
        print(f"جا رہا ہوں: {params[0]}")

    def handle_stop(self, params):
        print("روبوٹ رک رہا ہے")


# مرحلہ 3: لیب چلائیں
if __name__ == "__main__":
    lab = VoiceRobotLab()
    lab.run_demo()
```

**لیب کے مقاصد:**
1. Whisper اسپیچ ریکگنیشن سیٹ اپ کریں
2. ویک ورڈ ڈیٹیکشن لاگو کریں
3. آواز کمانڈ پارسر بنائیں
4. ٹیکسٹ ٹو اسپیچ فیڈبیک شامل کریں
5. 10+ مختلف آواز کمانڈز کے ساتھ ٹیسٹ کریں

**متوقع آؤٹ پٹ:**
```
آواز کنٹرول تیار۔ کمانڈ دیں۔

سن رہا ہوں...
آپ نے کہا: سرخ سیب اٹھاؤ
pick عمل میں لا رہا ہوں
اٹھا رہا ہوں: سرخ سیب

سن رہا ہوں...
آپ نے کہا: اسے پیالے میں رکھو
place عمل میں لا رہا ہوں
اسے پیالے پر رکھ رہا ہوں
```

## خلاصہ

- LLMs پیچیدہ ہدایات کو ایٹامک ایکشنز میں توڑتے ہیں
- پارسنگ ایکشن کی اقسام اور پیرامیٹرز نکالتی ہے
- CLIP زبان کو بصری آبجیکٹس سے جوڑنے کو ممکن بناتا ہے
- ابہام کو سنبھالنا مضبوطی بڑھاتا ہے
- **Whisper آواز کنٹرول کے لیے درست اسپیچ ٹو ٹیکسٹ فراہم کرتا ہے**
- **ویک ورڈ ڈیٹیکشن ہینڈز فری ایکٹیویشن ممکن بناتا ہے**
- **آواز فیڈبیک انٹرایکٹو روبوٹ تجربات بناتا ہے**
- **ROS 2 انٹیگریشن روبوٹ سسٹمز میں آواز کنٹرول ممکن بناتا ہے**

## مزید پڑھنے کے لیے

- [OpenAI Whisper دستاویزات](https://github.com/openai/whisper)
- [Porcupine ویک ورڈ انجن](https://picovoice.ai/platform/porcupine/)
- [ROS 2 آڈیو کامن](https://github.com/ros-drivers/audio_common)
- [SpeechBrain: اسپیچ AI ٹول کٹ](https://speechbrain.github.io/)

[باب 4.4 پر جائیں →](/docs/module-4-vla/chapter-4-action)
