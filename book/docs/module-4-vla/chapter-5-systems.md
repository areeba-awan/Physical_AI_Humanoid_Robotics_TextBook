---
sidebar_position: 5
title: "4.5 اینڈ ٹو اینڈ VLA سسٹمز"
description: آواز کنٹرول کے ساتھ مکمل VLA روبوٹ سسٹمز بنانا
keywords: [VLA, اینڈ ٹو اینڈ, سسٹم, ڈیپلائمنٹ, آواز کنٹرول, ملٹی موڈل]
---

# باب 4.5: اینڈ ٹو اینڈ VLA سسٹمز

## سیکھنے کے مقاصد

- VLA اجزاء کو مکمل سسٹم میں یکجا کریں
- VLA ماڈلز کو روبوٹ ہارڈویئر پر ڈیپلائی کریں
- حقیقی دنیا کے چیلنجز سنبھالیں
- انفرنس پرفارمنس آپٹیمائز کریں
- **آواز سے چلنے والے VLA سسٹمز بنائیں**
- **ملٹی موڈل انٹریکشن پائپ لائنز لاگو کریں**

## مکمل VLA پائپ لائن

```
┌─────────────────────────────────────────────────────────────┐
│           آواز کے ساتھ اینڈ ٹو اینڈ VLA سسٹم               │
│                                                              │
│   ┌─────────┐                              ┌─────────┐      │
│   │ کیمرہ   │──┐                      ┌───>│ روبوٹ   │      │
│   │ سٹریم  │  │                      │    │ کنٹرول  │      │
│   └─────────┘  │    ┌────────────┐    │    └─────────┘      │
│                │    │            │    │                      │
│   ┌─────────┐  ├───>│    VLA     │────┤    ┌─────────┐      │
│   │ کلائی   │──┤    │   ماڈل    │    │    │ گرپر   │      │
│   │ کیمرہ   │  │    │            │    └───>│ کنٹرول  │      │
│   └─────────┘  │    └────────────┘         └─────────┘      │
│                │          ▲                      │           │
│   ┌─────────┐  │          │                      │           │
│   │🎤 آواز  │──┘          │                      ▼           │
│   │ ان پٹ   │         فیڈبیک            ┌─────────┐       │
│   └─────────┘             │               │🔊 آواز  │       │
│        ▲                  └───────────────│ آؤٹ پٹ │       │
│        │                                  └─────────┘       │
│   [ویک ورڈ: "ارے روبوٹ"]                                  │
└─────────────────────────────────────────────────────────────┘
```

## سسٹم کا نفاذ

```python
class VLASystem:
    def __init__(self, model_path, robot_interface):
        self.model = OpenVLA.from_pretrained(model_path)
        self.robot = robot_interface
        self.cameras = CameraArray()

    def run(self, instruction):
        """ہدایات کو اینڈ ٹو اینڈ عمل میں لائیں"""
        while not self.task_complete():
            # موجودہ مشاہدہ حاصل کریں
            images = self.cameras.capture()

            # ایکشن کی پیش گوئی کریں
            action = self.model.predict(
                images=images,
                instruction=instruction
            )

            # ایکشن عمل میں لائیں
            self.robot.execute(action)

            # تکمیل کی جانچ کریں
            if self.detect_goal_reached(images, instruction):
                break

        return True
```

## حقیقی دنیا کے چیلنجز

### 1. لیٹنسی مینجمنٹ

```python
class LatencyOptimizer:
    def __init__(self, target_hz=10):
        self.target_period = 1.0 / target_hz
        self.action_buffer = deque(maxlen=5)

    def run_loop(self, vla_system):
        while True:
            start = time.time()

            # بفرڈ ایکشن استعمال کریں یا نیا پیش گوئی کریں
            if self.action_buffer:
                action = self.action_buffer.popleft()
            else:
                action = vla_system.predict()
                # اضافی پیش گوئیاں بفر کریں
                self.prefetch_actions(vla_system)

            vla_system.execute(action)

            # کنٹرول فریکوئنسی برقرار رکھیں
            elapsed = time.time() - start
            if elapsed < self.target_period:
                time.sleep(self.target_period - elapsed)
```

### 2. ناکامی کی بازیابی

```python
class FailureRecovery:
    def __init__(self):
        self.failure_detectors = [
            GraspFailureDetector(),     # گرپ ناکامی کا پتہ لگانے والا
            CollisionDetector(),         # ٹکراؤ کا پتہ لگانے والا
            StuckDetector(),             # پھنسنے کا پتہ لگانے والا
        ]

    def monitor_and_recover(self, vla_system):
        for detector in self.failure_detectors:
            if detector.detect():
                recovery_action = detector.get_recovery()
                vla_system.execute(recovery_action)
                return True
        return False
```

### 3. حفاظتی پابندیاں

```python
class SafetyFilter:
    def __init__(self, workspace_bounds, max_velocity):
        self.bounds = workspace_bounds
        self.max_vel = max_velocity

    def filter_action(self, action, current_state):
        # ورک اسپیس میں محدود کریں
        target_pos = current_state.position + action.delta_position
        target_pos = np.clip(target_pos, self.bounds[0], self.bounds[1])

        # رفتار محدود کریں
        action.delta_position = np.clip(
            action.delta_position,
            -self.max_vel,
            self.max_vel
        )

        return action
```

## ڈیپلائمنٹ آپٹیمائزیشن

### TensorRT آپٹیمائزیشن

```python
import tensorrt as trt

def optimize_model(model_path, output_path):
    """تیز انفرنس کے لیے TensorRT میں تبدیل کریں"""
    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    network = builder.create_network()

    # ONNX ماڈل پارس کریں
    parser = trt.OnnxParser(network, logger)
    with open(model_path, 'rb') as f:
        parser.parse(f.read())

    # انجن بنائیں
    config = builder.create_builder_config()
    config.set_flag(trt.BuilderFlag.FP16)  # FP16 استعمال کریں

    engine = builder.build_engine(network, config)

    with open(output_path, 'wb') as f:
        f.write(engine.serialize())
```

### کوانٹائزیشن

```python
from transformers import AutoModelForCausalLM
import torch

# ماڈل لوڈ کریں
model = AutoModelForCausalLM.from_pretrained("openvla/openvla-7b")

# 4-بٹ میں کوانٹائز کریں
model = model.to(torch.bfloat16)
model = torch.quantization.quantize_dynamic(
    model, {torch.nn.Linear}, dtype=torch.qint8
)
```

---

## آواز سے چلنے والا VLA سسٹم

آواز کنٹرول کو یکجا کرنا روبوٹ آپریشن کے لیے قدرتی، ہینڈز فری انٹرفیس بناتا ہے۔

### مکمل آواز-VLA آرکیٹیکچر

```
┌────────────────────────────────────────────────────────────────┐
│              آواز سے چلنے والا VLA آرکیٹیکچر                    │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                    ان پٹ پرت                             │  │
│  │  ┌────────┐  ┌────────┐  ┌────────┐  ┌────────────────┐  │  │
│  │  │  RGB   │  │ ڈیپتھ  │  │ کلائی  │  │   مائیکروفون   │  │  │
│  │  │ کیمرہ  │  │ کیمرہ  │  │ کیمرہ  │  │   (16kHz)      │  │  │
│  │  └───┬────┘  └───┬────┘  └───┬────┘  └───────┬────────┘  │  │
│  └──────┼──────────┼──────────┼────────────────┼────────────┘  │
│         │          │          │                │               │
│         ▼          ▼          ▼                ▼               │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                  پروسیسنگ پرت                             │  │
│  │                                                           │  │
│  │  ┌─────────────────────┐    ┌─────────────────────────┐  │  │
│  │  │   ویژن انکوڈر      │    │    Whisper STT          │  │  │
│  │  │       (ViT)         │    │    + ویک ورڈ           │  │  │
│  │  └──────────┬──────────┘    └───────────┬─────────────┘  │  │
│  │             │                           │                 │  │
│  │             └───────────┬───────────────┘                 │  │
│  │                         ▼                                 │  │
│  │              ┌──────────────────────┐                    │  │
│  │              │   VLA فیوژن ماڈل    │                    │  │
│  │              │  (ویژن + زبان)      │                    │  │
│  │              └──────────┬───────────┘                    │  │
│  │                         │                                 │  │
│  └─────────────────────────┼─────────────────────────────────┘  │
│                            ▼                                    │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                   آؤٹ پٹ پرت                              │  │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────────────┐  │  │
│  │  │   ایکشن   │  │  حفاظتی   │  │   آواز فیڈبیک     │  │  │
│  │  │  ڈیکوڈر  │──│   فلٹر    │──│   (TTS جواب)      │  │  │
│  │  └────────────┘  └────────────┘  └────────────────────┘  │  │
│  └──────────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────┘
```

### آواز-VLA سسٹم کا نفاذ

```python
import whisper
import numpy as np
import sounddevice as sd
from gtts import gTTS
import threading
import queue

class VoiceEnabledVLA:
    """آواز کنٹرول کے ساتھ مکمل VLA سسٹم"""

    def __init__(self, model_path, robot_interface):
        # VLA اجزاء
        self.vla_model = OpenVLA.from_pretrained(model_path)
        self.robot = robot_interface
        self.cameras = CameraArray()

        # آواز کے اجزاء
        self.whisper = whisper.load_model("small")
        self.audio_queue = queue.Queue()
        self.is_listening = True

        # حالت
        self.current_task = None
        self.task_active = False

        # حفاظت
        self.safety_filter = SafetyFilter(
            workspace_bounds=[[-0.5, -0.5, 0], [0.5, 0.5, 0.5]],
            max_velocity=0.1
        )

    def start(self):
        """آواز سے چلنے والا VLA سسٹم شروع کریں"""
        print("آواز سے چلنے والا VLA سسٹم شروع ہو رہا ہے...")
        print("'ارے روبوٹ' کہیں اور پھر اپنا حکم دیں")

        # آڈیو سننے والا تھریڈ شروع کریں
        self.audio_thread = threading.Thread(target=self._audio_listener)
        self.audio_thread.start()

        # مین کنٹرول لوپ شروع کریں
        self._control_loop()

    def _audio_listener(self):
        """مسلسل آواز کے احکامات سنیں"""
        with sd.InputStream(samplerate=16000, channels=1, dtype=np.int16) as stream:
            buffer = []
            silence_count = 0

            while self.is_listening:
                audio_chunk, _ = stream.read(1024)
                buffer.extend(audio_chunk.flatten())

                # آواز کی سرگرمی کا پتہ لگائیں
                if np.abs(audio_chunk).mean() > 500:
                    silence_count = 0
                else:
                    silence_count += 1

                # 1 سیکنڈ خاموشی کے بعد پروسیس کریں
                if silence_count > 15 and len(buffer) > 16000:
                    audio = np.array(buffer, dtype=np.float32) / 32768.0
                    self.audio_queue.put(audio)
                    buffer = []
                    silence_count = 0

    def _control_loop(self):
        """آواز اور VLA کو سنبھالنے والا مین کنٹرول لوپ"""
        while True:
            # نئے آواز کے حکم کی جانچ کریں
            try:
                audio = self.audio_queue.get_nowait()
                command = self._process_voice(audio)

                if command:
                    self._handle_voice_command(command)

            except queue.Empty:
                pass

            # فعال کام انجام دیں
            if self.task_active and self.current_task:
                self._execute_vla_step()

    def _process_voice(self, audio):
        """آڈیو کو ٹیکسٹ میں تبدیل کریں"""
        result = self.whisper.transcribe(audio, language="ur")
        text = result["text"].strip().lower()

        # ویک ورڈ کی جانچ کریں
        if "ارے روبوٹ" in text or "hey robot" in text:
            # ویک ورڈ کے بعد حکم نکالیں
            if "ارے روبوٹ" in text:
                command = text.split("ارے روبوٹ")[-1].strip()
            else:
                command = text.split("hey robot")[-1].strip()
            return command

        return None

    def _handle_voice_command(self, command):
        """آواز کا حکم پروسیس کریں"""
        print(f"حکم موصول ہوا: {command}")

        # کنٹرول کمانڈز کی جانچ کریں
        if "رکو" in command or "stop" in command:
            self.task_active = False
            self.robot.stop()
            self._speak("رک رہا ہوں")
            return

        if "وقفہ" in command or "pause" in command:
            self.task_active = False
            self._speak("رکا ہوا")
            return

        if "جاری" in command or "resume" in command:
            self.task_active = True
            self._speak("جاری ہے")
            return

        # نیا ٹاسک کمانڈ
        self.current_task = command
        self.task_active = True
        self._speak(f"کام شروع: {command}")

    def _execute_vla_step(self):
        """VLA کنٹرول کا ایک مرحلہ انجام دیں"""
        # مشاہدہ حاصل کریں
        images = self.cameras.capture()

        # ایکشن کی پیش گوئی کریں
        action = self.vla_model.predict(
            images=images,
            instruction=self.current_task
        )

        # حفاظتی فلٹر لگائیں
        current_state = self.robot.get_state()
        safe_action = self.safety_filter.filter_action(action, current_state)

        # ایکشن انجام دیں
        self.robot.execute(safe_action)

        # تکمیل کی جانچ کریں
        if self._detect_task_complete(images):
            self.task_active = False
            self._speak("کام مکمل!")

    def _detect_task_complete(self, images):
        """جانچیں کہ موجودہ کام مکمل ہے"""
        # VLA ماڈل یا علیحدہ تکمیل ڈیٹیکٹر استعمال کریں
        completion_score = self.vla_model.check_completion(
            images=images,
            instruction=self.current_task
        )
        return completion_score > 0.9

    def _speak(self, text):
        """ٹیکسٹ سے آواز آؤٹ پٹ"""
        tts = gTTS(text=text, lang='ur')
        tts.save("/tmp/response.mp3")
        # آڈیو چلائیں (پلیٹ فارم پر منحصر)
        import pygame
        pygame.mixer.init()
        pygame.mixer.music.load("/tmp/response.mp3")
        pygame.mixer.music.play()

    def stop(self):
        """سسٹم بند کریں"""
        self.is_listening = False
        self.task_active = False
        self.robot.stop()
```

### گفتگو پر مبنی روبوٹ کنٹرول

پیچیدہ کاموں کے لیے قدرتی گفتگو فعال کریں:

```python
from openai import OpenAI

class ConversationalVLA:
    """گفتگو انٹرفیس کے ساتھ VLA"""

    def __init__(self, vla_system):
        self.vla = vla_system
        self.llm = OpenAI()
        self.conversation_history = []
        self.scene_context = {}

    def process_utterance(self, user_input, current_image):
        """سیاق و سباق کے ساتھ قدرتی زبان پروسیس کریں"""

        # سین سیاق و سباق اپڈیٹ کریں
        self.scene_context = self._analyze_scene(current_image)

        # سیاق و سباق کے ساتھ پرامپٹ بنائیں
        system_prompt = f"""آپ ایک مددگار روبوٹ اسسٹنٹ ہیں۔

موجودہ سین: {self.scene_context}
روبوٹ کی صلاحیتیں: اٹھانا، رکھنا، ہلنا، ڈالنا، کھولنا، بند کرنا

صارف کی درخواست کی بنیاد پر:
1. روبوٹ ایکشن انجام دیں (ACTION: <command> سے جواب دیں)
2. وضاحت مانگیں (CLARIFY: <question> سے جواب دیں)
3. معلومات فراہم کریں (INFO: <response> سے جواب دیں)
"""

        self.conversation_history.append({
            "role": "user",
            "content": user_input
        })

        response = self.llm.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": system_prompt},
                *self.conversation_history
            ]
        )

        assistant_message = response.choices[0].message.content
        self.conversation_history.append({
            "role": "assistant",
            "content": assistant_message
        })

        return self._parse_response(assistant_message)

    def _parse_response(self, response):
        """LLM جواب پارس کریں اور ایکشن لیں"""
        if response.startswith("ACTION:"):
            command = response.replace("ACTION:", "").strip()
            self.vla.execute_command(command)
            return {"type": "action", "command": command}

        elif response.startswith("CLARIFY:"):
            question = response.replace("CLARIFY:", "").strip()
            self.vla._speak(question)
            return {"type": "clarify", "question": question}

        else:
            info = response.replace("INFO:", "").strip()
            self.vla._speak(info)
            return {"type": "info", "response": info}

    def _analyze_scene(self, image):
        """سیاق و سباق کے لیے موجودہ سین کا تجزیہ کریں"""
        # اشیاء کا پتہ لگانے کے لیے ویژن ماڈل استعمال کریں
        detections = self.vla.vla_model.detect_objects(image)

        return {
            "objects": [d["label"] for d in detections],
            "robot_position": self.vla.robot.get_position(),
            "gripper_state": self.vla.robot.get_gripper_state()
        }


# گفتگو کی مثال:
# صارف: "ارے روبوٹ، کیا تم صفائی میں مدد کر سکتے ہو؟"
# روبوٹ: "ضرور! مجھے ایک سیب، ایک کپ اور کچھ کاغذات نظر آ رہے ہیں۔ کہاں سے شروع کروں؟"
# صارف: "سیب کو پیالے میں رکھو"
# روبوٹ: "سیب اٹھا رہا ہوں... پیالے میں رکھ رہا ہوں... ہو گیا!"
# صارف: "اب کپ"
# روبوٹ: "مجھے دو کپ نظر آ رہے ہیں - سرخ والا یا نیلا؟"
# صارف: "سرخ والا"
# روبوٹ: "ٹھیک ہے، سرخ کپ لے رہا ہوں..."
```

### آواز کمانڈ سٹیٹ مشین

آواز کی تعاملات کے لیے مضبوط اسٹیٹ ہینڈلنگ لاگو کریں:

```python
from enum import Enum, auto

class VoiceState(Enum):
    IDLE = auto()           # ویک ورڈ کا انتظار
    LISTENING = auto()      # کمانڈ ریکارڈ کرنا
    PROCESSING = auto()     # کمانڈ سمجھنا
    CONFIRMING = auto()     # تصدیق کا انتظار
    EXECUTING = auto()      # کام چل رہا ہے
    ERROR = auto()          # غلطی سنبھالنا

class VoiceStateMachine:
    """آواز سے کنٹرول روبوٹ کے لیے سٹیٹ مشین"""

    def __init__(self, vla_system):
        self.vla = vla_system
        self.state = VoiceState.IDLE
        self.pending_command = None
        self.retry_count = 0
        self.max_retries = 3

    def transition(self, event, data=None):
        """سٹیٹ ٹرانزیشن سنبھالیں"""
        transitions = {
            VoiceState.IDLE: {
                'wake_word': (VoiceState.LISTENING, self._start_listening),
            },
            VoiceState.LISTENING: {
                'speech_end': (VoiceState.PROCESSING, self._process_speech),
                'timeout': (VoiceState.IDLE, self._timeout_message),
            },
            VoiceState.PROCESSING: {
                'understood': (VoiceState.CONFIRMING, self._confirm_command),
                'unclear': (VoiceState.LISTENING, self._ask_repeat),
                'error': (VoiceState.ERROR, self._handle_error),
            },
            VoiceState.CONFIRMING: {
                'confirmed': (VoiceState.EXECUTING, self._execute_command),
                'denied': (VoiceState.IDLE, self._cancel_command),
                'modify': (VoiceState.LISTENING, self._modify_command),
            },
            VoiceState.EXECUTING: {
                'complete': (VoiceState.IDLE, self._task_complete),
                'failed': (VoiceState.ERROR, self._handle_failure),
                'stop': (VoiceState.IDLE, self._emergency_stop),
            },
            VoiceState.ERROR: {
                'retry': (VoiceState.LISTENING, self._retry),
                'abort': (VoiceState.IDLE, self._abort),
            },
        }

        if event in transitions.get(self.state, {}):
            new_state, action = transitions[self.state][event]
            print(f"سٹیٹ: {self.state.name} -> {new_state.name}")
            self.state = new_state
            action(data)
        else:
            print(f"غلط ٹرانزیشن: {self.state.name} + {event}")

    def _start_listening(self, _):
        self.vla._speak("جی؟")

    def _process_speech(self, audio):
        text = self.vla._process_voice(audio)
        if text and len(text) > 2:
            self.pending_command = text
            self.transition('understood', text)
        else:
            self.transition('unclear')

    def _confirm_command(self, command):
        self.vla._speak(f"کیا میں {command} کروں؟")

    def _execute_command(self, _):
        self.vla._speak("ابھی شروع کرتا ہوں")
        self.vla.current_task = self.pending_command
        self.vla.task_active = True

    def _task_complete(self, _):
        self.vla._speak("ہو گیا!")
        self.pending_command = None

    def _ask_repeat(self, _):
        self.retry_count += 1
        if self.retry_count >= self.max_retries:
            self.transition('abort')
        else:
            self.vla._speak("معذرت، دوبارہ کہیں؟")

    def _emergency_stop(self, _):
        self.vla.robot.stop()
        self.vla.task_active = False
        self.vla._speak("رک گیا")

    def _handle_error(self, error):
        self.vla._speak(f"غلطی: {error}")

    def _handle_failure(self, _):
        self.vla._speak("کام ناکام۔ دوبارہ کوشش کروں؟")

    def _retry(self, _):
        self.retry_count = 0

    def _abort(self, _):
        self.vla._speak("ٹھیک ہے، جب ضرورت ہو بتائیں")
        self.pending_command = None
        self.retry_count = 0

    def _cancel_command(self, _):
        self.vla._speak("منسوخ")
        self.pending_command = None

    def _modify_command(self, _):
        self.vla._speak("کیا کروں بدلے میں؟")

    def _timeout_message(self, _):
        self.vla._speak("کچھ نہیں سنا")
```

### کثیر زبان آواز سپورٹ

اردو سمیت متعدد زبانوں کی سپورٹ:

```python
class MultiLanguageVoice:
    """VLA کے لیے کثیر زبان آواز سپورٹ"""

    SUPPORTED_LANGUAGES = {
        'en': {'name': 'English', 'tts_lang': 'en'},
        'ur': {'name': 'اردو', 'tts_lang': 'ur'},
        'hi': {'name': 'हिंदी', 'tts_lang': 'hi'},
        'ar': {'name': 'العربية', 'tts_lang': 'ar'},
    }

    def __init__(self, default_lang='ur'):
        self.whisper = whisper.load_model("medium")  # بہتر کثیر زبان
        self.current_lang = default_lang

    def transcribe(self, audio):
        """زبان کی شناخت کے ساتھ آڈیو ٹرانسکرائب کریں"""
        result = self.whisper.transcribe(
            audio,
            task="transcribe"
        )

        detected_lang = result.get("language", "ur")
        text = result["text"].strip()

        return {
            "text": text,
            "language": detected_lang,
            "confidence": result.get("confidence", 0)
        }

    def speak(self, text, lang=None):
        """مخصوص زبان میں ٹیکسٹ سے آواز"""
        lang = lang or self.current_lang
        tts_lang = self.SUPPORTED_LANGUAGES.get(lang, {}).get('tts_lang', 'ur')

        tts = gTTS(text=text, lang=tts_lang)
        tts.save("/tmp/response.mp3")
        self._play_audio("/tmp/response.mp3")

    def translate_command(self, text, source_lang, target_lang='en'):
        """پروسیسنگ کے لیے کمانڈ کو انگریزی میں ترجمہ کریں"""
        if source_lang == target_lang:
            return text

        # ترجمہ API یا ماڈل استعمال کریں
        # اس سے یقینی ہوتا ہے کہ VLA ماڈل کو انگریزی کمانڈز ملیں
        translated = self._translate(text, source_lang, target_lang)
        return translated

    def _translate(self, text, src, tgt):
        """ترجمہ کا نفاذ"""
        # OpenAI یا مخصوص ترجمہ سروس استعمال کریں
        from openai import OpenAI
        client = OpenAI()

        response = client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": f"{src} سے {tgt} میں ترجمہ کریں۔ صرف ترجمہ لکھیں۔"},
                {"role": "user", "content": text}
            ]
        )
        return response.choices[0].message.content


# اردو کے ساتھ استعمال
voice = MultiLanguageVoice(default_lang='ur')
result = voice.transcribe(audio)
# result: {"text": "سیب اٹھاؤ", "language": "ur"}

english_command = voice.translate_command(result["text"], "ur", "en")
# english_command: "pick up the apple"

# انجام دیں اور اردو میں جواب دیں
vla.execute(english_command)
voice.speak("سیب اٹھا لیا", lang='ur')  # "Picked up the apple" اردو میں
```

---

## عملی لیب

### لیب 4.5A: VLA سسٹم ڈیپلائی کریں

ایک مکمل VLA ڈیپلائمنٹ بنائیں جس میں:
1. ملٹی کیمرہ ان پٹ
2. حفاظتی فلٹرنگ
3. ناکامی کی بازیابی
4. ریئل ٹائم پرفارمنس

### لیب 4.5B: آواز سے چلنے والا VLA روبوٹ

ایک مکمل آواز سے کنٹرول VLA سسٹم بنائیں:

```python
# lab_voice_vla.py
"""
لیب 4.5B: آواز سے چلنے والا VLA روبوٹ
VLA اور Whisper استعمال کرتے ہوئے آواز سے کنٹرول روبوٹ بنائیں
"""

# مرحلہ 1: انحصارات انسٹال کریں
# pip install openai-whisper sounddevice numpy gtts pygame transformers

# مرحلہ 2: VoiceVLALab کلاس بنائیں
class VoiceVLALab:
    """آواز سے کنٹرول VLA کا لیب نفاذ"""

    def __init__(self):
        import whisper
        from gtts import gTTS

        # Whisper شروع کریں
        print("Whisper ماڈل لوڈ ہو رہا ہے...")
        self.whisper = whisper.load_model("base")

        # سمیولیٹڈ روبوٹ سٹیٹ
        self.robot_state = {
            'position': [0, 0, 0],
            'gripper': 'open',
            'current_task': None
        }

        # آبجیکٹ ڈیٹابیس (سین سمیولیٹ)
        self.scene_objects = [
            {'name': 'سیب', 'color': 'سرخ', 'position': [0.3, 0, 0]},
            {'name': 'پیالہ', 'color': 'نیلا', 'position': [0.4, 0.2, 0]},
            {'name': 'کپ', 'color': 'سفید', 'position': [0.2, -0.2, 0]},
        ]

    def listen_and_execute(self):
        """مین ڈیمو لوپ"""
        import sounddevice as sd
        import numpy as np

        print("\n" + "="*50)
        print("آواز سے چلنے والا VLA روبوٹ لیب")
        print("="*50)
        print("کمانڈز: '[چیز] اٹھاؤ', '[چیز] کو [جگہ] میں رکھو'")
        print("        'کیا نظر آتا ہے', 'رکو', 'باہر'")
        print("="*50 + "\n")

        while True:
            # آڈیو ریکارڈ کریں
            print("سن رہا ہوں... (4 سیکنڈ بولیں)")
            audio = sd.rec(int(4 * 16000), samplerate=16000,
                          channels=1, dtype=np.int16)
            sd.wait()

            # ٹرانسکرائب کریں
            audio_float = audio.flatten().astype(np.float32) / 32768.0
            result = self.whisper.transcribe(audio_float, language="ur")
            command = result["text"].strip()

            print(f"آپ نے کہا: {command}")

            if not command:
                continue

            # کمانڈ پروسیس کریں
            if 'باہر' in command.lower() or 'خارج' in command.lower():
                self.speak("خدا حافظ!")
                break

            self.process_command(command)

    def process_command(self, command):
        """آواز کمانڈ پروسیس کریں"""
        command_lower = command.lower()

        # سین کا سوال
        if 'کیا نظر' in command_lower or 'کیا دکھتا' in command_lower:
            objects = [f"{o['color']} {o['name']}" for o in self.scene_objects]
            response = f"مجھے نظر آ رہا ہے: {', '.join(objects)}"
            print(f"روبوٹ: {response}")
            self.speak(response)
            return

        # روکنے کی کمانڈ
        if 'رکو' in command_lower or 'بند' in command_lower:
            print("روبوٹ: تمام ایکشنز رک رہے ہیں")
            self.speak("رک رہا ہوں")
            self.robot_state['current_task'] = None
            return

        # اٹھانے کی کمانڈ
        if 'اٹھا' in command_lower:
            for obj in self.scene_objects:
                if obj['name'] in command_lower or obj['color'] in command_lower:
                    self.simulate_pick(obj)
                    return
            self.speak("وہ چیز نہیں ملی")
            return

        # رکھنے کی کمانڈ
        if 'رکھ' in command_lower:
            # آبجیکٹ اور منزل تلاش کریں
            held_obj = self.robot_state.get('holding')
            if held_obj:
                for obj in self.scene_objects:
                    if obj['name'] in command_lower:
                        self.simulate_place(held_obj, obj)
                        return
            self.speak("میرے ہاتھ میں کچھ نہیں")
            return

        # نامعلوم کمانڈ
        self.speak("یہ کمانڈ سمجھ نہیں آئی")

    def simulate_pick(self, obj):
        """آبجیکٹ اٹھانے کی سمیولیشن"""
        print(f"روبوٹ: {obj['name']} کی طرف جا رہا ہوں...")
        self.speak(f"{obj['color']} {obj['name']} اٹھا رہا ہوں")

        # حرکت کی سمیولیشن
        import time
        for i in range(3):
            print(f"  مرحلہ {i+1}: قریب آ رہا ہوں...")
            time.sleep(0.5)

        print(f"  گرپر {obj['name']} پر بند ہو رہا ہے")
        self.robot_state['gripper'] = 'closed'
        self.robot_state['holding'] = obj

        print(f"روبوٹ: {obj['name']} اٹھا لیا!")
        self.speak("لے لیا!")

    def simulate_place(self, obj, destination):
        """آبجیکٹ رکھنے کی سمیولیشن"""
        print(f"روبوٹ: {destination['name']} کی طرف جا رہا ہوں...")
        self.speak(f"{destination['name']} میں رکھ رہا ہوں")

        import time
        for i in range(3):
            print(f"  مرحلہ {i+1}: حرکت...")
            time.sleep(0.5)

        print(f"  گرپر کھل رہا ہے")
        self.robot_state['gripper'] = 'open'
        self.robot_state['holding'] = None

        print(f"روبوٹ: {obj['name']} {destination['name']} میں رکھ دیا!")
        self.speak("ہو گیا!")

    def speak(self, text):
        """ٹیکسٹ سے آواز"""
        try:
            from gtts import gTTS
            import pygame
            import io

            tts = gTTS(text=text, lang='ur')
            fp = io.BytesIO()
            tts.write_to_fp(fp)
            fp.seek(0)

            pygame.mixer.init()
            pygame.mixer.music.load(fp)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.wait(100)
        except Exception as e:
            print(f"[TTS غلطی: {e}]")


# لیب چلائیں
if __name__ == "__main__":
    lab = VoiceVLALab()
    lab.listen_and_execute()
```

**لیب کے مقاصد:**
1. Whisper اسپیچ ریکگنیشن کو VLA کے ساتھ یکجا کریں
2. آواز کمانڈ پارسنگ لاگو کریں
3. ٹیکسٹ سے آواز فیڈبیک شامل کریں
4. ملٹی سٹیپ آواز تعاملات سنبھالیں
5. سین کے سوالات کی سپورٹ ("کیا نظر آتا ہے؟")

**متوقع ڈیمو فلو:**
```
آواز سے چلنے والا VLA روبوٹ لیب
==================================================
کمانڈز: '[چیز] اٹھاؤ', '[چیز] کو [جگہ] میں رکھو'
        'کیا نظر آتا ہے', 'رکو', 'باہر'
==================================================

سن رہا ہوں... (4 سیکنڈ بولیں)
آپ نے کہا: کیا نظر آتا ہے؟
روبوٹ: مجھے نظر آ رہا ہے: سرخ سیب، نیلا پیالہ، سفید کپ

سن رہا ہوں... (4 سیکنڈ بولیں)
آپ نے کہا: سرخ سیب اٹھاؤ
روبوٹ: سیب کی طرف جا رہا ہوں...
  مرحلہ 1: قریب آ رہا ہوں...
  مرحلہ 2: قریب آ رہا ہوں...
  مرحلہ 3: قریب آ رہا ہوں...
  گرپر سیب پر بند ہو رہا ہے
روبوٹ: سیب اٹھا لیا!

سن رہا ہوں... (4 سیکنڈ بولیں)
آپ نے کہا: پیالے میں رکھو
روبوٹ: پیالے کی طرف جا رہا ہوں...
  مرحلہ 1: حرکت...
  مرحلہ 2: حرکت...
  مرحلہ 3: حرکت...
  گرپر کھل رہا ہے
روبوٹ: سیب پیالے میں رکھ دیا!
```

## خلاصہ

- VLA سسٹمز تمام اجزاء کو اینڈ ٹو اینڈ یکجا کرتے ہیں
- حقیقی دنیا کی ڈیپلائمنٹ کے لیے لیٹنسی اور حفاظت کی ہینڈلنگ ضروری ہے
- آپٹیمائزیشن ایج ڈیوائسز پر ڈیپلائمنٹ ممکن بناتی ہے
- **آواز کنٹرول ہینڈز فری روبوٹ آپریشن ممکن بناتا ہے**
- **ویک ورڈز اور سٹیٹ مشینز مضبوط آواز انٹرفیسز بناتی ہیں**
- **گفتگو AI قدرتی ملٹی ٹرن تعاملات ممکن بناتی ہے**
- **کثیر زبان سپورٹ رسائی بڑھاتی ہے (انگریزی، اردو، وغیرہ)**

## مزید پڑھائی

- [OpenAI Whisper](https://github.com/openai/whisper) - اسپیچ ریکگنیشن
- [Porcupine](https://picovoice.ai/platform/porcupine/) - ویک ورڈ ڈیٹیکشن
- [OpenVLA](https://openvla.github.io/) - ویژن-لینگویج-ایکشن ماڈلز
- [ROS 2 Audio](https://github.com/ros-drivers/audio_common) - آڈیو انٹیگریشن

[باب 4.6: کیپسٹون پر جائیں →](/docs/module-4-vla/chapter-6-capstone)
