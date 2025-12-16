---
sidebar_position: 6
title: "4.6 Capstone: VLA Robot Project"
description: Build a complete voice-controlled instruction-following robot
keywords: [capstone, project, VLA, robot, instruction-following, voice control, Whisper]
---

# Chapter 4.6: Capstone - VLA Robot Project

## Project Overview

In this capstone, you'll build a complete VLA-powered robot that can:

- Understand natural language instructions
- Perceive objects in the environment
- Plan and execute manipulation tasks
- Learn from demonstrations
- **Respond to voice commands with wake word activation**
- **Provide voice feedback and confirmations**

## Project: Voice-Controlled Kitchen Assistant Robot

### System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│           VOICE-CONTROLLED KITCHEN ASSISTANT VLA SYSTEM           │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────┐    │
│  │                    USER INTERFACE                          │    │
│  │                                                            │    │
│  │   🎤 Voice Input ─────────────┐                           │    │
│  │   "Hey Chef, put the apple    │                           │    │
│  │    in the bowl"               │                           │    │
│  │                               │                           │    │
│  │   🔊 Voice Output ◄───────────┼────────────────────┐     │    │
│  │   "Picking up the apple..."   │                    │     │    │
│  │                               │                    │     │    │
│  └───────────────────────────────┼────────────────────┼─────┘    │
│                                  │                    │           │
│  ┌───────────────────────────────┴────────────────────┴─────┐    │
│  │                    VOICE PROCESSING                        │    │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────────────┐  │    │
│  │  │ Wake Word  │  │  Whisper   │  │   Text-to-Speech   │  │    │
│  │  │ Detection  │──│    STT     │  │      (gTTS)        │  │    │
│  │  │(Porcupine) │  │            │  │                    │  │    │
│  │  └────────────┘  └──────┬─────┘  └────────────────────┘  │    │
│  └─────────────────────────┼────────────────────────────────┘    │
│                            │                                      │
│  ┌─────────────────────────┴────────────────────────────────┐    │
│  │                    VLA BRAIN                               │    │
│  │                                                            │    │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────┐          │    │
│  │  │   Vision   │  │  Language  │  │   Action   │          │    │
│  │  │  Encoder   │  │  Encoder   │  │  Decoder   │          │    │
│  │  │   (ViT)    │  │   (T5)     │  │   (MLP)    │          │    │
│  │  └──────┬─────┘  └──────┬─────┘  └──────┬─────┘          │    │
│  │         └───────────────┼───────────────┘                 │    │
│  │                         │                                  │    │
│  │                  Fusion + Action                          │    │
│  └─────────────────────────┬────────────────────────────────┘    │
│                            │                                      │
│  ┌─────────────────────────┴────────────────────────────────┐    │
│  │                    ROBOT HARDWARE                          │    │
│  │  ┌────────┐  ┌────────┐  ┌────────┐  ┌────────────────┐  │    │
│  │  │ Camera │  │  Arm   │  │Gripper │  │  Microphone    │  │    │
│  │  │  x2    │  │ 7-DOF  │  │ 2-jaw  │  │  + Speaker     │  │    │
│  │  └────────┘  └────────┘  └────────┘  └────────────────┘  │    │
│  └──────────────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────────────┘
```

## Phase 1: Environment Setup

### Isaac Sim Scene

```python
# capstone/setup_kitchen.py
from omni.isaac.core import World
from omni.isaac.manipulators import SingleManipulator

def setup_kitchen():
    world = World()

    # Add robot
    robot = world.scene.add(
        SingleManipulator(
            prim_path="/World/FrankaKitchen",
            usd_path="/Isaac/Robots/Franka/franka.usd"
        )
    )

    # Add kitchen items
    items = [
        ("apple", "/assets/apple.usd", [0.3, 0.0, 0.02]),
        ("bowl", "/assets/bowl.usd", [0.4, 0.2, 0.0]),
        ("cup", "/assets/cup.usd", [0.2, -0.2, 0.0]),
        ("plate", "/assets/plate.usd", [0.5, 0.0, 0.0]),
    ]

    for name, usd, pos in items:
        world.scene.add_usd_to_stage(usd, f"/World/{name}")

    return world, robot
```

## Phase 2: VLA Model

### Model Architecture

```python
# capstone/vla_model.py
import torch
import torch.nn as nn
from transformers import ViTModel, T5EncoderModel

class KitchenVLA(nn.Module):
    def __init__(self):
        super().__init__()

        # Vision encoder
        self.vision = ViTModel.from_pretrained('google/vit-base-patch16-224')

        # Language encoder
        self.language = T5EncoderModel.from_pretrained('t5-base')

        # Fusion layer
        self.fusion = nn.MultiheadAttention(embed_dim=768, num_heads=8)

        # Action decoder
        self.action_head = nn.Sequential(
            nn.Linear(768, 256),
            nn.ReLU(),
            nn.Linear(256, 7),  # 7-DOF action
            nn.Tanh()
        )

    def forward(self, image, instruction_tokens):
        # Encode vision
        vis_features = self.vision(image).last_hidden_state

        # Encode language
        lang_features = self.language(instruction_tokens).last_hidden_state

        # Cross-attention fusion
        fused, _ = self.fusion(
            query=vis_features,
            key=lang_features,
            value=lang_features
        )

        # Decode action
        pooled = fused.mean(dim=1)
        action = self.action_head(pooled)

        return action
```

## Phase 3: Training

### Data Collection

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

        print(f"Recording demo for: {instruction}")
        print("Move the robot to complete the task...")

        while not self.task_complete():
            obs = self.get_observation()
            action = self.get_human_action()

            demo['observations'].append(obs)
            demo['actions'].append(action)

        self.demos.append(demo)
        return demo
```

### Training Loop

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

            # Forward pass
            pred_actions = model(images, instructions)

            # Loss
            loss = criterion(pred_actions, target_actions)

            # Backward
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

        print(f"Epoch {epoch}: Loss = {loss.item():.4f}")
```

## Phase 4: Voice Control Integration

### Voice Control System

```python
# capstone/voice_control.py
"""Voice control system for Kitchen Assistant"""

import whisper
import numpy as np
import sounddevice as sd
from gtts import gTTS
import pygame
import io
import threading
import queue

class VoiceController:
    """Voice control for kitchen robot"""

    def __init__(self, wake_word="hey chef"):
        # Speech recognition
        print("Loading Whisper model...")
        self.whisper = whisper.load_model("small")
        self.wake_word = wake_word.lower()

        # Audio settings
        self.sample_rate = 16000
        self.audio_queue = queue.Queue()
        self.is_listening = True

        # State
        self.is_awake = False
        self.awake_timeout = 10.0  # seconds

        # Initialize pygame for audio playback
        pygame.mixer.init()

    def start(self):
        """Start voice control listener"""
        self.listen_thread = threading.Thread(target=self._listen_loop)
        self.listen_thread.daemon = True
        self.listen_thread.start()
        print(f"Voice control active. Say '{self.wake_word}' to start.")

    def _listen_loop(self):
        """Continuous listening loop"""
        buffer = []
        silence_frames = 0

        with sd.InputStream(samplerate=self.sample_rate,
                           channels=1, dtype=np.int16,
                           blocksize=1024) as stream:
            while self.is_listening:
                audio_chunk, _ = stream.read(1024)
                audio_chunk = audio_chunk.flatten()

                # Voice activity detection
                energy = np.abs(audio_chunk).mean()

                if energy > 300:  # Voice detected
                    buffer.extend(audio_chunk)
                    silence_frames = 0
                else:
                    silence_frames += 1

                # Process after silence
                if silence_frames > 10 and len(buffer) > self.sample_rate:
                    audio = np.array(buffer, dtype=np.float32) / 32768.0
                    self.audio_queue.put(audio)
                    buffer = []

    def get_command(self, timeout=5.0):
        """Get voice command (blocking)"""
        try:
            audio = self.audio_queue.get(timeout=timeout)
            return self._transcribe(audio)
        except queue.Empty:
            return None

    def _transcribe(self, audio):
        """Transcribe audio to text"""
        result = self.whisper.transcribe(audio, language="en")
        text = result["text"].strip().lower()

        # Check for wake word
        if not self.is_awake:
            if self.wake_word in text:
                self.is_awake = True
                # Extract command after wake word
                parts = text.split(self.wake_word)
                if len(parts) > 1:
                    return parts[-1].strip()
                return ""  # Wake word only, wait for command
            return None
        else:
            return text

    def speak(self, text):
        """Text-to-speech output"""
        print(f"Robot: {text}")
        tts = gTTS(text=text, lang='en')
        fp = io.BytesIO()
        tts.write_to_fp(fp)
        fp.seek(0)

        pygame.mixer.music.load(fp)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pygame.time.wait(100)

    def confirm_action(self, action_description):
        """Speak action confirmation"""
        self.speak(action_description)

    def ask_clarification(self, question):
        """Ask user for clarification and get response"""
        self.speak(question)
        return self.get_command(timeout=10.0)

    def stop(self):
        """Stop voice control"""
        self.is_listening = False


class VoiceCommandParser:
    """Parse voice commands into robot actions"""

    COMMANDS = {
        'pick': ['pick up', 'grab', 'get', 'take'],
        'place': ['put', 'place', 'set', 'drop'],
        'move': ['move', 'go', 'navigate'],
        'pour': ['pour', 'fill'],
        'stop': ['stop', 'halt', 'pause', 'wait'],
        'status': ['what', 'where', 'show', 'tell me'],
    }

    OBJECTS = ['apple', 'bowl', 'cup', 'plate', 'bottle', 'glass', 'fork', 'knife', 'spoon']
    LOCATIONS = ['table', 'counter', 'sink', 'fridge', 'cabinet', 'left', 'right', 'here']

    def parse(self, command):
        """Parse voice command into structured action"""
        if not command:
            return None

        command = command.lower()

        # Detect action type
        action_type = None
        for action, keywords in self.COMMANDS.items():
            if any(kw in command for kw in keywords):
                action_type = action
                break

        if not action_type:
            return {'type': 'unknown', 'raw': command}

        # Extract objects and locations
        objects = [obj for obj in self.OBJECTS if obj in command]
        locations = [loc for loc in self.LOCATIONS if loc in command]

        return {
            'type': action_type,
            'objects': objects,
            'locations': locations,
            'raw': command
        }
```

### Voice-Enabled ROS 2 Node

```python
# capstone/voice_vla_node.py
"""Voice-controlled VLA ROS 2 Node"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from cv_bridge import CvBridge
import torch

class VoiceVLANode(Node):
    """ROS 2 node with voice control for kitchen robot"""

    def __init__(self):
        super().__init__('voice_vla_node')

        # Voice control
        self.voice = VoiceController(wake_word="hey chef")
        self.parser = VoiceCommandParser()

        # VLA Model
        self.model = KitchenVLA()
        self.model.load_state_dict(torch.load('kitchen_vla.pt'))
        self.model.eval()

        # ROS setup
        self.bridge = CvBridge()
        self.current_image = None
        self.task_active = False

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # Publishers
        self.action_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory', 10
        )
        self.status_pub = self.create_publisher(
            String, '/robot_status', 10
        )

        # Start voice control
        self.voice.start()
        self.voice.speak("Kitchen assistant ready. Say 'Hey Chef' to give me a command.")

        # Voice processing timer
        self.create_timer(0.1, self.process_voice)

        self.get_logger().info('Voice VLA Node initialized')

    def image_callback(self, msg):
        """Store current camera image"""
        self.current_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def process_voice(self):
        """Process incoming voice commands"""
        command_text = self.voice.get_command(timeout=0.05)

        if command_text is None:
            return

        if command_text == "":
            # Wake word detected, waiting for command
            self.voice.speak("Yes? What would you like me to do?")
            return

        self.get_logger().info(f'Voice command: {command_text}')

        # Parse command
        parsed = self.parser.parse(command_text)
        self.handle_command(parsed)

    def handle_command(self, parsed):
        """Handle parsed voice command"""
        if parsed['type'] == 'stop':
            self.task_active = False
            self.voice.speak("Stopping")
            return

        if parsed['type'] == 'status':
            self.report_status()
            return

        if parsed['type'] == 'unknown':
            self.voice.speak("Sorry, I didn't understand that. Try saying 'pick up the apple' or 'put it in the bowl'.")
            return

        # Handle pick/place/move commands
        if parsed['type'] == 'pick':
            if not parsed['objects']:
                self.voice.speak("What should I pick up?")
                return
            obj = parsed['objects'][0]
            self.voice.confirm_action(f"Picking up the {obj}")
            self.execute_vla_task(f"pick up the {obj}")

        elif parsed['type'] == 'place':
            if not parsed['locations']:
                self.voice.speak("Where should I put it?")
                return
            loc = parsed['locations'][0]
            self.voice.confirm_action(f"Placing in the {loc}")
            self.execute_vla_task(f"place in the {loc}")

        elif parsed['type'] == 'pour':
            self.voice.confirm_action("Pouring")
            self.execute_vla_task("pour into container")

    def execute_vla_task(self, instruction):
        """Execute VLA task with voice feedback"""
        self.task_active = True

        while self.task_active:
            if self.current_image is None:
                continue

            # Get action from VLA
            action = self.model(
                self.preprocess_image(self.current_image),
                instruction
            )

            # Publish action
            traj_msg = self.action_to_trajectory(action)
            self.action_pub.publish(traj_msg)

            # Check completion
            if self.detect_task_complete():
                self.task_active = False
                self.voice.speak("Done!")
                break

            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

    def report_status(self):
        """Report robot status via voice"""
        status = "I'm ready for commands."
        if self.task_active:
            status = "I'm currently working on a task."
        self.voice.speak(status)

    def preprocess_image(self, image):
        """Preprocess image for model"""
        # Resize, normalize, convert to tensor
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
        """Convert model output to ROS trajectory message"""
        msg = JointTrajectory()
        # ... conversion logic
        return msg

    def detect_task_complete(self):
        """Detect if current task is complete"""
        # Implementation depends on task type
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

### Multi-Language Support (English/Urdu)

```python
# capstone/multilingual_voice.py
"""Multi-language voice control with Urdu support"""

class MultilingualVoiceController(VoiceController):
    """Voice controller supporting English and Urdu"""

    WAKE_WORDS = {
        'en': 'hey chef',
        'ur': 'ارے شیف',  # "arey chef" in Urdu
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
            'ready': "باورچی خانے کا معاون تیار ہے۔",
            'listening': "جی ہاں؟ آپ کیا کرنا چاہیں گے؟",
            'picking': "{} اٹھا رہا ہوں",
            'placing': "{} میں رکھ رہا ہوں",
            'done': "ہو گیا!",
            'unknown': "معذرت، میں سمجھ نہیں سکا۔",
        }
    }

    def __init__(self, default_lang='en'):
        super().__init__()
        self.current_lang = default_lang
        self.whisper = whisper.load_model("medium")  # Better multilingual

    def _transcribe(self, audio):
        """Transcribe with language detection"""
        result = self.whisper.transcribe(audio)
        detected_lang = result.get("language", "en")

        # Map to supported languages
        if detected_lang in ['ur', 'hi']:  # Urdu/Hindi
            self.current_lang = 'ur'
        else:
            self.current_lang = 'en'

        text = result["text"].strip().lower()

        # Check for wake word in detected language
        wake_word = self.WAKE_WORDS.get(self.current_lang, 'hey chef')
        if wake_word in text:
            self.is_awake = True
            parts = text.split(wake_word)
            return parts[-1].strip() if len(parts) > 1 else ""

        return text if self.is_awake else None

    def speak(self, key, *args):
        """Speak in current language"""
        responses = self.RESPONSES.get(self.current_lang, self.RESPONSES['en'])
        text = responses.get(key, key)
        if args:
            text = text.format(*args)

        print(f"Robot ({self.current_lang}): {text}")
        tts = gTTS(text=text, lang=self.current_lang)
        # ... play audio
```

---

## Phase 5: ROS 2 Deployment

### Standard ROS 2 Node (Text Input)

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

        # Load model
        self.model = KitchenVLA()
        self.model.load_state_dict(torch.load('kitchen_vla.pt'))
        self.model.eval()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.instruction_sub = self.create_subscription(
            String, '/instruction', self.instruction_callback, 10
        )

        # Publisher
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
            # Get action from model
            action = self.model(self.current_image, self.current_instruction)

            # Publish action
            traj_msg = self.action_to_trajectory(action)
            self.action_pub.publish(traj_msg)

            time.sleep(0.1)
```

## Phase 6: Testing

### Test Scenarios

```yaml
# capstone/test_scenarios.yaml
scenarios:
  # Text-based scenarios
  - name: "Simple Pick and Place"
    instruction: "Put the apple in the bowl"
    expected_outcome: apple_in_bowl

  - name: "Spatial Reasoning"
    instruction: "Put the cup next to the plate"
    expected_outcome: cup_near_plate

  - name: "Multi-step Task"
    instruction: "Stack the cups"
    expected_outcome: cups_stacked

  - name: "Ambiguous Reference"
    instruction: "Pick up the fruit"
    expected_outcome: clarification_requested

  # Voice-based scenarios
  - name: "Voice Wake Word"
    voice_input: "Hey Chef"
    expected_outcome: robot_listening

  - name: "Voice Pick Command"
    voice_input: "Hey Chef, pick up the red apple"
    expected_outcome: apple_picked
    voice_confirmation: "Picking up the red apple"

  - name: "Voice Place Command"
    voice_input: "Put it in the bowl"
    expected_outcome: apple_in_bowl
    voice_confirmation: "Placing in the bowl"

  - name: "Voice Stop Command"
    voice_input: "Stop"
    expected_outcome: robot_stopped
    voice_confirmation: "Stopping"

  - name: "Voice Status Query"
    voice_input: "What do you see?"
    expected_outcome: scene_description
    voice_confirmation: "I can see..."

  # Multi-language scenarios (Urdu)
  - name: "Urdu Wake Word"
    voice_input: "ارے شیف"
    expected_outcome: robot_listening_urdu

  - name: "Urdu Pick Command"
    voice_input: "سیب اٹھاؤ"
    expected_outcome: apple_picked
    voice_confirmation: "سیب اٹھا رہا ہوں"
```

### Evaluation

```python
def evaluate_system(vla_node, scenarios):
    results = []

    for scenario in scenarios:
        # Reset environment
        reset_scene()

        # Execute instruction (voice or text)
        if 'voice_input' in scenario:
            success = vla_node.process_voice_test(scenario['voice_input'])
            # Verify voice confirmation
            if 'voice_confirmation' in scenario:
                confirmed = verify_voice_output(scenario['voice_confirmation'])
                success = success and confirmed
        else:
            success = vla_node.execute(scenario['instruction'])

        # Check outcome
        outcome = check_outcome(scenario['expected_outcome'])

        results.append({
            'scenario': scenario['name'],
            'success': success and outcome,
        })

    # Report
    success_rate = sum(r['success'] for r in results) / len(results)
    print(f"Success Rate: {success_rate:.1%}")

    # Breakdown by category
    voice_tests = [r for r in results if 'voice' in r['scenario'].lower()]
    text_tests = [r for r in results if 'voice' not in r['scenario'].lower()]

    print(f"Voice Commands: {sum(r['success'] for r in voice_tests)}/{len(voice_tests)}")
    print(f"Text Commands: {sum(r['success'] for r in text_tests)}/{len(text_tests)}")
```

## Deliverables Checklist

### Core Requirements
- [ ] Isaac Sim kitchen environment with objects
- [ ] Trained VLA model (minimum 80% accuracy)
- [ ] ROS 2 deployment node

### Voice Control Requirements
- [ ] Whisper-based speech recognition
- [ ] Wake word activation ("Hey Chef")
- [ ] Voice command parsing
- [ ] Text-to-speech feedback
- [ ] Voice control ROS 2 node

### Demo Requirements
- [ ] Demo video: 5+ text-based tasks
- [ ] Demo video: 5+ voice-controlled tasks
- [ ] Multi-language demo (English + Urdu)
- [ ] Error handling and clarification demo

### Documentation
- [ ] Technical report
- [ ] Voice command reference guide
- [ ] Installation and setup instructions

## Example Demo Script

```
Demo: Voice-Controlled Kitchen Assistant Robot
================================================

[Robot starts up]
Robot: "Kitchen assistant ready. Say 'Hey Chef' to give me a command."

[User speaks]
User: "Hey Chef"
Robot: "Yes? What would you like me to do?"

[User gives command]
User: "Pick up the red apple"
Robot: "Picking up the red apple"
[Robot moves to apple, picks it up]
Robot: "Got it!"

[User gives another command]
User: "Put it in the bowl"
Robot: "Placing in the bowl"
[Robot moves to bowl, places apple]
Robot: "Done!"

[User asks status]
User: "What do you see?"
Robot: "I can see a bowl with an apple, a cup, and a plate."

[User gives Urdu command]
User: "ارے شیف، پیالہ اٹھاؤ" (Hey Chef, pick up the bowl)
Robot: "پیالہ اٹھا رہا ہوں" (Picking up the bowl)
[Robot picks up bowl]
Robot: "ہو گیا!" (Done!)

[User stops robot]
User: "Stop"
Robot: "Stopping"
[Robot halts all movement]

[Demo complete]
Robot: "Thank you for using the Kitchen Assistant!"
```

## Congratulations!

You've completed the Physical AI & Humanoid Robotics course! You now have the skills to:

- Build ROS 2 robot systems
- Create digital twins in simulation
- Deploy AI-powered perception
- Implement Vision-Language-Action robots
- **Build voice-controlled robots with Whisper**
- **Create multilingual robot interfaces**

### What's Next?

- Contribute to open-source robotics projects
- Join the ROS community
- Explore research papers on embodied AI
- Build your own voice-controlled robot!
- Experiment with other languages and dialects

### Resources

- [OpenAI Whisper](https://github.com/openai/whisper) - Speech recognition
- [OpenVLA](https://openvla.github.io/) - Vision-Language-Action models
- [ROS 2 Documentation](https://docs.ros.org/en/humble/) - Robot Operating System
- [NVIDIA Isaac](https://developer.nvidia.com/isaac-sim) - Robot simulation

---

**Thank you for learning with us!** 🤖🎓

*Now go build amazing voice-controlled robots!*
