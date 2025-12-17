---
sidebar_position: 3
title: "4.3 Language Models for Robotics"
description: Natural language understanding and voice control for robot instructions
keywords: [LLM, language model, instruction following, robotics, voice control, Whisper, speech-to-text]
---

# Chapter 4.3: Language Models for Robotics

## Learning Objectives

- Use LLMs for instruction parsing
- Implement task decomposition
- Ground language to robot actions
- Handle ambiguous instructions
- **Integrate voice control with Whisper speech-to-text**
- **Build real-time voice command pipelines**

## LLMs for Robot Instructions

```
┌─────────────────────────────────────────────────────────────┐
│               LANGUAGE → ROBOT PIPELINE                      │
│                                                              │
│  "Make me a sandwich"                                       │
│          │                                                   │
│          ▼                                                   │
│  ┌───────────────────────────────────────┐                  │
│  │           LLM Planner                  │                  │
│  │   1. Go to kitchen                     │                  │
│  │   2. Open fridge                       │                  │
│  │   3. Get bread, cheese, lettuce        │                  │
│  │   4. Assemble sandwich                 │                  │
│  │   5. Deliver to user                   │                  │
│  └───────────────────────────────────────┘                  │
│          │                                                   │
│          ▼                                                   │
│  ┌───────────────────────────────────────┐                  │
│  │        Action Primitives               │                  │
│  │   navigate(kitchen)                    │                  │
│  │   open(fridge)                         │                  │
│  │   pick(bread)                          │                  │
│  │   ...                                  │                  │
│  └───────────────────────────────────────┘                  │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## Task Decomposition with LLMs

```python
from openai import OpenAI

client = OpenAI()

def decompose_task(instruction):
    response = client.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": """You are a robot task planner.
            Given a high-level instruction, break it into atomic actions.
            Available actions: navigate(location), pick(object), place(object, location),
            open(container), close(container), pour(from, to)"""},
            {"role": "user", "content": instruction}
        ]
    )
    return response.choices[0].message.content

# Example
tasks = decompose_task("Pour water into the glass on the table")
# Output:
# 1. navigate(kitchen)
# 2. pick(water_bottle)
# 3. navigate(table)
# 4. pour(water_bottle, glass)
# 5. place(water_bottle, table)
```

## Instruction Parsing

```python
import re

def parse_instruction(text):
    """Extract action, object, and location from instruction"""

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

# Example
result = parse_instruction("Pick up the red apple")
# {'action': 'pick', 'params': ('red apple',)}
```

## Grounding Language to Objects

```python
class LanguageGrounder:
    def __init__(self):
        self.clip_encoder = CLIPEncoder()

    def ground_object(self, description, detections):
        """Find object matching description"""
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

# Example
grounder = LanguageGrounder()
target = grounder.ground_object("red apple", detected_objects)
```

## Handling Ambiguity

```python
def clarify_instruction(instruction, scene_objects):
    """Ask for clarification when instruction is ambiguous"""

    # Check for ambiguous references
    referenced_objects = extract_objects(instruction)

    for ref in referenced_objects:
        matches = find_matches(ref, scene_objects)

        if len(matches) > 1:
            return {
                'needs_clarification': True,
                'question': f"Which {ref}? I see {describe_options(matches)}"
            }

    return {'needs_clarification': False}
```

---

## Voice Control Integration

Voice control enables hands-free robot operation, essential for real-world applications where operators may be occupied with other tasks.

```
┌─────────────────────────────────────────────────────────────┐
│              VOICE → ROBOT CONTROL PIPELINE                  │
│                                                              │
│  ┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────┐    │
│  │  Mic    │──▶│ Whisper │──▶│   LLM   │──▶│ Robot   │    │
│  │ Input   │   │  STT    │   │ Planner │   │ Control │    │
│  └─────────┘   └─────────┘   └─────────┘   └─────────┘    │
│       │             │             │             │          │
│       ▼             ▼             ▼             ▼          │
│   Audio Stream   "Pick up    Task Plan     Actions        │
│   16kHz mono     the apple"  [pick,move]   Executed       │
│                                                            │
└─────────────────────────────────────────────────────────────┘
```

### Speech-to-Text with Whisper

OpenAI's Whisper provides state-of-the-art speech recognition for robot voice control:

```python
import whisper
import numpy as np
import sounddevice as sd

class WhisperSTT:
    """Speech-to-Text using OpenAI Whisper"""

    def __init__(self, model_size="base"):
        # Model sizes: tiny, base, small, medium, large
        self.model = whisper.load_model(model_size)
        self.sample_rate = 16000

    def transcribe_audio(self, audio_data):
        """Transcribe audio numpy array to text"""
        # Whisper expects float32 audio normalized to [-1, 1]
        audio = audio_data.astype(np.float32) / 32768.0

        result = self.model.transcribe(
            audio,
            language="en",
            task="transcribe"
        )
        return result["text"].strip()

    def record_and_transcribe(self, duration=5):
        """Record from microphone and transcribe"""
        print(f"Recording for {duration} seconds...")

        audio = sd.rec(
            int(duration * self.sample_rate),
            samplerate=self.sample_rate,
            channels=1,
            dtype=np.int16
        )
        sd.wait()

        return self.transcribe_audio(audio.flatten())

# Usage
stt = WhisperSTT(model_size="base")
command = stt.record_and_transcribe(duration=3)
print(f"You said: {command}")
# Output: "You said: pick up the red apple"
```

### Real-Time Streaming Voice Recognition

For responsive robot control, use streaming recognition:

```python
import threading
import queue
from collections import deque

class StreamingVoiceControl:
    """Real-time voice control with streaming recognition"""

    def __init__(self):
        self.stt = WhisperSTT(model_size="tiny")  # Fast model for real-time
        self.audio_queue = queue.Queue()
        self.is_listening = False
        self.buffer = deque(maxlen=int(16000 * 3))  # 3 second buffer

    def audio_callback(self, indata, frames, time, status):
        """Callback for audio stream"""
        if self.is_listening:
            self.audio_queue.put(indata.copy())

    def start_listening(self):
        """Start the audio stream"""
        self.is_listening = True
        self.stream = sd.InputStream(
            samplerate=16000,
            channels=1,
            dtype=np.int16,
            callback=self.audio_callback,
            blocksize=1024
        )
        self.stream.start()

        # Start processing thread
        self.process_thread = threading.Thread(target=self._process_audio)
        self.process_thread.start()

    def _process_audio(self):
        """Process audio chunks and detect commands"""
        while self.is_listening:
            try:
                chunk = self.audio_queue.get(timeout=0.1)
                self.buffer.extend(chunk.flatten())

                # Check for voice activity
                if self._detect_voice_activity(chunk):
                    # Transcribe buffer
                    audio = np.array(self.buffer)
                    text = self.stt.transcribe_audio(audio)

                    if text:
                        self._handle_command(text)
                        self.buffer.clear()

            except queue.Empty:
                continue

    def _detect_voice_activity(self, audio, threshold=500):
        """Simple voice activity detection"""
        return np.abs(audio).mean() > threshold

    def _handle_command(self, text):
        """Process recognized command"""
        print(f"Command received: {text}")
        # Send to robot controller

    def stop_listening(self):
        """Stop the audio stream"""
        self.is_listening = False
        self.stream.stop()
```

### Wake Word Detection

Use wake words to activate the robot only when addressed:

```python
import pvporcupine
import struct

class WakeWordDetector:
    """Wake word detection using Porcupine"""

    def __init__(self, wake_words=["jarvis", "robot"]):
        self.porcupine = pvporcupine.create(
            keywords=wake_words
        )
        self.sample_rate = self.porcupine.sample_rate
        self.frame_length = self.porcupine.frame_length

    def process_audio(self, audio_frame):
        """Check if wake word is detected"""
        pcm = struct.unpack_from(
            "h" * self.frame_length,
            audio_frame
        )

        keyword_index = self.porcupine.process(pcm)

        if keyword_index >= 0:
            return True  # Wake word detected
        return False

    def cleanup(self):
        self.porcupine.delete()


class VoiceControlledRobot:
    """Complete voice-controlled robot system"""

    def __init__(self, robot_interface):
        self.robot = robot_interface
        self.wake_detector = WakeWordDetector(["hey robot"])
        self.stt = WhisperSTT(model_size="small")
        self.is_awake = False
        self.awake_timeout = 10  # seconds

    def run(self):
        """Main loop for voice control"""
        print("Say 'Hey Robot' to activate...")

        with sd.InputStream(
            samplerate=16000,
            channels=1,
            dtype=np.int16,
            blocksize=512
        ) as stream:
            while True:
                audio, _ = stream.read(512)

                if not self.is_awake:
                    # Listen for wake word
                    if self.wake_detector.process_audio(audio.tobytes()):
                        print("Robot activated! Listening for command...")
                        self.is_awake = True
                        self._play_activation_sound()
                else:
                    # Record command
                    command = self.stt.record_and_transcribe(duration=5)
                    print(f"Command: {command}")

                    # Execute command
                    self._execute_voice_command(command)
                    self.is_awake = False

    def _execute_voice_command(self, command):
        """Parse and execute voice command"""
        # Use LLM to decompose task
        tasks = decompose_task(command)

        for task in tasks:
            action = parse_instruction(task)
            if action:
                self.robot.execute(action)

    def _play_activation_sound(self):
        """Play sound to indicate robot is listening"""
        # Play a short beep
        pass
```

### Voice Feedback and Confirmation

Enable bidirectional voice communication:

```python
from gtts import gTTS
import pygame
import io

class VoiceFeedback:
    """Text-to-speech for robot responses"""

    def __init__(self):
        pygame.mixer.init()

    def speak(self, text):
        """Convert text to speech and play"""
        tts = gTTS(text=text, lang='en')

        # Save to memory buffer
        fp = io.BytesIO()
        tts.write_to_fp(fp)
        fp.seek(0)

        # Play audio
        pygame.mixer.music.load(fp)
        pygame.mixer.music.play()

        while pygame.mixer.music.get_busy():
            pygame.time.wait(100)

    def confirm_action(self, action):
        """Speak confirmation of action"""
        confirmations = {
            'pick': f"Picking up the {action['params'][0]}",
            'place': f"Placing {action['params'][0]} on {action['params'][1]}",
            'move': f"Moving to {action['params'][0]}"
        }

        message = confirmations.get(
            action['action'],
            f"Executing {action['action']}"
        )
        self.speak(message)


class InteractiveVoiceRobot:
    """Robot with voice input and output"""

    def __init__(self, robot):
        self.robot = robot
        self.stt = WhisperSTT()
        self.tts = VoiceFeedback()
        self.grounder = LanguageGrounder()

    def process_command(self, audio):
        """Full voice command pipeline with feedback"""
        # 1. Transcribe
        command = self.stt.transcribe_audio(audio)
        self.tts.speak(f"I heard: {command}")

        # 2. Parse instruction
        action = parse_instruction(command)

        if action is None:
            self.tts.speak("Sorry, I didn't understand that command")
            return

        # 3. Check for ambiguity
        scene_objects = self.robot.get_detected_objects()
        clarity = clarify_instruction(command, scene_objects)

        if clarity['needs_clarification']:
            self.tts.speak(clarity['question'])
            # Wait for clarification
            clarification = self.stt.record_and_transcribe(duration=5)
            command = f"{command}, specifically the {clarification}"
            action = parse_instruction(command)

        # 4. Confirm and execute
        self.tts.confirm_action(action)
        self.robot.execute(action)
        self.tts.speak("Done!")
```

### ROS 2 Voice Control Node

Integrate voice control with ROS 2:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np

class VoiceControlNode(Node):
    """ROS 2 node for voice-controlled robot"""

    def __init__(self):
        super().__init__('voice_control_node')

        # Publishers
        self.cmd_pub = self.create_publisher(String, '/robot_command', 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Voice control components
        self.stt = WhisperSTT(model_size="small")
        self.wake_detector = WakeWordDetector(["hey robot"])

        # Parameters
        self.declare_parameter('wake_word', 'hey robot')
        self.declare_parameter('listen_duration', 5.0)

        # Timer for audio processing
        self.create_timer(0.1, self.process_audio)

        self.get_logger().info('Voice control node started')

    def process_audio(self):
        """Process incoming audio for commands"""
        # Record short audio segment
        audio = self.record_audio(duration=0.5)

        # Check for wake word
        if self.wake_detector.process_audio(audio.tobytes()):
            self.get_logger().info('Wake word detected!')

            # Record full command
            command_audio = self.record_audio(duration=5.0)
            command = self.stt.transcribe_audio(command_audio)

            self.get_logger().info(f'Command: {command}')
            self.process_voice_command(command)

    def process_voice_command(self, command):
        """Convert voice command to robot action"""
        command_lower = command.lower()

        # Direct movement commands
        if 'forward' in command_lower:
            self.send_velocity(linear=0.5)
        elif 'backward' in command_lower or 'back' in command_lower:
            self.send_velocity(linear=-0.5)
        elif 'left' in command_lower:
            self.send_velocity(angular=0.5)
        elif 'right' in command_lower:
            self.send_velocity(angular=-0.5)
        elif 'stop' in command_lower:
            self.send_velocity(linear=0.0, angular=0.0)
        else:
            # Send complex command for LLM processing
            msg = String()
            msg.data = command
            self.cmd_pub.publish(msg)

    def send_velocity(self, linear=0.0, angular=0.0):
        """Publish velocity command"""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.vel_pub.publish(twist)

    def record_audio(self, duration):
        """Record audio from microphone"""
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

## Hands-on Lab

### Lab 4.3A: Build Instruction Parser

Create a system that:
1. Parses natural language instructions
2. Decomposes complex tasks
3. Grounds object references
4. Asks for clarification when needed

### Lab 4.3B: Voice-Controlled Robot

Build a complete voice control system:

```python
# lab_voice_control.py
"""
Lab 4.3B: Voice-Controlled Robot
Build a voice-controlled robot using Whisper and ROS 2
"""

# Step 1: Install dependencies
# pip install openai-whisper sounddevice numpy pvporcupine gtts pygame

# Step 2: Implement the VoiceRobotLab class
class VoiceRobotLab:
    def __init__(self):
        # Initialize components
        self.stt = WhisperSTT(model_size="base")
        self.tts = VoiceFeedback()

        # Define command vocabulary
        self.commands = {
            'pick': self.handle_pick,
            'place': self.handle_place,
            'move': self.handle_move,
            'stop': self.handle_stop,
        }

    def run_demo(self):
        """Run interactive voice control demo"""
        self.tts.speak("Voice control ready. Say a command.")

        while True:
            # Record command
            print("\nListening...")
            command = self.stt.record_and_transcribe(duration=5)
            print(f"You said: {command}")

            if 'exit' in command.lower() or 'quit' in command.lower():
                self.tts.speak("Goodbye!")
                break

            # Process command
            self.process_command(command)

    def process_command(self, command):
        """Parse and execute voice command"""
        action = parse_instruction(command)

        if action and action['action'] in self.commands:
            self.tts.speak(f"Executing {action['action']}")
            self.commands[action['action']](action['params'])
        else:
            self.tts.speak("Command not recognized. Try again.")

    def handle_pick(self, params):
        print(f"Picking up: {params[0]}")

    def handle_place(self, params):
        print(f"Placing {params[0]} on {params[1]}")

    def handle_move(self, params):
        print(f"Moving to: {params[0]}")

    def handle_stop(self, params):
        print("Stopping robot")


# Step 3: Run the lab
if __name__ == "__main__":
    lab = VoiceRobotLab()
    lab.run_demo()
```

**Lab Objectives:**
1. Set up Whisper speech recognition
2. Implement wake word detection
3. Create voice command parser
4. Add text-to-speech feedback
5. Test with 10+ different voice commands

**Expected Output:**
```
Voice control ready. Say a command.

Listening...
You said: pick up the red apple
Executing pick
Picking up: red apple

Listening...
You said: put it in the bowl
Executing place
Placing it on bowl
```

## Summary

- LLMs decompose complex instructions into atomic actions
- Parsing extracts action types and parameters
- CLIP enables grounding language to visual objects
- Ambiguity handling improves robustness
- **Whisper provides accurate speech-to-text for voice control**
- **Wake word detection enables hands-free activation**
- **Voice feedback creates interactive robot experiences**
- **ROS 2 integration enables voice control in robot systems**

## Further Reading

- [OpenAI Whisper Documentation](https://github.com/openai/whisper)
- [Porcupine Wake Word Engine](https://picovoice.ai/platform/porcupine/)
- [ROS 2 Audio Common](https://github.com/ros-drivers/audio_common)
- [SpeechBrain: Speech AI Toolkit](https://speechbrain.github.io/)

[Continue to Chapter 4.4 →](/ur/docs/module-4-vla/chapter-4-action)
