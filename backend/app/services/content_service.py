from typing import Dict, Any, List, Optional
from openai import AsyncOpenAI

from app.config import settings


class ContentService:
    """Service for content transformation operations."""

    def __init__(self):
        self.openai = AsyncOpenAI(api_key=settings.openai_api_key)

    async def personalize_content(
        self,
        content: str,
        profile: Dict[str, Any],
    ) -> Dict[str, Any]:
        """Personalize content based on user profile."""
        system_prompt = f"""You are an expert educational content adapter.

Adapt the following robotics/AI textbook content based on the learner's background:

User Profile:
- Software Experience: {profile.get('software_experience', 'beginner')}
- Programming Languages: {', '.join(profile.get('programming_languages', ['Python']))}
- ROS Experience: {'Yes' if profile.get('ros_experience') else 'No'}
- Linux Experience: {profile.get('linux_experience', 'basic')}
- Hardware Experience: {profile.get('hardware_experience', 'none')}
- Has GPU Workstation: {'Yes' if profile.get('has_gpu_workstation') else 'No'}
- Has Jetson Kit: {'Yes' if profile.get('has_jetson_kit') else 'No'}

Adaptation Guidelines:
1. For beginners: Add more explanations, simpler examples, more context
2. For advanced users: Skip basics, add advanced tips, more efficient approaches
3. Adjust code examples to match known programming languages
4. Add relevant warnings/tips based on equipment availability
5. Reference prior knowledge when applicable

Output the adapted content maintaining the original markdown structure.
At the end, list the adaptations you made in a JSON array format like:
ADAPTATIONS: ["adaptation 1", "adaptation 2"]
"""

        response = await self.openai.chat.completions.create(
            model=settings.openai_model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": content},
            ],
            temperature=0.7,
            max_tokens=4000,
        )

        result = response.choices[0].message.content

        # Parse adaptations from response
        adaptations = []
        if "ADAPTATIONS:" in result:
            parts = result.split("ADAPTATIONS:")
            result = parts[0].strip()
            try:
                import json
                adaptations = json.loads(parts[1].strip())
            except:
                adaptations = ["Content personalized based on your profile"]

        return {
            "content": result,
            "adaptations": adaptations,
        }

    async def translate_content(
        self,
        content: str,
        target_language: str = "ur",
    ) -> Dict[str, Any]:
        """Translate content to Urdu while preserving technical terms."""
        preserve_terms = [
            "ROS 2", "ROS", "Gazebo", "Unity", "NVIDIA", "Isaac", "Isaac Sim",
            "VLA", "DDS", "URDF", "SDF", "Node", "Topic", "Service", "Action",
            "Publisher", "Subscriber", "Message", "Package", "Workspace",
            "Python", "C++", "Linux", "Ubuntu", "Docker", "Git", "GitHub",
            "API", "SDK", "GPU", "CPU", "CUDA", "TensorRT", "PyTorch",
            "RGB", "RGBD", "LiDAR", "IMU", "SLAM", "MoveIt", "Nav2"
        ]

        system_prompt = f"""You are an expert Urdu translator specializing in technical content.

Translate the following robotics/AI educational content from English to Urdu.

Translation Guidelines:
1. Maintain technical accuracy
2. Preserve these terms in English (do not translate):
   {', '.join(preserve_terms)}
3. Keep code blocks entirely in English
4. Preserve markdown formatting
5. Preserve URLs and links
6. Use proper Urdu technical vocabulary where established
7. Make the translation natural and fluent for Pakistani engineering students

IMPORTANT: Output ONLY the translated content with preserved markdown formatting.
"""

        response = await self.openai.chat.completions.create(
            model=settings.openai_model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": content},
            ],
            temperature=0.3,
            max_tokens=4000,
        )

        return {
            "content": response.choices[0].message.content,
            "preserved_terms": preserve_terms,
        }

    async def summarize_content(
        self,
        chapter_id: str,
        content: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Generate a summary of chapter content."""
        # For now, return a placeholder. In production, this would
        # fetch the actual chapter content and summarize it.
        return {
            "summary": f"Summary for chapter {chapter_id}. This chapter covers important concepts in Physical AI and Humanoid Robotics.",
            "key_concepts": [
                "Robot Operating System (ROS 2)",
                "Simulation environments",
                "AI-powered perception",
            ],
            "learning_objectives": [
                "Understand the fundamentals",
                "Apply concepts in practice",
                "Build working examples",
            ],
        }
