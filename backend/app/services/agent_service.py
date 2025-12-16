from typing import Dict, Any, List, Optional
from openai import AsyncOpenAI

from app.config import settings


class AgentService:
    """Service for AI agent operations."""

    def __init__(self):
        self.openai = AsyncOpenAI(api_key=settings.openai_api_key)

    async def summarize(
        self,
        content: str,
        summary_type: str = "detailed",
    ) -> Dict[str, Any]:
        """Summarize content using the ChapterSummarizerAgent."""
        prompts = {
            "brief": "Provide a 2-3 sentence summary capturing the main point.",
            "detailed": "Provide a comprehensive paragraph summarizing all key information.",
            "bullets": "Provide a bullet-point list of the main takeaways (5-10 points).",
        }

        system_prompt = f"""You are an expert at summarizing technical educational content about robotics and AI.

{prompts.get(summary_type, prompts['detailed'])}

Also extract:
1. Key concepts covered (as a list)
2. Learning objectives achieved (as a list)

Format your response as:
SUMMARY:
[Your summary here]

KEY_POINTS:
- Point 1
- Point 2
...
"""

        response = await self.openai.chat.completions.create(
            model=settings.openai_model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": content},
            ],
            temperature=0.5,
            max_tokens=1000,
        )

        result = response.choices[0].message.content

        # Parse response
        summary = result
        key_points = []

        if "SUMMARY:" in result:
            parts = result.split("KEY_POINTS:")
            summary = parts[0].replace("SUMMARY:", "").strip()
            if len(parts) > 1:
                key_points = [
                    p.strip().lstrip("- ")
                    for p in parts[1].strip().split("\n")
                    if p.strip()
                ]

        return {
            "summary": summary,
            "key_points": key_points,
        }

    async def translate(
        self,
        content: str,
        target_language: str = "ur",
        preserve_terms: List[str] = None,
    ) -> Dict[str, Any]:
        """Translate content using the UrduTranslatorAgent."""
        default_preserve = [
            "ROS 2", "Gazebo", "Unity", "NVIDIA", "Isaac", "VLA",
            "Python", "C++", "API", "SDK", "GPU", "CUDA",
        ]
        terms_to_preserve = preserve_terms or default_preserve

        system_prompt = f"""You are an expert Urdu translator for technical robotics content.

Translate to Urdu while preserving:
- All terms in this list: {', '.join(terms_to_preserve)}
- Code blocks (keep entirely in English)
- Markdown formatting
- URLs and links

Output only the translated content.
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
            "translated_content": response.choices[0].message.content,
            "preserved_terms": terms_to_preserve,
        }

    async def personalize(
        self,
        content: str,
        chapter_id: str,
        user_profile: Dict[str, Any],
    ) -> Dict[str, Any]:
        """Personalize content using the PersonalizationAgent."""
        system_prompt = f"""You are an expert at adapting educational content for individual learners.

Adapt this robotics content for a user with:
- Software Experience: {user_profile.get('software_experience')}
- Languages: {', '.join(user_profile.get('programming_languages', []))}
- ROS Experience: {'Yes' if user_profile.get('ros_experience') else 'No'}
- Linux: {user_profile.get('linux_experience')}
- Hardware: {user_profile.get('hardware_experience')}

Adaptations to make:
1. Adjust complexity based on experience level
2. Add examples in their known languages
3. Include tips based on their equipment
4. Skip basics if they're advanced, or explain more if they're beginners

Output the adapted content, then list adaptations made.
Format: CONTENT: [content] ADAPTATIONS: ["list", "of", "changes"]
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
        adaptations = ["Content personalized for your profile"]

        if "ADAPTATIONS:" in result:
            parts = result.split("ADAPTATIONS:")
            result = parts[0].replace("CONTENT:", "").strip()
            try:
                import json
                adaptations = json.loads(parts[1].strip())
            except:
                pass

        return {
            "personalized_content": result,
            "adaptations": adaptations,
        }

    async def generate(
        self,
        topic: str,
        content_type: str,
        difficulty: str = "intermediate",
        count: int = 5,
    ) -> Dict[str, Any]:
        """Generate educational content using the ContentGeneratorAgent."""
        type_prompts = {
            "quiz": f"""Generate {count} multiple-choice questions about {topic}.
Each question should have 4 options with one correct answer.
Format: Q1: [question]
A) option B) option C) option D) option
Correct: [letter]
Explanation: [why]""",

            "exercise": f"""Generate {count} hands-on exercises for {topic}.
Each exercise should have:
- Objective
- Steps to complete
- Expected outcome
- Hints""",

            "explanation": f"""Provide a detailed explanation of {topic}.
Include:
- Core concept
- How it works
- Real-world applications
- Code examples if relevant""",

            "code": f"""Generate {count} code examples demonstrating {topic}.
Include:
- Clear comments
- Working Python/C++ code
- Expected output
- Variations""",
        }

        difficulty_notes = {
            "beginner": "Keep it simple with lots of explanation.",
            "intermediate": "Balance theory and practice.",
            "advanced": "Include complex scenarios and optimizations.",
        }

        system_prompt = f"""You are an expert robotics educator creating content about {topic}.

Difficulty: {difficulty} - {difficulty_notes.get(difficulty)}

{type_prompts.get(content_type, type_prompts['explanation'])}
"""

        response = await self.openai.chat.completions.create(
            model=settings.openai_model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": f"Create {content_type} content for: {topic}"},
            ],
            temperature=0.8,
            max_tokens=2000,
        )

        return {
            "generated_content": {
                "topic": topic,
                "type": content_type,
                "difficulty": difficulty,
                "content": response.choices[0].message.content,
            },
        }
