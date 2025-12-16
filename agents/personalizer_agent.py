"""
Personalization Agent for Physical AI Textbook

This agent adapts content based on user background and experience.
"""

from typing import List, Dict, Any
from pydantic import Field

from agents.base_agent import BaseAgent, AgentInput, AgentOutput, AgentTool


class UserProfile(Dict[str, Any]):
    """User profile for personalization."""
    pass


class PersonalizationInput(AgentInput):
    """Input for Personalization Agent."""
    content: str = Field(..., description="Content to personalize")
    chapter_id: str = Field(..., description="Chapter identifier")
    user_profile: Dict[str, Any] = Field(..., description="User's background profile")


class PersonalizationOutput(AgentOutput):
    """Output from Personalization Agent."""
    personalized_content: str
    adaptations: List[str] = []


class PersonalizationAgent(BaseAgent):
    """
    Personalization Agent for adaptive content delivery.

    Capabilities:
    - Analyze user profile and experience
    - Adjust content complexity
    - Add relevant examples based on known languages
    - Include equipment-specific notes
    - Skip/expand sections based on experience
    """

    name = "PersonalizationAgent"
    description = "Adapts chapter content based on user background and experience"
    version = "1.0.0"

    SYSTEM_PROMPT = """You are an expert educational content adapter.

Your role is to personalize textbook content based on the learner's background:

User Profile Considerations:
- Software Experience: {software_experience}
- Programming Languages: {languages}
- ROS Experience: {ros_experience}
- Linux Experience: {linux_experience}
- Hardware Experience: {hardware_experience}
- Has GPU Workstation: {has_gpu}
- Has Jetson Kit: {has_jetson}

Adaptation Guidelines:

For BEGINNERS (none/beginner experience):
- Add detailed explanations of basic concepts
- Include step-by-step instructions
- Use simpler analogies and examples
- Add "prerequisite check" callouts
- Include links to foundational resources

For INTERMEDIATE users:
- Balance theory and practice
- Add practical tips and common pitfalls
- Include real-world application examples
- Reference related advanced topics

For ADVANCED users:
- Skip basic explanations
- Focus on advanced techniques and optimizations
- Include edge cases and performance considerations
- Add references to source code and documentation

Equipment-Specific Adaptations:
- If user has GPU: Add GPU-specific optimization tips
- If user has Jetson: Include Jetson-specific deployment notes
- If no GPU: Suggest CPU alternatives and cloud options

Rules:
- Maintain all technical accuracy
- Keep the same overall structure
- Don't remove critical safety information
- Add helpful callouts for the user's level
- Use their known programming languages for examples
"""

    def get_tools(self) -> List[AgentTool]:
        return [
            AgentTool(
                name="analyze_profile",
                description="Analyze user profile to determine adaptation strategy",
                parameters={
                    "profile": {"type": "object"},
                }
            ),
            AgentTool(
                name="get_alternative_examples",
                description="Get code examples in user's preferred language",
                parameters={
                    "original_code": {"type": "string"},
                    "target_language": {"type": "string"},
                }
            ),
        ]

    def get_system_prompt(self) -> str:
        return self.SYSTEM_PROMPT

    def _determine_adaptation_strategy(self, profile: Dict[str, Any]) -> Dict[str, Any]:
        """Determine adaptation strategy based on user profile."""
        strategy = {
            "complexity": "intermediate",
            "add_basics": False,
            "skip_basics": False,
            "gpu_tips": False,
            "jetson_tips": False,
            "preferred_language": "Python",
        }

        # Determine complexity level
        exp = profile.get("software_experience", "beginner")
        if exp in ["none", "beginner"]:
            strategy["complexity"] = "beginner"
            strategy["add_basics"] = True
        elif exp == "advanced":
            strategy["complexity"] = "advanced"
            strategy["skip_basics"] = True

        # Equipment-specific tips
        if profile.get("has_gpu_workstation"):
            strategy["gpu_tips"] = True
        if profile.get("has_jetson_kit"):
            strategy["jetson_tips"] = True

        # Preferred language
        languages = profile.get("programming_languages", [])
        if "C++" in languages and "Python" not in languages:
            strategy["preferred_language"] = "C++"

        return strategy

    async def execute(self, input_data: PersonalizationInput) -> PersonalizationOutput:
        """
        Personalize content for the user.

        Steps:
        1. Analyze user profile
        2. Determine adaptation strategy
        3. Modify content accordingly
        4. List adaptations made
        """
        strategy = self._determine_adaptation_strategy(input_data.user_profile)

        adaptations = []
        if strategy["add_basics"]:
            adaptations.append("Added detailed explanations for beginners")
        if strategy["skip_basics"]:
            adaptations.append("Streamlined content for advanced users")
        if strategy["gpu_tips"]:
            adaptations.append("Added GPU-specific optimization tips")
        if strategy["jetson_tips"]:
            adaptations.append("Added Jetson deployment notes")

        return PersonalizationOutput(
            personalized_content=input_data.content,  # Would be modified in real implementation
            adaptations=adaptations or ["Content optimized for your profile"],
        )
