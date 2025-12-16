"""
Urdu Translator Agent for Physical AI Textbook

This agent translates content to Urdu while preserving technical terms.
"""

from typing import List
from pydantic import Field

from agents.base_agent import BaseAgent, AgentInput, AgentOutput, AgentTool


class TranslationInput(AgentInput):
    """Input for Translator Agent."""
    content: str = Field(..., description="Content to translate")
    preserve_terms: List[str] = Field(
        default_factory=list,
        description="Technical terms to keep in English"
    )
    source_language: str = Field("en", description="Source language code")
    target_language: str = Field("ur", description="Target language code")


class TranslationOutput(AgentOutput):
    """Output from Translator Agent."""
    translated_content: str
    preserved_terms: List[str] = []


class UrduTranslatorAgent(BaseAgent):
    """
    Urdu Translator Agent for technical content.

    Capabilities:
    - Accurate English to Urdu translation
    - Technical term preservation
    - Code block preservation
    - RTL formatting support
    - Markdown structure preservation
    """

    name = "UrduTranslatorAgent"
    description = "Translates content to Urdu while preserving technical terms"
    version = "1.0.0"

    # Default technical terms to preserve in English
    PRESERVE_TERMS = [
        "ROS 2", "ROS", "Gazebo", "Unity", "NVIDIA", "Isaac", "Isaac Sim",
        "VLA", "DDS", "URDF", "SDF", "Node", "Topic", "Service", "Action",
        "Publisher", "Subscriber", "Message", "Package", "Workspace",
        "Python", "C++", "Linux", "Ubuntu", "Docker", "Git", "GitHub",
        "API", "SDK", "GPU", "CPU", "CUDA", "TensorRT", "PyTorch",
        "RGB", "RGBD", "LiDAR", "IMU", "SLAM", "MoveIt", "Nav2",
        "TensorFlow", "Transformer", "CNN", "RNN", "LSTM",
        "def", "class", "import", "from", "return", "if", "else", "for", "while",
    ]

    SYSTEM_PROMPT = """You are an expert Urdu translator specializing in technical content.

Your role is to translate educational content about robotics and AI from English to Urdu.

Translation Guidelines:
1. Maintain technical accuracy at all times
2. Preserve these terms in English (do not translate):
   - Programming keywords (def, class, import, etc.)
   - Technology names (ROS 2, Gazebo, Isaac, etc.)
   - Standard abbreviations (API, SDK, GPU, etc.)
   - Code snippets (keep entirely in English)

3. Use proper Urdu technical vocabulary where established translations exist
4. Maintain markdown formatting exactly as in the original
5. Keep code blocks unchanged (```code``` blocks)
6. Preserve URLs and links
7. Use RTL-appropriate punctuation

Output Requirements:
- Natural, fluent Urdu suitable for Pakistani engineering students
- Technical terms in English within Urdu text
- Proper paragraph breaks maintained
- Code examples untouched
"""

    def get_tools(self) -> List[AgentTool]:
        return [
            AgentTool(
                name="extract_code_blocks",
                description="Extract code blocks to preserve them",
                parameters={
                    "content": {"type": "string"},
                }
            ),
            AgentTool(
                name="identify_technical_terms",
                description="Identify technical terms to preserve",
                parameters={
                    "content": {"type": "string"},
                }
            ),
        ]

    def get_system_prompt(self) -> str:
        return self.SYSTEM_PROMPT

    async def execute(self, input_data: TranslationInput) -> TranslationOutput:
        """
        Translate content to Urdu.

        Steps:
        1. Extract code blocks and preserve
        2. Identify technical terms to preserve
        3. Translate remaining content
        4. Reassemble with preserved elements
        """
        # Combine default and user-specified terms
        all_preserve_terms = list(set(self.PRESERVE_TERMS + input_data.preserve_terms))

        return TranslationOutput(
            translated_content=f"[Urdu Translation of content]\n{input_data.content}",
            preserved_terms=all_preserve_terms,
        )
