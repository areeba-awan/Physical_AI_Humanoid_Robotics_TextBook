"""
Chapter Summarizer Agent for Physical AI Textbook

This agent generates concise summaries of textbook chapters.
"""

from typing import List, Literal
from pydantic import Field

from agents.base_agent import BaseAgent, AgentInput, AgentOutput, AgentTool


class SummaryInput(AgentInput):
    """Input for Summarizer Agent."""
    content: str = Field(..., description="Content to summarize")
    summary_type: Literal["brief", "detailed", "bullets"] = Field(
        "detailed",
        description="Type of summary to generate"
    )


class SummaryOutput(AgentOutput):
    """Output from Summarizer Agent."""
    summary: str
    key_concepts: List[str] = []
    learning_objectives: List[str] = []


class ChapterSummarizerAgent(BaseAgent):
    """
    Chapter Summarizer Agent for creating concise summaries.

    Capabilities:
    - Generate brief summaries (2-3 sentences)
    - Generate detailed summaries (full paragraph)
    - Generate bullet-point summaries
    - Extract key concepts
    - Identify learning objectives
    """

    name = "ChapterSummarizerAgent"
    description = "Generates concise summaries of textbook chapters"
    version = "1.0.0"

    SYSTEM_PROMPT = """You are an expert at summarizing technical educational content.

Your role is to create helpful summaries of robotics and AI textbook chapters.

Summary Types:
- brief: 2-3 sentences capturing the main point
- detailed: Full paragraph with key information
- bullets: Bullet-point list of main takeaways

Always extract:
1. Key concepts covered in the content
2. Learning objectives that readers should achieve
3. Prerequisites referenced (if any)
4. Practical applications mentioned

Guidelines:
- Be accurate to the source material
- Make summaries accessible to learners
- Focus on actionable knowledge
- Highlight connections between topics
"""

    def get_tools(self) -> List[AgentTool]:
        return [
            AgentTool(
                name="extract_concepts",
                description="Extract key concepts from content",
                parameters={
                    "content": {"type": "string"},
                }
            ),
            AgentTool(
                name="identify_objectives",
                description="Identify learning objectives",
                parameters={
                    "content": {"type": "string"},
                }
            ),
        ]

    def get_system_prompt(self) -> str:
        return self.SYSTEM_PROMPT

    async def execute(self, input_data: SummaryInput) -> SummaryOutput:
        """
        Generate summary of provided content.

        Steps:
        1. Parse the content
        2. Extract key sections
        3. Generate summary based on type
        4. Extract concepts and objectives
        """
        return SummaryOutput(
            summary=f"Summary of the provided content ({input_data.summary_type} format).",
            key_concepts=["Concept 1", "Concept 2", "Concept 3"],
            learning_objectives=["Objective 1", "Objective 2"],
        )
