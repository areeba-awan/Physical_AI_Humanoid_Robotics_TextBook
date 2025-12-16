"""
Content Generator Agent for Physical AI Textbook

This agent generates educational content like quizzes, exercises, and code examples.
"""

from typing import List, Dict, Any, Literal, Optional
from pydantic import Field

from agents.base_agent import BaseAgent, AgentInput, AgentOutput, AgentTool


class GenerationInput(AgentInput):
    """Input for Content Generator Agent."""
    topic: str = Field(..., description="Topic to generate content about")
    content_type: Literal["quiz", "exercise", "explanation", "code"] = Field(
        ...,
        description="Type of content to generate"
    )
    difficulty: Literal["beginner", "intermediate", "advanced"] = Field(
        "intermediate",
        description="Difficulty level"
    )
    count: int = Field(5, ge=1, le=20, description="Number of items to generate")
    context: Optional[str] = Field(None, description="Additional context")


class GenerationOutput(AgentOutput):
    """Output from Content Generator Agent."""
    generated_content: Dict[str, Any]
    content_type: str


class ContentGeneratorAgent(BaseAgent):
    """
    Content Generator Agent for creating educational materials.

    Capabilities:
    - Generate multiple-choice quizzes
    - Create hands-on exercises
    - Write detailed explanations
    - Produce working code examples
    """

    name = "ContentGeneratorAgent"
    description = "Generates educational content like quizzes, exercises, and explanations"
    version = "1.0.0"

    SYSTEM_PROMPT = """You are an expert robotics educator creating content for the Physical AI textbook.

Content Types:

1. QUIZ - Multiple choice questions
   - Clear, unambiguous questions
   - 4 options with one correct answer
   - Explanation for each answer
   - Tests understanding, not memorization

2. EXERCISE - Hands-on practical tasks
   - Clear objective
   - Step-by-step instructions
   - Expected outcome
   - Helpful hints
   - Common mistakes to avoid

3. EXPLANATION - Detailed concept explanations
   - Core concept definition
   - How it works technically
   - Real-world applications
   - Code examples if relevant
   - Connections to other topics

4. CODE - Working code examples
   - Clean, well-commented code
   - Python (primary) or C++ (for ROS)
   - Executable and tested
   - Expected output shown
   - Variations and extensions

Difficulty Levels:
- Beginner: Simple concepts, lots of guidance, basic examples
- Intermediate: Applied concepts, real scenarios, some complexity
- Advanced: Complex problems, optimizations, edge cases

All content must:
- Be technically accurate
- Use proper ROS 2 / Python / C++ syntax
- Reference actual tools and libraries
- Be testable and verifiable
"""

    # Topic templates for different content types
    QUIZ_TEMPLATE = """
Question {num}:
{question}

A) {option_a}
B) {option_b}
C) {option_c}
D) {option_d}

Correct Answer: {correct}
Explanation: {explanation}
"""

    EXERCISE_TEMPLATE = """
Exercise {num}: {title}

Objective:
{objective}

Steps:
{steps}

Expected Outcome:
{outcome}

Hints:
{hints}
"""

    def get_tools(self) -> List[AgentTool]:
        return [
            AgentTool(
                name="validate_code",
                description="Validate that generated code is syntactically correct",
                parameters={
                    "code": {"type": "string"},
                    "language": {"type": "string"},
                }
            ),
            AgentTool(
                name="get_topic_context",
                description="Get additional context for a topic from the textbook",
                parameters={
                    "topic": {"type": "string"},
                }
            ),
        ]

    def get_system_prompt(self) -> str:
        return self.SYSTEM_PROMPT

    def _get_difficulty_adjustments(self, difficulty: str) -> Dict[str, Any]:
        """Get adjustments based on difficulty level."""
        adjustments = {
            "beginner": {
                "complexity": "low",
                "hints_level": "detailed",
                "code_comments": "extensive",
                "explanation_depth": "thorough",
            },
            "intermediate": {
                "complexity": "medium",
                "hints_level": "moderate",
                "code_comments": "standard",
                "explanation_depth": "balanced",
            },
            "advanced": {
                "complexity": "high",
                "hints_level": "minimal",
                "code_comments": "minimal",
                "explanation_depth": "concise",
            },
        }
        return adjustments.get(difficulty, adjustments["intermediate"])

    async def execute(self, input_data: GenerationInput) -> GenerationOutput:
        """
        Generate educational content.

        Steps:
        1. Determine content structure based on type
        2. Adjust for difficulty level
        3. Generate content items
        4. Validate and format output
        """
        adjustments = self._get_difficulty_adjustments(input_data.difficulty)

        # Placeholder content generation
        content = {
            "topic": input_data.topic,
            "type": input_data.content_type,
            "difficulty": input_data.difficulty,
            "count": input_data.count,
            "items": [],
        }

        if input_data.content_type == "quiz":
            for i in range(input_data.count):
                content["items"].append({
                    "number": i + 1,
                    "question": f"Sample question {i + 1} about {input_data.topic}",
                    "options": ["Option A", "Option B", "Option C", "Option D"],
                    "correct": "A",
                    "explanation": "Explanation of the correct answer.",
                })

        elif input_data.content_type == "exercise":
            for i in range(input_data.count):
                content["items"].append({
                    "number": i + 1,
                    "title": f"Exercise on {input_data.topic}",
                    "objective": f"Practice {input_data.topic} concepts",
                    "steps": ["Step 1", "Step 2", "Step 3"],
                    "expected_outcome": "Expected result description",
                    "hints": ["Hint 1", "Hint 2"] if adjustments["hints_level"] != "minimal" else [],
                })

        elif input_data.content_type == "code":
            for i in range(input_data.count):
                content["items"].append({
                    "number": i + 1,
                    "title": f"Code example for {input_data.topic}",
                    "language": "python",
                    "code": f"# Example {i + 1}\n# {input_data.topic}\nprint('Hello, Robotics!')",
                    "output": "Hello, Robotics!",
                    "explanation": "Code explanation here.",
                })

        else:  # explanation
            content["items"].append({
                "title": f"Understanding {input_data.topic}",
                "core_concept": f"Definition of {input_data.topic}",
                "how_it_works": "Technical explanation",
                "applications": ["Application 1", "Application 2"],
                "code_example": "# Optional code example",
            })

        return GenerationOutput(
            generated_content=content,
            content_type=input_data.content_type,
        )
