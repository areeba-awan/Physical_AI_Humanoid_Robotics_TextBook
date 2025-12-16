"""
RAG Chat Agent for Physical AI Textbook

This agent answers questions about the textbook using
Retrieval-Augmented Generation (RAG).
"""

from typing import List, Optional, Dict, Any
from pydantic import BaseModel, Field

from agents.base_agent import BaseAgent, AgentInput, AgentOutput, AgentTool


class RAGChatInput(AgentInput):
    """Input for RAG Chat Agent."""
    query: str = Field(..., description="The user's question")
    selected_text: Optional[str] = Field(None, description="Selected text for context")
    chapter_context: Optional[str] = Field(None, description="Current chapter ID")
    chat_history: List[Dict[str, str]] = Field(default_factory=list)


class Citation(BaseModel):
    """Citation reference to textbook content."""
    chapter_id: str
    section: str
    relevance: float = Field(..., ge=0, le=1)
    snippet: Optional[str] = None


class RAGChatOutput(AgentOutput):
    """Output from RAG Chat Agent."""
    answer: str
    sources: List[Citation] = []
    confidence: float = Field(0.0, ge=0, le=1)


class RAGChatAgent(BaseAgent):
    """
    RAG Chat Agent for answering questions about the Physical AI textbook.

    Capabilities:
    - Semantic search across book content
    - Context-aware responses
    - Citation of sources
    - Selected text analysis
    """

    name = "RAGChatAgent"
    description = "Answers questions about the Physical AI textbook using RAG"
    version = "1.0.0"

    SYSTEM_PROMPT = """You are an expert AI assistant for the Physical AI & Humanoid Robotics textbook.

Your role is to:
1. Answer questions accurately based on the provided context from the textbook
2. Cite your sources by referencing chapter and section names
3. Explain complex robotics and AI concepts in an accessible way
4. Guide learners through difficult topics
5. Recommend relevant chapters for further reading

When answering:
- Always ground your answers in the provided context
- If the context doesn't contain enough information, say so
- Use code examples when helpful (Python, C++, ROS 2)
- Be encouraging and supportive of learners
- Format code blocks with proper syntax highlighting

If the user provides selected text, focus your explanation on that specific content.

Textbook Topics:
- Module 1: ROS 2 (Robot Operating System)
- Module 2: Gazebo & Unity Simulation
- Module 3: NVIDIA Isaac
- Module 4: Vision-Language-Action (VLA) Systems
"""

    def get_tools(self) -> List[AgentTool]:
        """Define tools available to this agent."""
        return [
            AgentTool(
                name="search_textbook",
                description="Search the textbook for relevant content",
                parameters={
                    "query": {"type": "string", "description": "Search query"},
                    "chapter_filter": {"type": "string", "description": "Optional chapter ID"},
                    "limit": {"type": "integer", "description": "Max results", "default": 5},
                }
            ),
            AgentTool(
                name="get_chapter_content",
                description="Retrieve full content of a specific chapter",
                parameters={
                    "chapter_id": {"type": "string", "description": "Chapter identifier"},
                }
            ),
        ]

    def get_system_prompt(self) -> str:
        return self.SYSTEM_PROMPT

    async def execute(self, input_data: RAGChatInput) -> RAGChatOutput:
        """
        Execute RAG chat query.

        Steps:
        1. Search Qdrant for relevant chunks
        2. Build context with citations
        3. Generate response with OpenAI
        4. Return answer with sources
        """
        # This would integrate with the RAGService
        # For now, return a placeholder response
        return RAGChatOutput(
            answer=f"Based on the textbook content, here's my answer to: {input_data.query}",
            sources=[
                Citation(
                    chapter_id="chapter-1-1",
                    section="Introduction to ROS 2",
                    relevance=0.95,
                )
            ],
            confidence=0.85,
        )
