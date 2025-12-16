"""
Claude Code Subagents for Physical AI Textbook Portal

This module contains specialized AI agents for:
- RAG Chat: Question answering with textbook context
- Summarization: Chapter and content summaries
- Translation: English to Urdu translation
- Personalization: Content adaptation based on user profile
- Content Generation: Quiz, exercise, and code generation
"""

from agents.base_agent import BaseAgent, AgentInput, AgentOutput
from agents.rag_chat_agent import RAGChatAgent
from agents.summarizer_agent import ChapterSummarizerAgent
from agents.translator_agent import UrduTranslatorAgent
from agents.personalizer_agent import PersonalizationAgent
from agents.content_generator_agent import ContentGeneratorAgent

__all__ = [
    "BaseAgent",
    "AgentInput",
    "AgentOutput",
    "RAGChatAgent",
    "ChapterSummarizerAgent",
    "UrduTranslatorAgent",
    "PersonalizationAgent",
    "ContentGeneratorAgent",
]
