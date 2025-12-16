"""
Chat Service with RAG Integration using OpenAI Responses API

Handles:
- Global book queries (RAG-powered)
- Selected text queries (strict mode)
- Streaming responses via Server-Sent Events
- Session-based chat memory
- Strict no-hallucination policy with verification
"""

from typing import Dict, Any, Optional, List, AsyncGenerator
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select, and_
from openai import AsyncOpenAI
from datetime import datetime, timedelta
import json
import logging
import uuid

from app.config import settings
from app.services.rag_service import RAGService
from app.models.chat import ChatMessage

logger = logging.getLogger(__name__)


class ChatService:
    """Service for chat operations using RAG with OpenAI Responses API."""

    # System prompt for global book queries - strict grounding
    SYSTEM_PROMPT_GLOBAL = """You are an expert AI assistant for the Physical AI & Humanoid Robotics textbook. You help students learn about ROS 2, Gazebo, NVIDIA Isaac, and VLA systems.

## CRITICAL RULES - YOU MUST FOLLOW THESE:

1. **ONLY answer using information from the provided context.**
   - Your answer must be directly supported by the context below
   - Do NOT use any external knowledge or training data
   - Every claim you make must be traceable to the context

2. **If the context doesn't contain the answer, say EXACTLY:**
   "I couldn't find information about this in the textbook. The provided context doesn't contain details about [topic]. Please try rephrasing your question or asking about a different topic covered in the book."

3. **NEVER make up information.** If you're unsure, say so.

4. **Always cite your sources:**
   - Reference the chapter and section when answering
   - Use format: "According to [Chapter] - [Section]..."

5. **Be helpful and educational:**
   - Explain complex concepts clearly
   - Use code examples when helpful (Python, C++, ROS 2)
   - Suggest related topics for further reading

## CONTEXT FROM THE TEXTBOOK:
{context}

## PREVIOUS CONVERSATION:
{chat_history}

Remember: Your answer MUST come from the context above. If the information isn't there, admit it."""

    # System prompt for selected text queries - EXTRA STRICT mode
    SYSTEM_PROMPT_SELECTED = """You are an AI assistant explaining specific text from the Physical AI & Humanoid Robotics textbook.

## ABSOLUTE RULES - NO EXCEPTIONS:

1. **ONLY answer based on the selected text provided below.**
   - Your ENTIRE answer must come from this text
   - Do NOT use any external knowledge whatsoever
   - Do NOT add information that isn't explicitly in the text

2. **If the selected text doesn't contain enough information to answer, respond EXACTLY with:**
   "The selected text does not contain enough information to answer this question."

3. **Stay focused on what the text explicitly states.**
   - Quote directly from the text when possible
   - Don't make inferences beyond what's written

## SELECTED TEXT:
\"\"\"{selected_text}\"\"\"

## PREVIOUS CONVERSATION:
{chat_history}

Your answer must come ONLY from the selected text above. Nothing else."""

    # Session timeout in hours
    SESSION_TIMEOUT_HOURS = 24

    # Maximum messages in chat history for context
    MAX_HISTORY_MESSAGES = 10

    def __init__(self, db: AsyncSession):
        # Determine which API to use: OpenAI first, then OpenRouter as fallback
        api_key = settings.openai_api_key or settings.openrouter_api_key
        base_url = settings.openai_base_url or settings.openrouter_base_url
        model = settings.openai_model if settings.openai_api_key else settings.openrouter_model

        if not api_key:
            raise ValueError("No API key configured. Please set either OPENAI_API_KEY or OPENROUTER_API_KEY")

        client_kwargs = {"api_key": api_key}
        if base_url:
            client_kwargs["base_url"] = base_url
        self.openai = AsyncOpenAI(**client_kwargs)
        self.model = model  # Use the appropriate model
        self.db = db
        self.rag_service = RAGService()

    async def get_chat_history(
        self,
        user_id: str,
        session_id: Optional[str] = None,
        limit: int = None,
    ) -> str:
        """
        Get recent chat history formatted for context.

        Supports session-based filtering for cleaner context.
        """
        if limit is None:
            limit = self.MAX_HISTORY_MESSAGES

        # Build query - filter by user and optionally by session time window
        query = select(ChatMessage).where(ChatMessage.user_id == user_id)

        # Add time-based session filter
        session_cutoff = datetime.utcnow() - timedelta(hours=self.SESSION_TIMEOUT_HOURS)
        query = query.where(ChatMessage.created_at >= session_cutoff)

        query = query.order_by(ChatMessage.created_at.desc()).limit(limit)

        result = await self.db.execute(query)
        messages = result.scalars().all()

        if not messages:
            return ""

        # Reverse to get chronological order
        messages = list(reversed(messages))

        history_parts = ["Previous conversation:"]
        for msg in messages:
            role = "User" if msg.role == "user" else "Assistant"
            # Truncate long messages in history to save tokens
            content = msg.content[:400] + "..." if len(msg.content) > 400 else msg.content
            history_parts.append(f"{role}: {content}")

        return "\n".join(history_parts)

    async def get_response(
        self,
        user_id: str,
        message: str,
        context_chapter: Optional[str] = None,
        session_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        """
        Get a response using Global Book Query mode.

        Uses OpenAI Responses API with RAG context.
        Includes anti-hallucination verification.
        """
        logger.info(f"Processing global query: {message[:100]}...")

        # Get relevant context from RAG
        context, citations, has_context = await self.rag_service.get_context_for_query(
            query=message,
            chapter_filter=context_chapter,
        )

        # Get chat history for context
        chat_history = await self.get_chat_history(user_id, session_id)

        # Handle case where no context found
        if not has_context:
            no_context_response = (
                "I couldn't find relevant information about this topic in the textbook. "
                "The textbook covers topics like ROS 2, Gazebo simulation, NVIDIA Isaac, "
                "and Vision-Language-Action (VLA) systems for humanoid robotics. "
                "Please try rephrasing your question or asking about these topics."
            )
            logger.warning(f"No context found for query: {message[:100]}")
            return {
                "content": no_context_response,
                "sources": [],
                "has_context": False,
                "grounded": True,  # This is a valid "no info" response
            }

        # Build system prompt with context
        system_prompt = self.SYSTEM_PROMPT_GLOBAL.format(
            context=context,
            chat_history=chat_history,
        )

        # Generate response using OpenAI
        try:
            response = await self.openai.chat.completions.create(
                model=settings.openai_model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": message},
                ],
                temperature=0.2,  # Low temperature for more factual responses
                max_tokens=1500,
                top_p=0.9,
            )

            answer = response.choices[0].message.content

            # Verify the answer is grounded in context
            is_grounded, confidence = await self.rag_service.verify_answer_in_context(
                query=message,
                context=context,
                answer=answer,
            )

            logger.info(f"Response generated. Grounded: {is_grounded}, Confidence: {confidence:.2f}")

            return {
                "content": answer,
                "sources": citations,
                "has_context": True,
                "grounded": is_grounded,
                "confidence": confidence,
            }

        except Exception as e:
            logger.error(f"OpenAI API error: {e}")
            raise

    async def get_response_with_selected_text(
        self,
        user_id: str,
        message: str,
        selected_text: str,
        context_chapter: Optional[str] = None,
        session_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        """
        Get a response using Selected Text Query mode (STRICT).

        ONLY answers from the provided selected text.
        Returns insufficient info message if text doesn't contain the answer.
        """
        logger.info(f"Processing selected text query: {message[:100]}...")

        # Analyze if selected text is sufficient
        context, citations, is_sufficient, analysis_message = \
            await self.rag_service.get_context_for_selected_text(
                query=message,
                selected_text=selected_text,
            )

        # Get chat history
        chat_history = await self.get_chat_history(user_id, session_id)

        # If text is clearly insufficient, return early
        if not is_sufficient:
            logger.info(f"Selected text insufficient: {analysis_message}")
            return {
                "content": "The selected text does not contain enough information to answer this question.",
                "sources": [],
                "is_sufficient": False,
                "selected_text_mode": True,
                "analysis": analysis_message,
            }

        # Build system prompt for strict selected text mode
        system_prompt = self.SYSTEM_PROMPT_SELECTED.format(
            selected_text=selected_text,
            chat_history=chat_history,
        )

        # Generate response
        try:
            response = await self.openai.chat.completions.create(
                model=settings.openai_model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": message},
                ],
                temperature=0.1,  # Very low temperature for strict factual responses
                max_tokens=1500,
                top_p=0.85,
            )

            content = response.choices[0].message.content

            # Check if the model itself said it couldn't answer
            insufficient_phrases = [
                "does not contain enough information",
                "doesn't contain enough information",
                "cannot answer",
                "not enough information",
                "no information about",
            ]

            model_says_insufficient = any(
                phrase.lower() in content.lower() for phrase in insufficient_phrases
            )

            return {
                "content": content,
                "sources": [],  # No external sources in selected text mode
                "is_sufficient": not model_says_insufficient,
                "selected_text_mode": True,
                "analysis": analysis_message,
            }

        except Exception as e:
            logger.error(f"OpenAI API error in selected text mode: {e}")
            raise

    async def stream_response(
        self,
        user_id: str,
        message: str,
        context_chapter: Optional[str] = None,
        session_id: Optional[str] = None,
    ) -> AsyncGenerator[str, None]:
        """
        Stream a response using Global Book Query mode.

        Yields Server-Sent Events (SSE) formatted chunks.
        """
        logger.info(f"Streaming global query: {message[:100]}...")

        # Get relevant context from RAG
        context, citations, has_context = await self.rag_service.get_context_for_query(
            query=message,
            chapter_filter=context_chapter,
        )

        # Get chat history
        chat_history = await self.get_chat_history(user_id, session_id)

        # Handle no context
        if not has_context:
            no_context_msg = (
                "I couldn't find relevant information about this topic in the textbook. "
                "Please try asking about ROS 2, Gazebo, NVIDIA Isaac, or VLA systems."
            )
            yield f"data: {json.dumps({'type': 'content', 'content': no_context_msg})}\n\n"
            yield f"data: {json.dumps({'type': 'done', 'sources': [], 'has_context': False})}\n\n"
            return

        # Build system prompt
        system_prompt = self.SYSTEM_PROMPT_GLOBAL.format(
            context=context,
            chat_history=chat_history,
        )

        # Stream response using OpenAI
        try:
            stream = await self.openai.chat.completions.create(
                model=settings.openai_model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": message},
                ],
                temperature=0.2,
                max_tokens=1500,
                stream=True,
            )

            full_content = ""
            async for chunk in stream:
                if chunk.choices[0].delta.content:
                    content = chunk.choices[0].delta.content
                    full_content += content
                    yield f"data: {json.dumps({'type': 'content', 'content': content})}\n\n"

            # Send final message with sources and metadata
            yield f"data: {json.dumps({'type': 'done', 'sources': citations, 'full_content': full_content, 'has_context': True})}\n\n"

        except Exception as e:
            logger.error(f"Streaming error: {e}")
            yield f"data: {json.dumps({'type': 'error', 'error': str(e)})}\n\n"

    async def stream_response_with_selected_text(
        self,
        user_id: str,
        message: str,
        selected_text: str,
        context_chapter: Optional[str] = None,
        session_id: Optional[str] = None,
    ) -> AsyncGenerator[str, None]:
        """
        Stream a response using Selected Text Query mode (STRICT).

        Yields Server-Sent Events (SSE) formatted chunks.
        """
        logger.info(f"Streaming selected text query: {message[:100]}...")

        # Analyze selected text
        context, citations, is_sufficient, analysis_message = \
            await self.rag_service.get_context_for_selected_text(
                query=message,
                selected_text=selected_text,
            )

        # If text is clearly insufficient, return early
        if not is_sufficient:
            insufficient_msg = "The selected text does not contain enough information to answer this question."
            yield f"data: {json.dumps({'type': 'content', 'content': insufficient_msg})}\n\n"
            yield f"data: {json.dumps({'type': 'done', 'sources': [], 'full_content': insufficient_msg, 'is_sufficient': False})}\n\n"
            return

        # Get chat history
        chat_history = await self.get_chat_history(user_id, session_id)

        # Build system prompt
        system_prompt = self.SYSTEM_PROMPT_SELECTED.format(
            selected_text=selected_text,
            chat_history=chat_history,
        )

        # Stream response
        try:
            stream = await self.openai.chat.completions.create(
                model=settings.openai_model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": message},
                ],
                temperature=0.1,
                max_tokens=1500,
                stream=True,
            )

            full_content = ""
            async for chunk in stream:
                if chunk.choices[0].delta.content:
                    content = chunk.choices[0].delta.content
                    full_content += content
                    yield f"data: {json.dumps({'type': 'content', 'content': content})}\n\n"

            # Send final message
            yield f"data: {json.dumps({'type': 'done', 'sources': [], 'full_content': full_content, 'is_sufficient': True})}\n\n"

        except Exception as e:
            logger.error(f"Streaming error in selected text mode: {e}")
            yield f"data: {json.dumps({'type': 'error', 'error': str(e)})}\n\n"

    async def save_message(
        self,
        user_id: str,
        role: str,
        content: str,
        sources: Optional[List[Dict]] = None,
        context_chapter: Optional[str] = None,
        selected_text: Optional[str] = None,
    ) -> ChatMessage:
        """Save a chat message to the database."""
        message = ChatMessage(
            user_id=user_id,
            role=role,
            content=content,
            sources=sources,
            context_chapter=context_chapter,
            selected_text=selected_text,
        )
        self.db.add(message)
        await self.db.flush()
        await self.db.refresh(message)
        return message

    async def clear_session_history(
        self,
        user_id: str,
        hours_back: Optional[int] = None,
    ) -> int:
        """
        Clear chat history for a user's session.

        Args:
            user_id: User ID
            hours_back: Optional hours to clear. If None, clears all.

        Returns:
            Number of messages deleted
        """
        query = select(ChatMessage).where(ChatMessage.user_id == user_id)

        if hours_back:
            cutoff = datetime.utcnow() - timedelta(hours=hours_back)
            query = query.where(ChatMessage.created_at >= cutoff)

        result = await self.db.execute(query)
        messages = result.scalars().all()

        count = len(messages)
        for message in messages:
            await self.db.delete(message)

        await self.db.commit()
        logger.info(f"Cleared {count} messages for user {user_id}")
        return count
