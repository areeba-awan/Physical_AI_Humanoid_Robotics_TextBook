"""
Chat API Endpoints

Handles:
- POST /chat - Global book query
- POST /chat/selected-text - Selected text query (STRICT mode)
- POST /chat/stream - Streaming global query
- POST /chat/selected-text/stream - Streaming selected text query
- GET /chat/history - Get chat history
- DELETE /chat/history - Clear chat history
"""

from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.responses import StreamingResponse
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select, func
from typing import Optional
from datetime import datetime
import json

from app.database import get_db
from app.models import User, ChatMessage
from app.schemas import (
    ChatRequest,
    ChatContextRequest,
    ChatResponse,
    ChatHistoryResponse,
    ChatMessageResponse,
    Citation,
)
from app.core.auth import get_current_user
from app.services.chat_service import ChatService

router = APIRouter()


@router.post("", response_model=ChatResponse)
async def send_message(
    data: ChatRequest,
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """
    Send a message to the RAG chatbot (Global Book Query mode).

    This endpoint:
    1. Searches the entire book for relevant context
    2. Uses session-based chat memory
    3. Returns answer with source citations
    4. Never hallucinates - only answers from book content
    """
    # Save user message
    user_message = ChatMessage(
        user_id=user.id,
        role="user",
        content=data.message,
        context_chapter=data.context_chapter,
    )
    db.add(user_message)
    await db.flush()

    # Get RAG response
    chat_service = ChatService(db)
    response = await chat_service.get_response(
        user_id=str(user.id),
        message=data.message,
        context_chapter=data.context_chapter,
    )

    # Save assistant message
    assistant_message = ChatMessage(
        user_id=user.id,
        role="assistant",
        content=response["content"],
        sources=response.get("sources"),
        context_chapter=data.context_chapter,
    )
    db.add(assistant_message)
    await db.commit()
    await db.refresh(assistant_message)

    return ChatResponse(
        id=assistant_message.id,
        role="assistant",
        content=response["content"],
        sources=[Citation(**s) for s in response.get("sources", [])],
        timestamp=assistant_message.created_at,
    )


@router.post("/selected-text", response_model=ChatResponse)
async def send_message_with_selected_text(
    data: ChatContextRequest,
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """
    Send a message with selected text (STRICT Selected Text Query mode).

    This endpoint:
    1. ONLY answers from the provided selected text
    2. Returns "insufficient information" if text doesn't contain answer
    3. Never searches the broader book - purely selected text based
    4. No hallucinations allowed
    """
    # Save user message
    user_message = ChatMessage(
        user_id=user.id,
        role="user",
        content=data.message,
        context_chapter=data.context_chapter,
        selected_text=data.selected_text,
    )
    db.add(user_message)
    await db.flush()

    # Get RAG response with selected text (STRICT mode)
    chat_service = ChatService(db)
    response = await chat_service.get_response_with_selected_text(
        user_id=str(user.id),
        message=data.message,
        selected_text=data.selected_text,
        context_chapter=data.context_chapter,
    )

    # Save assistant message
    assistant_message = ChatMessage(
        user_id=user.id,
        role="assistant",
        content=response["content"],
        sources=response.get("sources"),
        context_chapter=data.context_chapter,
    )
    db.add(assistant_message)
    await db.commit()
    await db.refresh(assistant_message)

    return ChatResponse(
        id=assistant_message.id,
        role="assistant",
        content=response["content"],
        sources=[Citation(**s) for s in response.get("sources", [])],
        timestamp=assistant_message.created_at,
    )


# Keep old endpoint for backward compatibility
@router.post("/context", response_model=ChatResponse)
async def send_message_with_context(
    data: ChatContextRequest,
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """
    [DEPRECATED] Use /chat/selected-text instead.
    Send a message with selected text context.
    """
    return await send_message_with_selected_text(data, user, db)


@router.post("/stream")
async def stream_message(
    data: ChatRequest,
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """
    Stream a response from the RAG chatbot (Global Book Query mode).

    Returns Server-Sent Events (SSE) stream with:
    - type: "content" - chunks of the response
    - type: "done" - final message with sources
    """
    # Save user message first
    user_message = ChatMessage(
        user_id=user.id,
        role="user",
        content=data.message,
        context_chapter=data.context_chapter,
    )
    db.add(user_message)
    await db.commit()

    chat_service = ChatService(db)

    async def generate():
        full_content = ""
        sources = []

        async for chunk in chat_service.stream_response(
            user_id=str(user.id),
            message=data.message,
            context_chapter=data.context_chapter,
        ):
            yield chunk

            # Parse chunk to capture full content for saving
            if chunk.startswith("data: "):
                try:
                    chunk_data = json.loads(chunk[6:].strip())
                    if chunk_data.get("type") == "done":
                        full_content = chunk_data.get("full_content", "")
                        sources = chunk_data.get("sources", [])
                except:
                    pass

        # Save assistant message after streaming completes
        if full_content:
            assistant_message = ChatMessage(
                user_id=user.id,
                role="assistant",
                content=full_content,
                sources=sources,
                context_chapter=data.context_chapter,
            )
            db.add(assistant_message)
            await db.commit()

    return StreamingResponse(
        generate(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",
        }
    )


@router.post("/selected-text/stream")
async def stream_message_with_selected_text(
    data: ChatContextRequest,
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """
    Stream a response using selected text (STRICT mode).

    Returns Server-Sent Events (SSE) stream.
    """
    # Save user message first
    user_message = ChatMessage(
        user_id=user.id,
        role="user",
        content=data.message,
        context_chapter=data.context_chapter,
        selected_text=data.selected_text,
    )
    db.add(user_message)
    await db.commit()

    chat_service = ChatService(db)

    async def generate():
        full_content = ""
        sources = []

        async for chunk in chat_service.stream_response_with_selected_text(
            user_id=str(user.id),
            message=data.message,
            selected_text=data.selected_text,
            context_chapter=data.context_chapter,
        ):
            yield chunk

            # Parse chunk to capture full content for saving
            if chunk.startswith("data: "):
                try:
                    chunk_data = json.loads(chunk[6:].strip())
                    if chunk_data.get("type") == "done":
                        full_content = chunk_data.get("full_content", "")
                        sources = chunk_data.get("sources", [])
                except:
                    pass

        # Save assistant message after streaming completes
        if full_content:
            assistant_message = ChatMessage(
                user_id=user.id,
                role="assistant",
                content=full_content,
                sources=sources,
                context_chapter=data.context_chapter,
            )
            db.add(assistant_message)
            await db.commit()

    return StreamingResponse(
        generate(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",
        }
    )


@router.get("/history", response_model=ChatHistoryResponse)
async def get_chat_history(
    limit: int = 50,
    offset: int = 0,
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """Get the user's chat history."""
    # Get total count
    count_result = await db.execute(
        select(func.count(ChatMessage.id)).where(ChatMessage.user_id == user.id)
    )
    total = count_result.scalar()

    # Get messages
    result = await db.execute(
        select(ChatMessage)
        .where(ChatMessage.user_id == user.id)
        .order_by(ChatMessage.created_at.desc())
        .offset(offset)
        .limit(limit)
    )
    messages = result.scalars().all()

    return ChatHistoryResponse(
        messages=[
            ChatMessageResponse(
                id=msg.id,
                role=msg.role,
                content=msg.content,
                sources=[Citation(**s) for s in msg.sources] if msg.sources else None,
                context_chapter=msg.context_chapter,
                selected_text=msg.selected_text,
                created_at=msg.created_at,
            )
            for msg in reversed(messages)
        ],
        total=total,
        has_more=offset + limit < total,
    )


@router.delete("/history")
async def clear_chat_history(
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """Clear the user's chat history."""
    result = await db.execute(
        select(ChatMessage).where(ChatMessage.user_id == user.id)
    )
    messages = result.scalars().all()

    for message in messages:
        await db.delete(message)

    await db.commit()

    return {"success": True, "message": "Chat history cleared"}
