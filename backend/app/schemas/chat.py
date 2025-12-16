from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime
from uuid import UUID


class Citation(BaseModel):
    chapter_id: str
    section: str
    relevance: float = Field(..., ge=0, le=1)
    snippet: Optional[str] = None


class ChatRequest(BaseModel):
    message: str = Field(..., min_length=1, max_length=4000)
    context_chapter: Optional[str] = None


class ChatContextRequest(BaseModel):
    message: str = Field(..., min_length=1, max_length=4000)
    selected_text: str = Field(..., min_length=1)
    context_chapter: Optional[str] = None


class ChatResponse(BaseModel):
    id: UUID
    role: str
    content: str
    sources: List[Citation] = []
    timestamp: datetime

    class Config:
        from_attributes = True


class ChatMessageResponse(BaseModel):
    id: UUID
    role: str
    content: str
    sources: Optional[List[Citation]] = None
    context_chapter: Optional[str] = None
    selected_text: Optional[str] = None
    created_at: datetime

    class Config:
        from_attributes = True


class ChatHistoryResponse(BaseModel):
    messages: List[ChatMessageResponse]
    total: int
    has_more: bool
