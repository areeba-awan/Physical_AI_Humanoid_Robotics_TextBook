from fastapi import APIRouter, HTTPException, Depends
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Optional
import uuid
import json

from backend.src.core.database import get_db
from backend.src.services.session_service import session_service
from backend.src.models.models import Session, UserQuery, GeneratedResponse
from pydantic import BaseModel

router = APIRouter()

# Pydantic models for request/response
class CreateSessionRequest(BaseModel):
    book_id: str
    user_id: Optional[str] = None
    metadata: Optional[dict] = None

class SessionResponse(BaseModel):
    id: str
    book_id: str
    user_id: Optional[str]
    metadata: Optional[dict]
    created_at: str
    updated_at: str
    expires_at: str

@router.post("/session", response_model=SessionResponse, status_code=201)
async def create_session(
    request: CreateSessionRequest,
    db: AsyncSession = Depends(get_db)
):
    """
    Create a new chat session for a user reading a book.
    """
    try:
        # Validate book_id format
        try:
            uuid.UUID(request.book_id)
        except ValueError:
            raise HTTPException(status_code=400, detail="Invalid book_id format")
        
        # Validate user_id format if provided
        if request.user_id:
            try:
                uuid.UUID(request.user_id)
            except ValueError:
                raise HTTPException(status_code=400, detail="Invalid user_id format")
        
        # Create the session using the service
        session = await session_service.create_session(
            db=db,
            book_id=request.book_id,
            user_id=request.user_id,
            session_metadata=request.metadata
        )
        
        return SessionResponse(
            id=str(session.id),
            book_id=str(session.book_id),
            user_id=str(session.user_id) if session.user_id else None,
            metadata=json.loads(session.session_metadata) if session.session_metadata else None,
            created_at=session.created_at.isoformat(),
            updated_at=session.updated_at.isoformat(),
            expires_at=session.expires_at.isoformat()
        )
    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        error_msg = f"Error creating session: {str(e)}"
        raise HTTPException(status_code=500, detail=error_msg)


@router.get("/session/{session_id}", response_model=SessionResponse)
async def get_session(
    session_id: str,
    db: AsyncSession = Depends(get_db)
):
    """
    Retrieve details about a specific chat session.
    """
    try:
        # Validate session_id format
        try:
            uuid.UUID(session_id)
        except ValueError:
            raise HTTPException(status_code=400, detail="Invalid session_id format")
        
        # Get the session using the service
        session = await session_service.get_session_by_id(db, session_id)
        
        if not session:
            raise HTTPException(status_code=404, detail="Session not found or expired")
        
        return SessionResponse(
            id=str(session.id),
            book_id=str(session.book_id),
            user_id=str(session.user_id) if session.user_id else None,
            metadata=json.loads(session.session_metadata) if session.session_metadata else None,
            created_at=session.created_at.isoformat(),
            updated_at=session.updated_at.isoformat(),
            expires_at=session.expires_at.isoformat()
        )
    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        error_msg = f"Error retrieving session: {str(e)}"
        raise HTTPException(status_code=500, detail=error_msg)