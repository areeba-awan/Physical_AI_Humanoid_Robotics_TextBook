import os
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from typing import List, Optional
import uuid
from datetime import datetime, timedelta
from backend.src.models.models import Session as SessionModel
from backend.src.core.database import get_db


class SessionService:
    def __init__(self):
        self.session_timeout = int(os.getenv("SESSION_TIMEOUT_MINUTES", "30"))  # Default 30 minutes

    async def create_session(
        self, 
        db: AsyncSession, 
        book_id: str, 
        user_id: Optional[str] = None,
        session_metadata: Optional[dict] = None
    ) -> SessionModel:
        """
        Create a new chat session in the database.
        
        Args:
            db: Database session
            book_id: ID of the book for this session
            user_id: ID of the user (optional for anonymous sessions)
            session_metadata: Additional metadata for the session
            
        Returns:
            Created Session instance
        """
        expires_at = datetime.utcnow() + timedelta(minutes=self.session_timeout)
        
        # Parse metadata if it's a string, otherwise use as dict
        parsed_metadata = session_metadata
        if parsed_metadata and isinstance(parsed_metadata, str):
            try:
                import json
                parsed_metadata = json.loads(parsed_metadata)
            except:
                parsed_metadata = {}

        db_session = SessionModel(
            user_id=user_id,
            book_id=book_id,
            session_metadata=str(session_metadata) if session_metadata else None,
            current_page=parsed_metadata.get('current_page') if parsed_metadata else None,
            current_section=parsed_metadata.get('current_section') if parsed_metadata else None,
            reading_position=parsed_metadata.get('reading_position') if parsed_metadata else None,
            expires_at=expires_at
        )
        
        db.add(db_session)
        await db.commit()
        await db.refresh(db_session)
        
        return db_session

    async def get_session_by_id(
        self, 
        db: AsyncSession, 
        session_id: str
    ) -> Optional[SessionModel]:
        """
        Retrieve a session by its ID.
        
        Args:
            db: Database session
            session_id: ID of the session to retrieve
            
        Returns:
            Session instance if found and not expired, None otherwise
        """
        result = await db.execute(
            select(SessionModel)
            .filter(SessionModel.id == session_id)
            .filter(SessionModel.expires_at > datetime.utcnow())
        )
        session = result.scalar_one_or_none()
        
        # Update the session's last active time if it exists
        if session:
            session.updated_at = datetime.utcnow()
            await db.commit()
        
        return session

    async def get_sessions_by_user_id(
        self, 
        db: AsyncSession, 
        user_id: str
    ) -> List[SessionModel]:
        """
        Retrieve all active sessions for a specific user.
        
        Args:
            db: Database session
            user_id: ID of the user to retrieve sessions for
            
        Returns:
            List of active Session instances for the user
        """
        result = await db.execute(
            select(SessionModel)
            .filter(SessionModel.user_id == user_id)
            .filter(SessionModel.expires_at > datetime.utcnow())
        )
        return result.scalars().all()

    async def get_sessions_by_book_id(
        self, 
        db: AsyncSession, 
        book_id: str
    ) -> List[SessionModel]:
        """
        Retrieve all active sessions for a specific book.
        
        Args:
            db: Database session
            book_id: ID of the book to retrieve sessions for
            
        Returns:
            List of active Session instances for the book
        """
        result = await db.execute(
            select(SessionModel)
            .filter(SessionModel.book_id == book_id)
            .filter(SessionModel.expires_at > datetime.utcnow())
        )
        return result.scalars().all()

    async def update_session(
        self,
        db: AsyncSession,
        session_id: str,
        session_metadata: Optional[dict] = None,
        current_page: Optional[int] = None,
        current_section: Optional[str] = None,
        reading_position: Optional[str] = None,
        extend_session: bool = False
    ) -> Optional[SessionModel]:
        """
        Update a session.

        Args:
            db: Database session
            session_id: ID of the session to update
            session_metadata: New metadata (optional)
            current_page: Current page the user is reading (optional)
            current_section: Current section/chapter name (optional)
            reading_position: More detailed reading position (optional)
            extend_session: Whether to extend the session expiration time

        Returns:
            Updated Session instance if found, None otherwise
        """
        db_session = await self.get_session_by_id(db, session_id)
        if not db_session:
            return None

        if session_metadata is not None:
            db_session.session_metadata = str(session_metadata)

        if current_page is not None:
            db_session.current_page = current_page

        if current_section is not None:
            db_session.current_section = current_section

        if reading_position is not None:
            db_session.reading_position = reading_position

        if extend_session:
            db_session.expires_at = datetime.utcnow() + timedelta(minutes=self.session_timeout)

        await db.commit()
        await db.refresh(db_session)

        return db_session

    async def delete_session(
        self, 
        db: AsyncSession, 
        session_id: str
    ) -> bool:
        """
        Delete a session.
        
        Args:
            db: Database session
            session_id: ID of the session to delete
            
        Returns:
            True if deletion was successful, False otherwise
        """
        db_session = await self.get_session_by_id(db, session_id)
        if not db_session:
            # Try to find and delete even if expired
            result = await db.execute(
                select(SessionModel).filter(SessionModel.id == session_id)
            )
            db_session = result.scalar_one_or_none()
            if not db_session:
                return False

        await db.delete(db_session)
        await db.commit()
        
        return True
    
    async def expire_old_sessions(self, db: AsyncSession) -> int:
        """
        Expire and delete old sessions that have passed their expiration time.
        
        Args:
            db: Database session
            
        Returns:
            Number of deleted sessions
        """
        from sqlalchemy import delete
        
        stmt = delete(SessionModel).where(SessionModel.expires_at < datetime.utcnow())
        result = await db.execute(stmt)
        
        await db.commit()
        return result.rowcount


# Global instance for the application
session_service = SessionService()