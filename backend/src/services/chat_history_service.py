from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from typing import List, Optional
from backend.src.models.models import ChatMessage as ChatMessageModel
from backend.src.core.database import get_db
from datetime import datetime


class ChatMessageService:
    def __init__(self):
        pass

    async def create_chat_message(
        self, 
        db: AsyncSession, 
        session_id: str, 
        message_type: str, 
        content: str, 
        metadata: Optional[dict] = None
    ) -> ChatMessageModel:
        """
        Create a new chat message in the database.
        
        Args:
            db: Database session
            session_id: ID of the session this message belongs to
            message_type: Type of message (user, assistant, system)
            content: Content of the message
            metadata: Additional metadata for the message
            
        Returns:
            Created ChatMessage instance
        """
        db_chat_message = ChatMessageModel(
            session_id=session_id,
            message_type=message_type,
            content=content,
            metadata=str(metadata) if metadata else None
        )
        
        db.add(db_chat_message)
        await db.commit()
        await db.refresh(db_chat_message)
        
        return db_chat_message

    async def get_message_by_id(
        self, 
        db: AsyncSession, 
        message_id: str
    ) -> Optional[ChatMessageModel]:
        """
        Retrieve a chat message by its ID.
        
        Args:
            db: Database session
            message_id: ID of the message to retrieve
            
        Returns:
            ChatMessage instance if found, None otherwise
        """
        result = await db.execute(
            select(ChatMessageModel).filter(ChatMessageModel.id == message_id)
        )
        return result.scalar_one_or_none()

    async def get_messages_by_session_id(
        self, 
        db: AsyncSession, 
        session_id: str,
        limit: Optional[int] = None,
        offset: Optional[int] = 0
    ) -> List[ChatMessageModel]:
        """
        Retrieve all chat messages for a specific session.
        
        Args:
            db: Database session
            session_id: ID of the session to retrieve messages for
            limit: Maximum number of messages to return (optional)
            offset: Number of messages to skip (optional)
            
        Returns:
            List of ChatMessage instances for the session
        """
        query = select(ChatMessageModel).filter(ChatMessageModel.session_id == session_id).offset(offset)
        
        if limit:
            query = query.limit(limit)
        
        # Order by timestamp to get in chronological order
        query = query.order_by(ChatMessageModel.timestamp.asc())
        
        result = await db.execute(query)
        return result.scalars().all()

    async def get_messages_by_type(
        self, 
        db: AsyncSession, 
        session_id: str, 
        message_type: str
    ) -> List[ChatMessageModel]:
        """
        Retrieve chat messages of a specific type for a session.
        
        Args:
            db: Database session
            session_id: ID of the session to retrieve messages for
            message_type: Type of messages to retrieve (user, assistant, system)
            
        Returns:
            List of ChatMessage instances of the specified type
        """
        result = await db.execute(
            select(ChatMessageModel)
            .filter(ChatMessageModel.session_id == session_id)
            .filter(ChatMessageModel.message_type == message_type)
            .order_by(ChatMessageModel.timestamp.asc())
        )
        return result.scalars().all()

    async def update_chat_message(
        self, 
        db: AsyncSession, 
        message_id: str, 
        content: Optional[str] = None, 
        metadata: Optional[dict] = None
    ) -> Optional[ChatMessageModel]:
        """
        Update a chat message.
        
        Args:
            db: Database session
            message_id: ID of the message to update
            content: New content for the message (optional)
            metadata: New metadata for the message (optional)
            
        Returns:
            Updated ChatMessage instance if found, None otherwise
        """
        db_chat_message = await self.get_message_by_id(db, message_id)
        if not db_chat_message:
            return None

        if content is not None:
            db_chat_message.content = content
        if metadata is not None:
            db_chat_message.metadata = str(metadata)

        await db.commit()
        await db.refresh(db_chat_message)
        
        return db_chat_message

    async def delete_chat_message(
        self, 
        db: AsyncSession, 
        message_id: str
    ) -> bool:
        """
        Delete a chat message.
        
        Args:
            db: Database session
            message_id: ID of the message to delete
            
        Returns:
            True if deletion was successful, False otherwise
        """
        db_chat_message = await self.get_message_by_id(db, message_id)
        if not db_chat_message:
            return False

        await db.delete(db_chat_message)
        await db.commit()
        
        return True

    async def delete_session_messages(
        self, 
        db: AsyncSession, 
        session_id: str
    ) -> int:
        """
        Delete all messages for a specific session.
        
        Args:
            db: Database session
            session_id: ID of the session whose messages to delete
            
        Returns:
            Number of deleted messages
        """
        from sqlalchemy import delete
        
        stmt = delete(ChatMessageModel).where(ChatMessageModel.session_id == session_id)
        result = await db.execute(stmt)
        
        await db.commit()
        return result.rowcount


# Global instance for the application
chat_message_service = ChatMessageService()