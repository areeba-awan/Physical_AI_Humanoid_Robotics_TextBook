from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from typing import List, Optional
from backend.src.models.models import UserQuery as UserQueryModel
from backend.src.models.models import Session as SessionModel
from backend.src.core.database import get_db


class UserQueryService:
    def __init__(self):
        pass

    async def create_user_query(
        self, 
        db: AsyncSession, 
        session_id: str, 
        query_text: str, 
        query_metadata: Optional[dict] = None
    ) -> UserQueryModel:
        """
        Create a new user query in the database.
        
        Args:
            db: Database session
            session_id: ID of the session this query belongs to
            query_text: The text of the user's query
            query_metadata: Additional metadata for the query
            
        Returns:
            Created UserQuery instance
        """
        db_user_query = UserQueryModel(
            session_id=session_id,
            query_text=query_text,
            query_metadata=str(query_metadata) if query_metadata else None
        )
        
        db.add(db_user_query)
        await db.commit()
        await db.refresh(db_user_query)
        
        return db_user_query

    async def get_user_query_by_id(
        self, 
        db: AsyncSession, 
        query_id: str
    ) -> Optional[UserQueryModel]:
        """
        Retrieve a user query by its ID.
        
        Args:
            db: Database session
            query_id: ID of the query to retrieve
            
        Returns:
            UserQuery instance if found, None otherwise
        """
        result = await db.execute(
            select(UserQueryModel).filter(UserQueryModel.id == query_id)
        )
        return result.scalar_one_or_none()

    async def get_queries_by_session_id(
        self, 
        db: AsyncSession, 
        session_id: str
    ) -> List[UserQueryModel]:
        """
        Retrieve all queries for a specific session.
        
        Args:
            db: Database session
            session_id: ID of the session to retrieve queries for
            
        Returns:
            List of UserQuery instances for the session
        """
        result = await db.execute(
            select(UserQueryModel).filter(UserQueryModel.session_id == session_id)
        )
        return result.scalars().all()

    async def get_recent_queries(
        self, 
        db: AsyncSession, 
        limit: int = 10
    ) -> List[UserQueryModel]:
        """
        Retrieve the most recent queries.
        
        Args:
            db: Database session
            limit: Maximum number of queries to return
            
        Returns:
            List of the most recent UserQuery instances
        """
        from sqlalchemy import desc
        result = await db.execute(
            select(UserQueryModel)
            .order_by(desc(UserQueryModel.created_at))
            .limit(limit)
        )
        return result.scalars().all()

    async def update_user_query(
        self, 
        db: AsyncSession, 
        query_id: str, 
        query_text: Optional[str] = None, 
        query_metadata: Optional[dict] = None
    ) -> Optional[UserQueryModel]:
        """
        Update a user query.
        
        Args:
            db: Database session
            query_id: ID of the query to update
            query_text: New query text (optional)
            query_metadata: New metadata (optional)
            
        Returns:
            Updated UserQuery instance if found, None otherwise
        """
        db_user_query = await self.get_user_query_by_id(db, query_id)
        if not db_user_query:
            return None

        if query_text is not None:
            db_user_query.query_text = query_text
        if query_metadata is not None:
            db_user_query.query_metadata = str(query_metadata)

        await db.commit()
        await db.refresh(db_user_query)
        
        return db_user_query

    async def delete_user_query(
        self, 
        db: AsyncSession, 
        query_id: str
    ) -> bool:
        """
        Delete a user query.
        
        Args:
            db: Database session
            query_id: ID of the query to delete
            
        Returns:
            True if deletion was successful, False otherwise
        """
        db_user_query = await self.get_user_query_by_id(db, query_id)
        if not db_user_query:
            return False

        await db.delete(db_user_query)
        await db.commit()
        
        return True


# Global instance for the application
user_query_service = UserQueryService()