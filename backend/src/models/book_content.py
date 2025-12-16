from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from typing import List, Optional
from backend.src.models.models import BookContent as BookContentModel
from backend.src.core.database import get_db


class BookContentService:
    def __init__(self):
        pass

    async def create_book_content(
        self, 
        db: AsyncSession, 
        book_id: str, 
        chunk_text: str, 
        chunk_metadata: Optional[dict] = None
    ) -> BookContentModel:
        """
        Create a new book content chunk in the database.
        
        Args:
            db: Database session
            book_id: ID of the book this content belongs to
            chunk_text: The actual text content of the chunk
            chunk_metadata: Additional metadata for the chunk
            
        Returns:
            Created BookContent instance
        """
        db_book_content = BookContentModel(
            book_id=book_id,
            chunk_text=chunk_text,
            chunk_metadata=str(chunk_metadata) if chunk_metadata else None
        )
        
        db.add(db_book_content)
        await db.commit()
        await db.refresh(db_book_content)
        
        return db_book_content

    async def get_book_content_by_id(
        self, 
        db: AsyncSession, 
        content_id: str
    ) -> Optional[BookContentModel]:
        """
        Retrieve a book content chunk by its ID.
        
        Args:
            db: Database session
            content_id: ID of the content chunk to retrieve
            
        Returns:
            BookContent instance if found, None otherwise
        """
        result = await db.execute(
            select(BookContentModel).filter(BookContentModel.id == content_id)
        )
        return result.scalar_one_or_none()

    async def get_book_content_by_book_id(
        self, 
        db: AsyncSession, 
        book_id: str
    ) -> List[BookContentModel]:
        """
        Retrieve all content chunks for a specific book.
        
        Args:
            db: Database session
            book_id: ID of the book to retrieve content for
            
        Returns:
            List of BookContent instances for the book
        """
        result = await db.execute(
            select(BookContentModel).filter(BookContentModel.book_id == book_id)
        )
        return result.scalars().all()

    async def update_book_content(
        self, 
        db: AsyncSession, 
        content_id: str, 
        chunk_text: Optional[str] = None, 
        chunk_metadata: Optional[dict] = None
    ) -> Optional[BookContentModel]:
        """
        Update a book content chunk.
        
        Args:
            db: Database session
            content_id: ID of the content chunk to update
            chunk_text: New text content (optional)
            chunk_metadata: New metadata (optional)
            
        Returns:
            Updated BookContent instance if found, None otherwise
        """
        db_book_content = await self.get_book_content_by_id(db, content_id)
        if not db_book_content:
            return None

        if chunk_text is not None:
            db_book_content.chunk_text = chunk_text
        if chunk_metadata is not None:
            db_book_content.chunk_metadata = str(chunk_metadata)

        await db.commit()
        await db.refresh(db_book_content)
        
        return db_book_content

    async def delete_book_content(
        self, 
        db: AsyncSession, 
        content_id: str
    ) -> bool:
        """
        Delete a book content chunk.
        
        Args:
            db: Database session
            content_id: ID of the content chunk to delete
            
        Returns:
            True if deletion was successful, False otherwise
        """
        db_book_content = await self.get_book_content_by_id(db, content_id)
        if not db_book_content:
            return False

        await db.delete(db_book_content)
        await db.commit()
        
        return True


# Global instance for the application
book_content_service = BookContentService()