from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from typing import List, Optional
from backend.src.models.models import ContentReference as ContentReferenceModel
from backend.src.core.database import get_db


class ContentReferenceService:
    def __init__(self):
        pass

    async def create_content_reference(
        self, 
        db: AsyncSession, 
        response_id: str, 
        content_id: str, 
        text_snippet: Optional[str] = None,
        page_number: Optional[int] = None,
        section_title: Optional[str] = None
    ) -> ContentReferenceModel:
        """
        Create a new content reference in the database.
        
        Args:
            db: Database session
            response_id: ID of the generated response this reference belongs to
            content_id: ID of the book content being referenced
            text_snippet: A snippet of the referenced text
            page_number: Page number where the content appears
            section_title: Title of the section containing the content
            
        Returns:
            Created ContentReference instance
        """
        db_content_reference = ContentReferenceModel(
            response_id=response_id,
            content_id=content_id,
            text_snippet=text_snippet,
            page_number=page_number,
            section_title=section_title
        )
        
        db.add(db_content_reference)
        await db.commit()
        await db.refresh(db_content_reference)
        
        return db_content_reference

    async def get_content_reference_by_id(
        self, 
        db: AsyncSession, 
        reference_id: str
    ) -> Optional[ContentReferenceModel]:
        """
        Retrieve a content reference by its ID.
        
        Args:
            db: Database session
            reference_id: ID of the content reference to retrieve
            
        Returns:
            ContentReference instance if found, None otherwise
        """
        result = await db.execute(
            select(ContentReferenceModel).filter(ContentReferenceModel.id == reference_id)
        )
        return result.scalar_one_or_none()

    async def get_references_by_response_id(
        self, 
        db: AsyncSession, 
        response_id: str
    ) -> List[ContentReferenceModel]:
        """
        Retrieve all content references for a specific response.
        
        Args:
            db: Database session
            response_id: ID of the response to retrieve references for
            
        Returns:
            List of ContentReference instances for the response
        """
        result = await db.execute(
            select(ContentReferenceModel).filter(ContentReferenceModel.response_id == response_id)
        )
        return result.scalars().all()

    async def get_references_by_content_id(
        self, 
        db: AsyncSession, 
        content_id: str
    ) -> List[ContentReferenceModel]:
        """
        Retrieve all content references pointing to a specific content chunk.
        
        Args:
            db: Database session
            content_id: ID of the book content to find references for
            
        Returns:
            List of ContentReference instances pointing to the content
        """
        result = await db.execute(
            select(ContentReferenceModel).filter(ContentReferenceModel.content_id == content_id)
        )
        return result.scalars().all()

    async def update_content_reference(
        self, 
        db: AsyncSession, 
        reference_id: str, 
        text_snippet: Optional[str] = None, 
        page_number: Optional[int] = None,
        section_title: Optional[str] = None
    ) -> Optional[ContentReferenceModel]:
        """
        Update a content reference.
        
        Args:
            db: Database session
            reference_id: ID of the reference to update
            text_snippet: New text snippet (optional)
            page_number: New page number (optional)
            section_title: New section title (optional)
            
        Returns:
            Updated ContentReference instance if found, None otherwise
        """
        db_content_reference = await self.get_content_reference_by_id(db, reference_id)
        if not db_content_reference:
            return None

        if text_snippet is not None:
            db_content_reference.text_snippet = text_snippet
        if page_number is not None:
            db_content_reference.page_number = page_number
        if section_title is not None:
            db_content_reference.section_title = section_title

        await db.commit()
        await db.refresh(db_content_reference)
        
        return db_content_reference

    async def delete_content_reference(
        self, 
        db: AsyncSession, 
        reference_id: str
    ) -> bool:
        """
        Delete a content reference.
        
        Args:
            db: Database session
            reference_id: ID of the reference to delete
            
        Returns:
            True if deletion was successful, False otherwise
        """
        db_content_reference = await self.get_content_reference_by_id(db, reference_id)
        if not db_content_reference:
            return False

        await db.delete(db_content_reference)
        await db.commit()
        
        return True


# Global instance for the application
content_reference_service = ContentReferenceService()