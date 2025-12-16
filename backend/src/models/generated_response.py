from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from typing import List, Optional
from backend.src.models.models import GeneratedResponse as GeneratedResponseModel
from backend.src.models.models import ContentReference
from backend.src.core.database import get_db


class GeneratedResponseService:
    def __init__(self):
        pass

    async def create_generated_response(
        self,
        db: AsyncSession,
        query_id: str,
        response_text: str,
        referenced_content: Optional[dict] = None,
        confidence_score: Optional[float] = None,
        content_references: Optional[List[dict]] = None  # List of content reference data
    ) -> GeneratedResponseModel:
        """
        Create a new generated response in the database.

        Args:
            db: Database session
            query_id: ID of the query this response is for
            response_text: The text of the AI-generated response
            referenced_content: Content referenced in the response
            confidence_score: Confidence score of the response
            content_references: Optional list of content references to create

        Returns:
            Created GeneratedResponse instance
        """
        db_generated_response = GeneratedResponseModel(
            query_id=query_id,
            response_text=response_text,
            referenced_content=str(referenced_content) if referenced_content else None,
            confidence_score=confidence_score
        )

        db.add(db_generated_response)
        await db.flush()  # To get the ID before committing

        # If content references are provided, create them
        if content_references:
            from backend.src.models.content_reference import content_reference_service
            for ref_data in content_references:
                await content_reference_service.create_content_reference(
                    db=db,
                    response_id=str(db_generated_response.id),
                    content_id=ref_data.get('content_id'),
                    text_snippet=ref_data.get('text_snippet'),
                    page_number=ref_data.get('page_number'),
                    section_title=ref_data.get('section_title')
                )

        await db.commit()
        await db.refresh(db_generated_response)

        return db_generated_response

    async def get_generated_response_by_id(
        self,
        db: AsyncSession,
        response_id: str
    ) -> Optional[GeneratedResponseModel]:
        """
        Retrieve a generated response by its ID.

        Args:
            db: Database session
            response_id: ID of the response to retrieve

        Returns:
            GeneratedResponse instance if found, None otherwise
        """
        result = await db.execute(
            select(GeneratedResponseModel)
            .filter(GeneratedResponseModel.id == response_id)
        )
        return result.scalar_one_or_none()

    async def get_response_by_query_id(
        self,
        db: AsyncSession,
        query_id: str
    ) -> Optional[GeneratedResponseModel]:
        """
        Retrieve the generated response for a specific query.

        Args:
            db: Database session
            query_id: ID of the query to get response for

        Returns:
            GeneratedResponse instance if found, None otherwise
        """
        result = await db.execute(
            select(GeneratedResponseModel).filter(GeneratedResponseModel.query_id == query_id)
        )
        return result.scalar_one_or_none()

    async def get_responses_by_confidence(
        self,
        db: AsyncSession,
        min_confidence: float,
        limit: int = 10
    ) -> List[GeneratedResponseModel]:
        """
        Retrieve responses with confidence above a threshold.

        Args:
            db: Database session
            min_confidence: Minimum confidence score
            limit: Maximum number of responses to return

        Returns:
            List of GeneratedResponse instances with confidence above threshold
        """
        from sqlalchemy import desc
        result = await db.execute(
            select(GeneratedResponseModel)
            .filter(GeneratedResponseModel.confidence_score >= min_confidence)
            .order_by(desc(GeneratedResponseModel.confidence_score))
            .limit(limit)
        )
        return result.scalars().all()

    async def update_generated_response(
        self,
        db: AsyncSession,
        response_id: str,
        response_text: Optional[str] = None,
        referenced_content: Optional[dict] = None,
        confidence_score: Optional[float] = None,
        content_references: Optional[List[dict]] = None  # New content references to replace existing ones
    ) -> Optional[GeneratedResponseModel]:
        """
        Update a generated response.

        Args:
            db: Database session
            response_id: ID of the response to update
            response_text: New response text (optional)
            referenced_content: New referenced content (optional)
            confidence_score: New confidence score (optional)
            content_references: New content references to replace existing ones (optional)

        Returns:
            Updated GeneratedResponse instance if found, None otherwise
        """
        db_generated_response = await self.get_generated_response_by_id(db, response_id)
        if not db_generated_response:
            return None

        if response_text is not None:
            db_generated_response.response_text = response_text
        if referenced_content is not None:
            db_generated_response.referenced_content = str(referenced_content)
        if confidence_score is not None:
            db_generated_response.confidence_score = confidence_score

        # If new content references are provided, update them
        if content_references is not None:
            from backend.src.models.content_reference import content_reference_service
            # First, delete existing references
            existing_refs = await content_reference_service.get_references_by_response_id(db, response_id)
            for ref in existing_refs:
                await content_reference_service.delete_content_reference(db, str(ref.id))

            # Then, create new references
            for ref_data in content_references:
                await content_reference_service.create_content_reference(
                    db=db,
                    response_id=response_id,
                    content_id=ref_data.get('content_id'),
                    text_snippet=ref_data.get('text_snippet'),
                    page_number=ref_data.get('page_number'),
                    section_title=ref_data.get('section_title')
                )

        await db.commit()
        await db.refresh(db_generated_response)

        return db_generated_response

    async def delete_generated_response(
        self,
        db: AsyncSession,
        response_id: str
    ) -> bool:
        """
        Delete a generated response.

        Args:
            db: Database session
            response_id: ID of the response to delete

        Returns:
            True if deletion was successful, False otherwise
        """
        # First delete all content references associated with this response
        from backend.src.models.content_reference import content_reference_service
        existing_refs = await content_reference_service.get_references_by_response_id(db, response_id)
        for ref in existing_refs:
            await content_reference_service.delete_content_reference(db, str(ref.id))

        # Then delete the response
        db_generated_response = await self.get_generated_response_by_id(db, response_id)
        if not db_generated_response:
            return False

        await db.delete(db_generated_response)
        await db.commit()

        return True


# Global instance for the application
generated_response_service = GeneratedResponseService()