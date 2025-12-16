from fastapi import APIRouter, HTTPException, Depends, BackgroundTasks
from sqlalchemy.ext.asyncio import AsyncSession
from typing import List, Optional
from pydantic import BaseModel, field_validator
import uuid
import json
import logging

from backend.src.core.database import get_db
from backend.src.services.session_service import session_service
from backend.src.services.rag_service import rag_service
from backend.src.models.models import Session

# Configure logging
logger = logging.getLogger(__name__)

router = APIRouter()

# Pydantic models for request/response
class QueryRequest(BaseModel):
    query: str
    context: Optional[dict] = None  # Context for the query, including reading context

    @field_validator('query')
    @classmethod
    def validate_query(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('Query cannot be empty')
        if len(v) > 1000:
            raise ValueError('Query is too long (max 1000 characters)')
        return v.strip()

class SelectedTextQueryRequest(BaseModel):
    query: str
    selected_text: str
    context: Optional[dict] = None  # Context for the query, including reading context

    @field_validator('query')
    @classmethod
    def validate_query(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('Query cannot be empty')
        if len(v) > 1000:
            raise ValueError('Query is too long (max 1000 characters)')
        return v.strip()

    @field_validator('selected_text')
    @classmethod
    def validate_selected_text(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('Selected text cannot be empty')
        if len(v) > 5000:
            raise ValueError('Selected text is too long (max 5000 characters)')
        return v.strip()

class ContentReference(BaseModel):
    content_id: str
    text_snippet: str
    page_number: Optional[int] = None
    section_title: Optional[str] = None

class QueryResponse(BaseModel):
    response: str
    references: List[ContentReference]
    confidence: float
    conversation_id: str

import time
from fastapi import Request

@router.post("/book/{book_id}/query", response_model=QueryResponse)
async def query_book(
    request: Request,  # Add request to get client info
    book_id: str,
    query_request: QueryRequest,
    db: AsyncSession = Depends(get_db)
):
    """
    Query the book content using RAG.

    Submit a query about the book content and receive an AI-generated response based on the book content.
    """
    start_time = time.time()
    client_ip = request.client.host if request.client else "unknown"

    try:
        # Log the incoming request with comprehensive details
        logger.info(f"Query request received - Book: {book_id}, IP: {client_ip}, Query Length: {len(query_request.query)}")

        # Validate book_id format
        try:
            uuid.UUID(book_id)
        except ValueError:
            logger.warning(f"Invalid book_id format received: {book_id}")
            raise HTTPException(status_code=400, detail="Invalid book_id format")

        # Log more detailed query information
        logger.info(f"Processing query for book {book_id}: {query_request.query[:50]}...")

        # Create a new session for this interaction
        session = await session_service.create_session(
            db=db,
            book_id=book_id,
            session_metadata={"query_type": "general", "initiated_at": "now"}
        )

        # Extract reading context from the query_request
        current_page = None
        current_section = None
        if query_request.context:
            current_page = query_request.context.get('page_number')
            current_section = query_request.context.get('current_section')

        # Process the query using the RAG service with context information
        result = await rag_service.process_query(
            db=db,
            session_id=str(session.id),
            query_text=query_request.query,
            current_page=current_page,
            current_section=current_section
        )

        # Retrieve content references from the database
        from backend.src.models.generated_response import generated_response_service
        from backend.src.models.content_reference import content_reference_service
        from backend.src.models.models import GeneratedResponse as GeneratedResponseModel
        from sqlalchemy import select

        # Get the generated response and its content references
        gen_response = await generated_response_service.get_response_by_query_id(db, result['query_id'])
        if gen_response and hasattr(gen_response, 'content_references'):
            db_refs = gen_response.content_references
            formatted_references = [
                ContentReference(
                    content_id=str(ref.content_id),
                    text_snippet=ref.text_snippet or "",
                    page_number=ref.page_number,
                    section_title=ref.section_title
                )
                for ref in db_refs
            ]
        else:
            # Fallback to the original method if no database references found
            formatted_references = [
                ContentReference(
                    content_id=ref.get("id", "unknown"),
                    text_snippet=ref[:200] + "..." if len(ref) > 200 else ref,
                    page_number=None,  # Would be populated if available in metadata
                    section_title=None  # Would be populated if available in metadata
                )
                for ref in result.get("references", [])
            ]

        # Calculate processing time
        processing_time = time.time() - start_time

        logger.info(f"Query processed successfully for session {session.id}, Processing time: {processing_time:.2f}s")

        # Log response statistics
        logger.debug(f"Response generated - Length: {len(result.get('response', ''))}, References: {len(formatted_references)}")

        # Record successful query metrics
        from backend.src.services.metrics_service import metrics_service
        metrics_service.record_successful_query(processing_time)

        response_obj = QueryResponse(
            response=result.get("response", ""),
            references=formatted_references,
            confidence=result.get("confidence", 0.8),
            conversation_id=str(session.id)
        )

        logger.info(f"Response sent successfully, Total request time: {processing_time:.2f}s")
        return response_obj
    except HTTPException as http_ex:
        # Record failed query in metrics
        from backend.src.services.metrics_service import metrics_service
        metrics_service.record_failed_query()

        # Re-raise HTTP exceptions as-is
        logger.error(f"HTTP exception in query_book: {str(http_ex)}")
        raise
    except Exception as e:
        # Record failed query in metrics
        from backend.src.services.metrics_service import metrics_service
        metrics_service.record_failed_query()

        logger.error(f"Unexpected error processing book query: {str(e)}", exc_info=True)
        # Return a user-friendly error message
        raise HTTPException(
            status_code=500,
            detail="An unexpected error occurred while processing your query. Please try again later."
        )


@router.post("/book/{book_id}/query-selected", response_model=QueryResponse)
async def query_selected_text(
    book_id: str,
    request: SelectedTextQueryRequest,
    db: AsyncSession = Depends(get_db)
):
    """
    Query only the selected text in the book.

    Submit a query that should only consider the selected text in the book.
    """
    try:
        # Validate book_id format
        try:
            uuid.UUID(book_id)
        except ValueError:
            raise HTTPException(status_code=400, detail="Invalid book_id format")

        # Log the incoming query
        logger.info(f"Processing selected text query for book {book_id}")

        # Create a new session for this interaction
        session = await session_service.create_session(
            db=db,
            book_id=book_id,
            session_metadata={"query_type": "selected_text", "initiated_at": "now"}
        )

        # Extract reading context from the request
        current_page = None
        current_section = None
        if request.context:
            current_page = request.context.get('page_number')
            current_section = request.context.get('current_section')

        # Process the query using the RAG service with selected text and context information
        result = await rag_service.process_selected_text_query(
            db=db,
            session_id=str(session.id),
            query_text=request.query,
            selected_text=request.selected_text,
            current_page=current_page,
            current_section=current_section
        )

        # Retrieve content references from the database
        from backend.src.models.generated_response import generated_response_service
        from backend.src.models.content_reference import content_reference_service
        from backend.src.models.models import GeneratedResponse as GeneratedResponseModel
        from sqlalchemy import select

        # Get the generated response and its content references
        gen_response = await generated_response_service.get_response_by_query_id(db, result['query_id'])
        if gen_response and hasattr(gen_response, 'content_references'):
            db_refs = gen_response.content_references
            formatted_references = [
                ContentReference(
                    content_id=str(ref.content_id),
                    text_snippet=ref.text_snippet or "",
                    page_number=ref.page_number,
                    section_title=ref.section_title
                )
                for ref in db_refs
            ]
        else:
            # Fallback to the original method if no database references found
            formatted_references = [
                ContentReference(
                    content_id=ref.get("id", "unknown"),
                    text_snippet=ref[:200] + "..." if len(ref) > 200 else ref,
                    page_number=None,  # Would be populated if available in metadata
                    section_title=None  # Would be populated if available in metadata
                )
                for ref in result.get("references", [])
            ]

        logger.info(f"Selected text query processed successfully for session {session.id}")

        return QueryResponse(
            response=result.get("response", ""),
            references=formatted_references,
            confidence=result.get("confidence", 0.9),
            conversation_id=str(session.id)
        )
    except HTTPException as http_ex:
        # Re-raise HTTP exceptions as-is
        logger.error(f"HTTP exception in query_selected_text: {str(http_ex)}")
        raise
    except Exception as e:
        logger.error(f"Unexpected error processing selected text query: {str(e)}", exc_info=True)
        # Return a user-friendly error message
        raise HTTPException(
            status_code=500,
            detail="An unexpected error occurred while processing your selected text query. Please try again later."
        )


@router.get("/book/{book_id}/session/{session_id}", response_model=dict)
async def get_session_info(
    book_id: str,
    session_id: str,
    db: AsyncSession = Depends(get_db)
):
    """
    Retrieve information about a specific session.
    """
    try:
        # Validate UUID formats
        try:
            uuid.UUID(book_id)
            uuid.UUID(session_id)
        except ValueError:
            raise HTTPException(status_code=400, detail="Invalid book_id or session_id format")

        # Try to get the session
        session = await session_service.get_session_by_id(db, session_id)

        if not session:
            raise HTTPException(status_code=404, detail="Session not found or expired")

        # Verify the session belongs to the specified book
        if str(session.book_id) != book_id:
            raise HTTPException(status_code=404, detail="Session does not belong to the specified book")

        logger.info(f"Session info retrieved for session {session_id}")

        return {
            "session_id": str(session.id),
            "book_id": str(session.book_id),
            "created_at": session.created_at.isoformat(),
            "updated_at": session.updated_at.isoformat(),
            "expires_at": session.expires_at.isoformat(),
            "metadata": session.session_metadata
        }
    except HTTPException as http_ex:
        # Re-raise HTTP exceptions as-is
        logger.error(f"HTTP exception in get_session_info: {str(http_ex)}")
        raise
    except Exception as e:
        logger.error(f"Unexpected error retrieving session info: {str(e)}", exc_info=True)
        # Return a user-friendly error message
        raise HTTPException(
            status_code=500,
            detail="An unexpected error occurred while retrieving session information. Please try again later."
        )