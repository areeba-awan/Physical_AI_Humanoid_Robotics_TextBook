"""
Ingestion API Endpoints

Handles book content ingestion into the RAG system.
"""

from fastapi import APIRouter, HTTPException, status, Depends, UploadFile, File
from pydantic import BaseModel, Field
from typing import Optional, List
import os

from app.services.ingestion_service import IngestionService
from app.core.auth import get_current_user
from app.models import User


router = APIRouter()


class IngestTextRequest(BaseModel):
    """Request to ingest raw text content."""
    content: str = Field(..., min_length=10)
    chapter_id: str = Field(..., min_length=1)
    module_id: str = Field(..., min_length=1)
    section: str = Field(default="Content")


class IngestDirectoryRequest(BaseModel):
    """Request to ingest a directory of markdown files."""
    directory_path: str = Field(..., min_length=1)


class IngestResponse(BaseModel):
    """Response from ingestion operation."""
    success: bool
    chunks_created: int
    message: str
    details: Optional[dict] = None


class CollectionStatsResponse(BaseModel):
    """Response with collection statistics."""
    collection: str
    vectors_count: int
    points_count: int
    status: str


@router.post("", response_model=IngestResponse)
async def ingest_text(
    data: IngestTextRequest,
    user: User = Depends(get_current_user),
):
    """
    Ingest raw text content into the RAG system.

    This endpoint:
    1. Chunks the provided text
    2. Generates embeddings using text-embedding-3-large
    3. Stores vectors in Qdrant with metadata
    """
    try:
        service = IngestionService()
        result = await service.ingest_text(
            content=data.content,
            chapter_id=data.chapter_id,
            module_id=data.module_id,
            section=data.section,
        )

        return IngestResponse(
            success=result["status"] == "success",
            chunks_created=result.get("chunks", 0),
            message=f"Successfully ingested content into {data.chapter_id}",
            details=result,
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Ingestion failed: {str(e)}"
        )


@router.post("/directory", response_model=IngestResponse)
async def ingest_directory(
    data: IngestDirectoryRequest,
    user: User = Depends(get_current_user),
):
    """
    Ingest all markdown files from a directory.

    This endpoint:
    1. Scans the directory for .md files
    2. Extracts metadata from file paths
    3. Chunks, embeds, and stores all content
    """
    if not os.path.isdir(data.directory_path):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Directory not found: {data.directory_path}"
        )

    try:
        service = IngestionService()
        result = await service.ingest_directory(data.directory_path)

        return IngestResponse(
            success=True,
            chunks_created=result.get("total_chunks", 0),
            message=f"Processed {result.get('total_files', 0)} files",
            details=result,
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Directory ingestion failed: {str(e)}"
        )


@router.post("/file")
async def ingest_file(
    file: UploadFile = File(...),
    chapter_id: str = None,
    module_id: str = None,
    user: User = Depends(get_current_user),
):
    """
    Ingest a single markdown file upload.
    """
    if not file.filename.endswith('.md'):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Only markdown (.md) files are supported"
        )

    try:
        content = await file.read()
        content_str = content.decode('utf-8')

        # Use filename for metadata if not provided
        if not chapter_id:
            chapter_id = file.filename.replace('.md', '')
        if not module_id:
            module_id = "uploaded"

        service = IngestionService()
        result = await service.ingest_file(file.filename, content_str)

        return IngestResponse(
            success=result["status"] == "success",
            chunks_created=result.get("chunks", 0),
            message=f"Successfully ingested {file.filename}",
            details=result,
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"File ingestion failed: {str(e)}"
        )


@router.get("/stats", response_model=CollectionStatsResponse)
async def get_collection_stats(
    user: User = Depends(get_current_user),
):
    """
    Get statistics about the Qdrant collection.
    """
    try:
        service = IngestionService()
        stats = service.get_collection_stats()

        if "error" in stats:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=stats["error"]
            )

        return CollectionStatsResponse(
            collection=stats["collection"],
            vectors_count=stats.get("vectors_count", 0),
            points_count=stats.get("points_count", 0),
            status=stats.get("status", "unknown"),
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to get stats: {str(e)}"
        )


@router.delete("/collection")
async def delete_collection(
    user: User = Depends(get_current_user),
):
    """
    Delete the entire Qdrant collection.
    WARNING: This removes all ingested data!
    """
    try:
        service = IngestionService()
        success = await service.delete_collection()

        if success:
            return {"success": True, "message": "Collection deleted successfully"}
        else:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to delete collection"
            )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to delete collection: {str(e)}"
        )
