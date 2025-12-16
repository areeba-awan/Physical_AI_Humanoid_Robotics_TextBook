"""
Ingestion Service for RAG Pipeline

Handles:
- Document loading from markdown files
- Smart text chunking with overlap
- Embedding generation using Qwen embeddings
- Vector storage in Qdrant with rich metadata
- Batch processing for efficiency
"""

import os
import re
import hashlib
import logging
from typing import List, Dict, Any, Optional
from pathlib import Path
from dataclasses import dataclass
from uuid import uuid4

from qdrant_client import QdrantClient
from qdrant_client.http.models import (
    VectorParams,
    Distance,
    PointStruct,
    CollectionStatus,
    OptimizersConfigDiff,
)

from app.config import settings
import sys
from pathlib import Path
# Add the backend directory to the Python path
backend_dir = Path(__file__).resolve().parent.parent.parent  # Go up to backend
sys.path.insert(0, str(backend_dir))

from src.services.embedding_service import qwen_embedding_service

logger = logging.getLogger(__name__)


@dataclass
class DocumentChunk:
    """Represents a chunk of document with metadata."""
    id: str
    content: str
    chapter_id: str
    module_id: str
    section: str
    page_number: int
    chunk_index: int
    source_file: str
    word_count: int


class IngestionService:
    """Service for ingesting book content into the RAG system."""

    # Chunking parameters - optimized for RAG retrieval
    CHUNK_SIZE_CHARS = 1500  # ~375 tokens, good for context
    CHUNK_OVERLAP_CHARS = 200  # ~50 tokens overlap
    MIN_CHUNK_SIZE = 100  # Don't create tiny chunks

    # Batch processing
    EMBEDDING_BATCH_SIZE = 50  # Process embeddings in batches
    UPSERT_BATCH_SIZE = 100  # Upsert to Qdrant in batches

    def __init__(self):
        self.qdrant = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key if settings.qdrant_api_key else None,
        )
        self.collection_name = settings.qdrant_collection

    async def ensure_collection_exists(self) -> bool:
        """Create Qdrant collection if it doesn't exist."""
        try:
            collections = self.qdrant.get_collections()
            collection_names = [c.name for c in collections.collections]

            if self.collection_name not in collection_names:
                # Use Qwen embedding dimensions if available, otherwise fallback to OpenAI dimensions
                embedding_size = settings.qwen_embedding_dimensions if settings.qwen_embedding_dimensions else settings.openai_embedding_dimensions
                self.qdrant.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=embedding_size,  # Use Qwen embedding dimensions
                        distance=Distance.COSINE,
                    ),
                    optimizers_config=OptimizersConfigDiff(
                        indexing_threshold=10000,  # Start indexing after 10k vectors
                    ),
                )
                logger.info(f"Created collection: {self.collection_name}")
                return True

            logger.info(f"Collection {self.collection_name} already exists")
            return True
        except Exception as e:
            logger.error(f"Error ensuring collection exists: {e}")
            return False

    def extract_metadata_from_path(self, file_path: str) -> Dict[str, str]:
        """Extract chapter and module info from file path."""
        path = Path(file_path)
        filename = path.stem
        parent = path.parent.name

        # Try multiple patterns
        module_match = re.search(r'module[-_]?(\d+)', parent, re.IGNORECASE)
        chapter_match = re.search(r'chapter[-_]?(\d+)[-_]?(\d+)?', filename, re.IGNORECASE)

        if module_match:
            module_id = f"module-{module_match.group(1)}"
        else:
            module_id = "module-general"

        if chapter_match:
            if chapter_match.group(2):
                chapter_id = f"chapter-{chapter_match.group(1)}-{chapter_match.group(2)}"
            else:
                chapter_id = f"chapter-{chapter_match.group(1)}"
        else:
            # Use filename as chapter ID
            chapter_id = filename.lower().replace(' ', '-')

        return {
            "module_id": module_id,
            "chapter_id": chapter_id,
        }

    def extract_sections(self, content: str) -> List[Dict[str, Any]]:
        """
        Extract sections from markdown content.

        Handles:
        - H1, H2, H3 headers
        - Code blocks (preserves them in sections)
        - Frontmatter (YAML)
        """
        sections = []

        # Remove YAML frontmatter
        frontmatter_pattern = r'^---\s*\n.*?\n---\s*\n'
        content = re.sub(frontmatter_pattern, '', content, flags=re.DOTALL)

        # Split by headers (##, ###, ####)
        header_pattern = r'^(#{1,4})\s+(.+)$'
        lines = content.split('\n')

        current_section = "Introduction"
        current_content = []
        current_level = 0

        for line in lines:
            header_match = re.match(header_pattern, line)
            if header_match:
                # Save previous section if it has content
                if current_content:
                    section_text = '\n'.join(current_content).strip()
                    if section_text and len(section_text) > self.MIN_CHUNK_SIZE:
                        sections.append({
                            "section": current_section,
                            "content": section_text,
                            "level": current_level,
                        })

                current_level = len(header_match.group(1))
                current_section = header_match.group(2).strip()
                current_content = []
            else:
                current_content.append(line)

        # Save last section
        if current_content:
            section_text = '\n'.join(current_content).strip()
            if section_text and len(section_text) > self.MIN_CHUNK_SIZE:
                sections.append({
                    "section": current_section,
                    "content": section_text,
                    "level": current_level,
                })

        return sections

    def smart_chunk_text(
        self,
        text: str,
        chunk_size: int = None,
        overlap: int = None
    ) -> List[str]:
        """
        Split text into overlapping chunks intelligently.

        Tries to break at:
        1. Paragraph boundaries
        2. Sentence boundaries
        3. Word boundaries (fallback)

        Preserves code blocks intact when possible.
        """
        chunk_size = chunk_size or self.CHUNK_SIZE_CHARS
        overlap = overlap or self.CHUNK_OVERLAP_CHARS

        if len(text) <= chunk_size:
            return [text] if text.strip() else []

        chunks = []
        start = 0

        # Identify code blocks to preserve them
        code_block_pattern = r'```[\s\S]*?```'
        code_blocks = [(m.start(), m.end()) for m in re.finditer(code_block_pattern, text)]

        def is_in_code_block(pos: int) -> bool:
            return any(start <= pos <= end for start, end in code_blocks)

        while start < len(text):
            end = min(start + chunk_size, len(text))

            if end < len(text):
                # Try to find a good break point
                best_break = end

                # Look for paragraph break first
                para_break = text.rfind('\n\n', start + chunk_size // 2, end)
                if para_break != -1 and not is_in_code_block(para_break):
                    best_break = para_break + 2
                else:
                    # Try sentence boundaries
                    for delimiter in ['. ', '.\n', '? ', '!\n']:
                        sent_break = text.rfind(delimiter, start + chunk_size // 2, end)
                        if sent_break != -1 and not is_in_code_block(sent_break):
                            best_break = sent_break + len(delimiter)
                            break
                    else:
                        # Fall back to word boundary
                        word_break = text.rfind(' ', start + chunk_size // 2, end)
                        if word_break != -1:
                            best_break = word_break + 1

                end = best_break

            chunk = text[start:end].strip()
            if chunk and len(chunk) >= self.MIN_CHUNK_SIZE:
                chunks.append(chunk)

            # Move start with overlap
            start = max(start + 1, end - overlap)

            # Avoid infinite loop
            if start >= len(text) - overlap:
                break

        return chunks

    def create_chunks_from_content(
        self,
        content: str,
        metadata: Dict[str, str],
        source_file: str,
    ) -> List[DocumentChunk]:
        """Create document chunks with metadata."""
        chunks = []
        sections = self.extract_sections(content)

        page_number = 1
        chunk_index = 0

        for section_data in sections:
            section_name = section_data["section"]
            section_content = section_data["content"]

            if not section_content.strip():
                continue

            text_chunks = self.smart_chunk_text(section_content)

            for text in text_chunks:
                # Create deterministic ID based on content
                chunk_id = hashlib.md5(
                    f"{metadata['chapter_id']}:{section_name}:{chunk_index}:{text[:100]}".encode()
                ).hexdigest()

                word_count = len(text.split())

                chunks.append(DocumentChunk(
                    id=chunk_id,
                    content=text,
                    chapter_id=metadata["chapter_id"],
                    module_id=metadata["module_id"],
                    section=section_name,
                    page_number=page_number,
                    chunk_index=chunk_index,
                    source_file=source_file,
                    word_count=word_count,
                ))

                chunk_index += 1
                # Estimate page numbers based on content
                page_number += len(text) // 3000

        logger.info(f"Created {len(chunks)} chunks from {source_file}")
        return chunks

    async def generate_embeddings(
        self,
        texts: List[str],
        batch_size: int = None
    ) -> List[List[float]]:
        """
        Generate embeddings for multiple texts with batching using Qwen service.
        """
        batch_size = batch_size or self.EMBEDDING_BATCH_SIZE
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            try:
                # Use Qwen embedding service to generate embeddings
                batch_embeddings = await qwen_embedding_service.generate_embeddings(batch)
                all_embeddings.extend(batch_embeddings)
                logger.debug(f"Generated embeddings for batch {i // batch_size + 1}")
            except Exception as e:
                logger.error(f"Embedding generation failed for batch {i}: {e}")
                raise

        return all_embeddings

    async def ingest_file(self, file_path: str, content: str) -> Dict[str, Any]:
        """Ingest a single file into Qdrant."""
        metadata = self.extract_metadata_from_path(file_path)
        chunks = self.create_chunks_from_content(content, metadata, file_path)

        if not chunks:
            return {
                "file": file_path,
                "chunks": 0,
                "status": "skipped",
                "message": "No content to ingest"
            }

        # Generate embeddings
        texts = [chunk.content for chunk in chunks]
        embeddings = await self.generate_embeddings(texts)

        # Create points for Qdrant with rich metadata
        points = []
        for chunk, embedding in zip(chunks, embeddings):
            points.append(PointStruct(
                id=chunk.id,
                vector=embedding,
                payload={
                    "content": chunk.content,
                    "chapter_id": chunk.chapter_id,
                    "module_id": chunk.module_id,
                    "section": chunk.section,
                    "page_number": chunk.page_number,
                    "chunk_index": chunk.chunk_index,
                    "source_file": chunk.source_file,
                    "word_count": chunk.word_count,
                }
            ))

        # Upsert to Qdrant in batches
        for i in range(0, len(points), self.UPSERT_BATCH_SIZE):
            batch = points[i:i + self.UPSERT_BATCH_SIZE]
            self.qdrant.upsert(
                collection_name=self.collection_name,
                points=batch,
            )

        logger.info(f"Ingested {len(chunks)} chunks from {file_path}")

        return {
            "file": file_path,
            "chunks": len(chunks),
            "status": "success",
            "chapter_id": metadata["chapter_id"],
            "module_id": metadata["module_id"],
        }

    async def ingest_directory(self, directory_path: str) -> Dict[str, Any]:
        """Ingest all markdown files from a directory."""
        await self.ensure_collection_exists()

        results = []
        total_chunks = 0
        errors = []

        path = Path(directory_path)
        md_files = list(path.rglob("*.md"))

        logger.info(f"Found {len(md_files)} markdown files in {directory_path}")

        for file_path in md_files:
            try:
                with open(file_path, "r", encoding="utf-8") as f:
                    content = f.read()

                result = await self.ingest_file(str(file_path), content)
                results.append(result)
                total_chunks += result.get("chunks", 0)

            except Exception as e:
                error_msg = f"Error processing {file_path}: {str(e)}"
                logger.error(error_msg)
                errors.append(error_msg)
                results.append({
                    "file": str(file_path),
                    "status": "error",
                    "message": str(e),
                })

        return {
            "total_files": len(md_files),
            "total_chunks": total_chunks,
            "successful": sum(1 for r in results if r.get("status") == "success"),
            "errors": errors,
            "results": results,
        }

    async def ingest_text(
        self,
        content: str,
        chapter_id: str,
        module_id: str,
        section: str = "Content",
    ) -> Dict[str, Any]:
        """Ingest raw text content."""
        await self.ensure_collection_exists()

        metadata = {
            "chapter_id": chapter_id,
            "module_id": module_id,
        }

        chunks = []
        text_chunks = self.smart_chunk_text(content)

        for i, text in enumerate(text_chunks):
            chunk_id = hashlib.md5(
                f"{chapter_id}:{section}:{i}:{text[:100]}".encode()
            ).hexdigest()

            word_count = len(text.split())

            chunks.append(DocumentChunk(
                id=chunk_id,
                content=text,
                chapter_id=chapter_id,
                module_id=module_id,
                section=section,
                page_number=1 + (i // 3),
                chunk_index=i,
                source_file="direct_input",
                word_count=word_count,
            ))

        if not chunks:
            return {"chunks": 0, "status": "skipped", "message": "No content to ingest"}

        texts = [chunk.content for chunk in chunks]
        embeddings = await self.generate_embeddings(texts)

        points = []
        for chunk, embedding in zip(chunks, embeddings):
            points.append(PointStruct(
                id=chunk.id,
                vector=embedding,
                payload={
                    "content": chunk.content,
                    "chapter_id": chunk.chapter_id,
                    "module_id": chunk.module_id,
                    "section": chunk.section,
                    "page_number": chunk.page_number,
                    "chunk_index": chunk.chunk_index,
                    "source_file": chunk.source_file,
                    "word_count": chunk.word_count,
                }
            ))

        self.qdrant.upsert(
            collection_name=self.collection_name,
            points=points,
        )

        logger.info(f"Ingested {len(chunks)} chunks for {chapter_id}")

        return {
            "chunks": len(chunks),
            "status": "success",
            "chapter_id": chapter_id,
            "module_id": module_id,
        }

    def get_collection_stats(self) -> Dict[str, Any]:
        """Get statistics about the Qdrant collection."""
        try:
            collection_info = self.qdrant.get_collection(self.collection_name)
            # Use Qwen embedding dimensions if available, otherwise fallback to OpenAI dimensions
            vector_size = settings.qwen_embedding_dimensions if settings.qwen_embedding_dimensions else settings.openai_embedding_dimensions
            return {
                "collection": self.collection_name,
                "vectors_count": collection_info.vectors_count,
                "points_count": collection_info.points_count,
                "status": collection_info.status.value,
                "vector_size": vector_size,
            }
        except Exception as e:
            logger.error(f"Failed to get collection stats: {e}")
            return {
                "collection": self.collection_name,
                "error": str(e),
            }

    async def delete_collection(self) -> bool:
        """Delete the entire collection (use with caution)."""
        try:
            self.qdrant.delete_collection(self.collection_name)
            logger.warning(f"Deleted collection: {self.collection_name}")
            return True
        except Exception as e:
            logger.error(f"Failed to delete collection: {e}")
            return False

    async def get_sample_documents(self, limit: int = 5) -> List[Dict[str, Any]]:
        """Get sample documents for debugging."""
        try:
            results = self.qdrant.scroll(
                collection_name=self.collection_name,
                limit=limit,
                with_payload=True,
                with_vectors=False,
            )

            samples = []
            for point in results[0]:
                samples.append({
                    "id": point.id,
                    "chapter_id": point.payload.get("chapter_id", ""),
                    "section": point.payload.get("section", ""),
                    "word_count": point.payload.get("word_count", 0),
                    "content_preview": point.payload.get("content", "")[:200],
                })
            return samples
        except Exception as e:
            logger.error(f"Failed to get sample documents: {e}")
            return []
