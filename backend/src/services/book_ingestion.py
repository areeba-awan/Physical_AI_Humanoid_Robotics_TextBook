import asyncio
import os
from typing import List, Dict, Any, Optional
from pathlib import Path
import PyPDF2
from pypdf import PdfReader
import json
from datetime import datetime
import uuid
from backend.src.services.embedding_service import qwen_embedding_service
from backend.src.core.vector_db import vector_db
from backend.src.core.database import get_db
from backend.src.models.models import BookContent
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.orm import sessionmaker
from sqlalchemy import select
import logging

logger = logging.getLogger(__name__)

class BookIngestionService:
    def __init__(self):
        self.max_chunk_size = int(os.getenv("MAX_CHUNK_SIZE", "1000"))
        self.chunk_overlap = int(os.getenv("CHUNK_OVERLAP", "100"))  # characters to overlap between chunks
    
    async def ingest_book(self, book_path: str, book_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Main method to ingest a book: parse, chunk, embed, and store.
        
        Args:
            book_path: Path to the book file (PDF, text, etc.)
            book_id: Optional book ID (will be generated if not provided)
            
        Returns:
            Dict with ingestion results
        """
        try:
            if not book_id:
                book_id = str(uuid.uuid4())
            
            logger.info(f"Starting ingestion for book {book_id} from {book_path}")
            
            # 1. Parse the book content
            book_content = await self.parse_book(book_path)
            
            # 2. Chunk the content
            chunks = self.chunk_content(book_content, book_id)
            
            # 3. Generate embeddings
            logger.info(f"Generating embeddings for {len(chunks)} chunks...")
            embeddings = await qwen_embedding_service.generate_embeddings([chunk['text'] for chunk in chunks])
            
            # 4. Store in vector database and metadata in Postgres
            await self.store_chunks(chunks, embeddings, book_id)
            
            logger.info(f"Successfully ingested book {book_id} with {len(chunks)} chunks")
            
            return {
                "book_id": book_id,
                "chunks_processed": len(chunks),
                "status": "success"
            }
        except Exception as e:
            logger.error(f"Error ingesting book {book_id}: {str(e)}")
            logger.error(f"Traceback: {traceback.format_exc()}")
            return {
                "book_id": book_id or "unknown",
                "status": "error",
                "error": str(e)
            }
    
    async def parse_book(self, book_path: str) -> str:
        """
        Parse book content from various formats (PDF, text, etc.).
        
        Args:
            book_path: Path to the book file
            
        Returns:
            The full text content of the book
        """
        path = Path(book_path)
        
        if not path.exists():
            raise FileNotFoundError(f"Book file not found: {book_path}")
        
        if path.suffix.lower() == '.pdf':
            return await self.parse_pdf(book_path)
        elif path.suffix.lower() in ['.txt', '.text']:
            return await self.parse_text_file(book_path)
        else:
            # Try to read as text file regardless of extension
            return await self.parse_text_file(book_path)
    
    async def parse_pdf(self, pdf_path: str) -> str:
        """
        Parse content from a PDF file.
        
        Args:
            pdf_path: Path to the PDF file
            
        Returns:
            The text content of the PDF
        """
        try:
            with open(pdf_path, 'rb') as file:
                pdf_reader = PdfReader(file)
                text = ""
                
                for page_num, page in enumerate(pdf_reader.pages):
                    try:
                        page_text = page.extract_text()
                        text += f"\n\n--- PAGE {page_num + 1} ---\n\n{page_text}"
                    except Exception as e:
                        logger.warning(f"Could not extract text from page {page_num + 1}: {str(e)}")
                        continue
                
                return text
        except Exception as e:
            logger.error(f"Error parsing PDF {pdf_path}: {str(e)}")
            raise e
    
    async def parse_text_file(self, text_path: str) -> str:
        """
        Parse content from a text file.
        
        Args:
            text_path: Path to the text file
            
        Returns:
            The text content of the file
        """
        try:
            with open(text_path, 'r', encoding='utf-8') as file:
                return file.read()
        except Exception as e:
            logger.error(f"Error parsing text file {text_path}: {str(e)}")
            raise e
    
    def chunk_content(self, content: str, book_id: str) -> List[Dict[str, Any]]:
        """
        Chunk the book content into semantic chunks with overlap.
        
        Args:
            content: The full text content of the book
            book_id: The book ID to associate with chunks
            
        Returns:
            List of chunk dictionaries with text and metadata
        """
        chunks = []
        
        # Split content into sentences to maintain semantic boundaries
        sentences = content.split('. ')
        
        current_chunk = ""
        current_size = 0
        chunk_id = 1
        
        for sentence in sentences:
            # Add a period back if it was removed during splitting
            sentence_with_period = sentence + "."
            
            # If adding this sentence would exceed the max chunk size
            if current_size + len(sentence_with_period) > self.max_chunk_size:
                # Save the current chunk
                if current_chunk.strip():
                    chunks.append({
                        "id": f"{book_id}-chunk-{chunk_id}",
                        "text": current_chunk.strip(),
                        "metadata": {
                            "book_id": book_id,
                            "chunk_id": chunk_id,
                            "created_at": datetime.utcnow().isoformat()
                        }
                    })
                    chunk_id += 1
                
                # Start a new chunk, possibly with overlap from the previous chunk
                if len(sentence_with_period) > self.max_chunk_size:
                    # If the sentence is too long, split it by maximum chunk size
                    for i in range(0, len(sentence_with_period), self.max_chunk_size):
                        sub_sentence = sentence_with_period[i:i + self.max_chunk_size]
                        chunks.append({
                            "id": f"{book_id}-chunk-{chunk_id}",
                            "text": sub_sentence.strip(),
                            "metadata": {
                                "book_id": book_id,
                                "chunk_id": chunk_id,
                                "created_at": datetime.utcnow().isoformat()
                            }
                        })
                        chunk_id += 1
                    current_chunk = ""
                    current_size = 0
                else:
                    # Use overlap strategy - include end of previous chunk in new chunk
                    overlap = min(self.chunk_overlap, current_size)
                    current_chunk = current_chunk[-overlap:] + " " + sentence_with_period
                    current_size = len(current_chunk)
            else:
                # Add sentence to current chunk
                current_chunk += " " + sentence_with_period
                current_size += len(sentence_with_period)
        
        # Add the last chunk if it has content
        if current_chunk.strip():
            chunks.append({
                "id": f"{book_id}-chunk-{chunk_id}",
                "text": current_chunk.strip(),
                "metadata": {
                    "book_id": book_id,
                    "chunk_id": chunk_id,
                    "created_at": datetime.utcnow().isoformat()
                }
            })
        
        return chunks
    
    async def store_chunks(self, chunks: List[Dict[str, Any]], embeddings: List[List[float]], book_id: str):
        """
        Store chunk text in Postgres and embeddings in Qdrant.
        
        Args:
            chunks: List of chunk dictionaries
            embeddings: List of embedding vectors corresponding to chunks
            book_id: The book ID these chunks belong to
        """
        # Store book metadata in Postgres first
        db_gen = get_db()
        db: AsyncSession = await db_gen.__anext__()

        try:
            # Store book metadata record
            from ..models.models import BookContent
            # Also track book metadata separately if needed
            # For now, we'll focus on storing the content chunks

            # Store text chunks in Postgres
            for i, chunk in enumerate(chunks):
                # Validation: check if chunk text is valid
                if not chunk["text"] or not chunk["text"].strip():
                    logger.warning(f"Skipping empty chunk {chunk['id']}")
                    continue

                # Validate chunk text length
                if len(chunk["text"]) > 2000:  # From validation rules
                    logger.warning(f"Truncating long chunk {chunk['id']} from {len(chunk['text'])} to 2000 characters")
                    chunk["text"] = chunk["text"][:2000]

                # Create BookContent model instance
                book_content = BookContent(
                    book_id=uuid.UUID(book_id),
                    chunk_text=chunk["text"],
                    chunk_metadata=json.dumps(chunk["metadata"]),
                    created_at=datetime.utcnow(),
                    updated_at=datetime.utcnow()
                )

                # Add to database session
                db.add(book_content)

            # Commit Postgres changes
            await db.commit()

            # Store embeddings in Qdrant with the same IDs
            # Only for chunks that were actually stored (passed validation)
            valid_chunks = [chunk for chunk in chunks if chunk["text"].strip()]
            qdrant_ids = [chunk["id"] for chunk in valid_chunks]
            payloads = [
                {
                    "book_id": chunk["metadata"]["book_id"],
                    "chunk_id": chunk["metadata"]["chunk_id"],
                    "text_preview": chunk["text"][:200] + "..." if len(chunk["text"]) > 200 else chunk["text"],
                    "created_at": chunk["metadata"]["created_at"]
                }
                for chunk in valid_chunks
            ]

            if qdrant_ids:  # Only store if we have valid chunks
                vector_db.add_vectors(
                    vectors=embeddings,
                    payloads=payloads,
                    ids=qdrant_ids
                )

                logger.info(f"Stored {len(valid_chunks)} chunks in database and vector store")
            else:
                logger.warning(f"No valid chunks to store for book {book_id}")

        except Exception as e:
            await db.rollback()
            logger.error(f"Error storing chunks for book {book_id}: {str(e)}")
            raise e
        finally:
            await db.close()


# Global instance for the application
book_ingestion_service = BookIngestionService()