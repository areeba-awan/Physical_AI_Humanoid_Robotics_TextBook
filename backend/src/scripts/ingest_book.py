#!/usr/bin/env python3
"""
CLI script for ingesting books into the RAG system.

This script handles the complete flow of processing a book file:
1. Parse the book content (PDF, EPUB, etc.)
2. Chunk the content with appropriate overlap
3. Generate embeddings using Qwen
4. Store chunks in Qdrant vector database
5. Store metadata in Postgres
"""

import argparse
import asyncio
import sys
import uuid
from pathlib import Path

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent / 'backend' / 'src'))

from backend.src.services.book_ingestion import book_ingestion_service


async def main():
    parser = argparse.ArgumentParser(
        description="Ingest a book into the RAG chatbot system"
    )
    parser.add_argument(
        "--book-path", 
        type=str, 
        required=True,
        help="Path to the book file to ingest (PDF, EPUB, TXT, etc.)"
    )
    parser.add_argument(
        "--book-id", 
        type=str, 
        required=False,
        help="Unique identifier for the book (will be generated if not provided)"
    )
    parser.add_argument(
        "--chunk-size", 
        type=int, 
        default=1000,
        help="Maximum size of text chunks (default: 1000 characters)"
    )
    parser.add_argument(
        "--overlap", 
        type=int, 
        default=100,
        help="Overlap between chunks in characters (default: 100)"
    )
    parser.add_argument(
        "--verbose", 
        action="store_true", 
        help="Enable verbose output"
    )

    args = parser.parse_args()

    # Validate book file exists
    book_path = Path(args.book_path)
    if not book_path.exists():
        print(f"Error: Book file does not exist: {args.book_path}", file=sys.stderr)
        sys.exit(1)

    # Generate book ID if not provided
    book_id = args.book_id or str(uuid.uuid4())

    if args.verbose:
        print(f"Ingesting book: {book_path.name}")
        print(f"Book ID: {book_id}")
        print(f"Chunk size: {args.chunk_size}")
        print(f"Overlap: {args.overlap}")

    try:
        # Perform ingestion
        result = await book_ingestion_service.ingest_book(
            book_path=str(book_path),
            book_id=book_id
        )

        if result.get("status") == "success":
            print(f"✅ Successfully ingested book '{book_path.name}'")
            print(f"📊 Chunks processed: {result.get('chunks_processed', 0)}")
            print(f"📚 Book ID: {book_id}")
        else:
            print(f"❌ Failed to ingest book: {result.get('error', 'Unknown error')}")
            sys.exit(1)

    except Exception as e:
        print(f"❌ Error during ingestion: {str(e)}", file=sys.stderr)
        if args.verbose:
            import traceback
            traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())