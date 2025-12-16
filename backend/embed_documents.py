import asyncio
from pathlib import Path
import sys
import os

# Add the backend directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent))

from app.services.ingestion_service import IngestionService
from app.services.rag_service import RAGService

async def main():
    """Main function to run the embedding process"""
    print("Starting the document embedding process...")

    # Get the ingestion service instance
    ingestion_service = IngestionService()

    # Get the RAG service instance
    rag_service = RAGService()

    # Specify the path to your book documents
    books_dir = Path("../book/docs")  # Looking for docs in the book directory relative to backend

    if not books_dir.exists():
        print(f"Book docs directory '{books_dir}' does not exist. Looking for markdown files in local 'docs' directory...")
        books_dir = Path("docs")  # Fallback to local docs directory

    if not books_dir.exists():
        print(f"Books/docs directory '{books_dir}' does not exist. Looking in 'book/docs' directory...")
        books_dir = Path("../../book/docs")  # Try higher level for proper path

    if not books_dir.exists():
        print(f"Books/docs directory '{books_dir}' does not exist. Creating 'docs' directory...")
        books_dir.mkdir(exist_ok=True)
        print(f"Please place your markdown files in the '{books_dir}' directory.")
        print("The system will automatically find all .md files in the directory structure.")
        return

    print(f"Looking for markdown files in '{books_dir}' directory...")

    # Process all markdown files in the directory
    try:
        result = await ingestion_service.ingest_directory(str(books_dir))
        print(f"Ingestion process completed: {result}")
    except Exception as e:
        print(f"Error during ingestion: {str(e)}")
        return

    # Display collection statistics
    print("\nCollection statistics:")
    stats = ingestion_service.get_collection_stats()
    print(f"  - Collection: {stats.get('collection', 'N/A')}")
    print(f"  - Vectors count: {stats.get('vectors_count', 'N/A')}")
    print(f"  - Points count: {stats.get('points_count', 'N/A')}")
    print(f"  - Status: {stats.get('status', 'N/A')}")

    # Show sample documents
    print("\nSample documents in collection:")
    samples = await ingestion_service.get_sample_documents(limit=3)
    for i, sample in enumerate(samples, 1):
        print(f"  {i}. Chapter: {sample['chapter_id']}")
        print(f"     Section: {sample['section']}")
        print(f"     Preview: {sample['content_preview'][:100]}...")
        print()

    print("\nEmbedding process completed!")

if __name__ == "__main__":
    asyncio.run(main())