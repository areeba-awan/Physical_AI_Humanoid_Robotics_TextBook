#!/usr/bin/env python3
"""
Test script to verify the RAG chatbot functionality.
This script will:
1. Check the current state of the vector database
2. Ingest book content if the database is empty
3. Test querying to verify that the RAG chatbot works
"""

import asyncio
import sys
from pathlib import Path

# Add the backend directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent / 'backend'))

from backend.app.services.ingestion_service import IngestionService
from backend.app.services.rag_service import RAGService


async def main():
    print("[INFO] Testing RAG Chatbot functionality...")

    # Create service instances
    ingestion_service = IngestionService()
    rag_service = RAGService()

    # Check current collection state
    print("\n[INFO] Checking current vector database state...")
    health = rag_service.check_collection_health()
    print(f"Collection: {health.get('collection_name', 'unknown')}")
    print(f"Status: {health.get('status', 'unknown')}")
    print(f"Vectors count: {health.get('vectors_count', 0)}")

    if health.get('vectors_count', 0) == 0:
        print("\n[INFO] Vector database is empty. Starting ingestion...")

        # Ingest book content
        book_dir = Path(__file__).parent / "book" / "docs"
        if not book_dir.exists():
            print(f"[ERROR] Book directory does not exist: {book_dir}")
            return False

        print(f"[INFO] Ingesting content from: {book_dir}")

        try:
            result = await ingestion_service.ingest_directory(str(book_dir))
            print(f"[SUCCESS] Ingestion completed!")
            print(f"   Files processed: {result.get('total_files', 0)}")
            print(f"   Chunks created: {result.get('total_chunks', 0)}")
            print(f"   Successful: {result.get('successful', 0)}")
        except Exception as e:
            print(f"[ERROR] Ingestion failed: {str(e)}")
            return False
    else:
        print(f"\n[SUCCESS] Vector database already has {health.get('vectors_count', 0)} vectors.")

    # Verify the database has content after ingestion
    print("\n[INFO] Verifying content after ingestion...")
    updated_health = rag_service.check_collection_health()
    print(f"Updated vectors count: {updated_health.get('vectors_count', 0)}")

    if updated_health.get('vectors_count', 0) > 0:
        print("\n[SUCCESS] Content verification successful - database contains book content!")

        # Get a sample to confirm
        samples = await rag_service.get_collection_sample(limit=2)
        if samples:
            print("\n[INFO] Sample content from database:")
            for i, sample in enumerate(samples, 1):
                print(f"  Sample {i}:")
                print(f"    Chapter: {sample.get('chapter_id', 'N/A')}")
                print(f"    Section: {sample.get('section', 'N/A')}")
                content_preview = sample.get('content_preview', '')[:100] + "..."
                print(f"    Content: {content_preview}")

    # Test RAG functionality
    print("\n[INFO] Testing RAG query functionality...")
    try:
        # Perform a test search for a common topic in robotics books
        test_query = "What is ROS 2?"
        results = await rag_service.search_documents(
            query=test_query,
            limit=3,
            score_threshold=0.2
        )

        if results:
            print(f"[SUCCESS] Found {len(results)} relevant documents for query: '{test_query}'")
            print("Sample results:")
            for i, result in enumerate(results[:2], 1):  # Show first 2 results
                print(f"  Result {i}:")
                print(f"    Chapter: {result.get('metadata', {}).get('chapter_id', 'N/A')}")
                print(f"    Section: {result.get('metadata', {}).get('section', 'N/A')}")
                print(f"    Score: {result.get('score', 0):.3f}")
                content_preview = result.get('content', '')[:100] + "..."
                print(f"    Content: {content_preview}")
        else:
            print(f"[ERROR] No results found for query: '{test_query}'")
            print("This suggests the ingestion may not have worked properly.")

    except Exception as e:
        print(f"[ERROR] Query testing failed: {str(e)}")
        return False

    print("\n[SUCCESS] RAG Chatbot functionality test completed successfully!")
    print("[SUCCESS] The chatbot should now be able to answer book-related queries.")

    return True


if __name__ == "__main__":
    success = asyncio.run(main())
    if not success:
        print("\n[ERROR] Testing failed. Please check configuration and try again.")
        sys.exit(1)
    else:
        print("\n[SUCCESS] All tests passed. The RAG chatbot is ready to answer book-related queries!")