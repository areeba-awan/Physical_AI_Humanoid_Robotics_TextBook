#!/usr/bin/env python3
"""
Quick test script to check Qdrant status and fix the RAG chatbot
"""

import os
import sys
from pathlib import Path

# Add the backend directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent / 'backend'))

from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load environment variables
load_dotenv(dotenv_path=Path(__file__).parent / 'backend' / '.env')

from backend.src.services.embedding_service import qwen_embedding_service
from backend.app.services.ingestion_service import IngestionService


async def quick_fix():
    print("[INFO] Starting RAG chatbot fix...")
    
    # Connect to Qdrant using the environment settings
    qdrant_url = os.getenv('QDRANT_URL', 'http://localhost:6333')
    qdrant_api_key = os.getenv('QDRANT_API_KEY')
    
    print(f"[INFO] Using Qdrant URL: {qdrant_url}")
    
    # Initialize Qdrant client
    if qdrant_api_key:
        qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
    else:
        qdrant_client = QdrantClient(url=qdrant_url)
    
    # Check if the collection exists, create if it doesn't
    collection_name = os.getenv('QDRANT_COLLECTION', 'physicalai_textbook')
    
    try:
        collections = qdrant_client.get_collections()
        collection_names = [c.name for c in collections.collections]
        print(f"[INFO] Existing collections: {collection_names}")
        
        if collection_name not in collection_names:
            print(f"[INFO] Collection '{collection_name}' not found. Creating...")
            
            # Use dimensions from environment or default
            embedding_size = int(os.getenv('QWEN_EMBEDDING_DIMENSIONS', '1536'))  # Default to common size
            
            # This might be a different API call depending on the client version
            from qdrant_client.http import models
            from qdrant_client.http.models import Distance, VectorParams
            
            qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(
                    size=embedding_size,
                    distance=Distance.COSINE
                )
            )
            print(f"[SUCCESS] Created collection: {collection_name}")
        else:
            print(f"[SUCCESS] Collection '{collection_name}' already exists")
        
        # Check collection status
        collection_info = qdrant_client.get_collection(collection_name)
        print(f"[INFO] Collection status: {collection_info.status}")
        print(f"[INFO] Points count: {collection_info.points_count}")
        
        # Test embedding service
        print(f"[INFO] Testing embedding service...")
        try:
            test_embedding = await qwen_embedding_service.generate_embeddings("test query")
            print(f"[SUCCESS] Embedding service working. Embedding dimension: {len(test_embedding)}")
        except Exception as e:
            print(f"[ERROR] Embedding service issue: {e}")
            return False

        # Now that we know Qdrant is available, run ingestion
        print(f"[INFO] Starting ingestion...")
        ingestion_service = IngestionService()
        
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
            import traceback
            traceback.print_exc()
            return False
            
        # Final check
        updated_collection_info = qdrant_client.get_collection(collection_name)
        print(f"[INFO] Updated points count: {updated_collection_info.points_count}")
        
        if updated_collection_info.points_count > 0:
            print(f"[SUCCESS] RAG chatbot is now ready with {updated_collection_info.points_count} vectors!")
            print("[SUCCESS] The chatbot should now respond to book-related queries.")
            return True
        else:
            print("[ERROR] No vectors were ingested. Chatbot will not work.")
            return False
            
    except Exception as e:
        print(f"[ERROR] Failed to interact with Qdrant: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    import asyncio
    success = asyncio.run(quick_fix())
    if success:
        print("\n[SUCCESS] RAG chatbot fix completed successfully!")
        print("The chatbot should now respond to book-related queries.")
    else:
        print("\n[ERROR] RAG chatbot fix failed. Please check configuration.")