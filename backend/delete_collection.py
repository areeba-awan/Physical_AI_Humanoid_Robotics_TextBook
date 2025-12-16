import asyncio
import sys
from pathlib import Path

# Add the backend directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent))

from app.config import settings
from qdrant_client import QdrantClient


async def delete_collection():
    """Delete the existing Qdrant collection to start fresh."""
    qdrant = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key if settings.qdrant_api_key else None,
    )
    
    collection_name = settings.qdrant_collection
    print(f"Deleting collection: {collection_name}")
    
    try:
        qdrant.delete_collection(collection_name)
        print(f"Successfully deleted collection: {collection_name}")
        return True
    except Exception as e:
        print(f"Error deleting collection: {str(e)}")
        return False


if __name__ == "__main__":
    asyncio.run(delete_collection())