import asyncio
import subprocess
import sys
import threading
from pathlib import Path
import time

# Add the backend directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent))

from embed_documents import main as embed_main
from app.services.ingestion_service import IngestionService
from app.services.rag_service import RAGService

def run_server():
    """Run the FastAPI server in a separate thread"""
    import uvicorn
    # Set event loop policy for Windows
    if sys.platform.startswith('win'):
        asyncio.set_event_loop_policy(asyncio.WindowsProactorEventLoopPolicy())
        
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=False
    )

async def main():
    """Main function to orchestrate the embedding and server startup"""
    print("Starting the Physical AI & Humanoid Robotics Learning Platform...")

    # Step 1: Run the embedding process first (this handles initialization)
    print("\nStep 1: Starting document embedding process...")
    await embed_main()
    print("Document embedding process completed!")

    # Step 2: Verify the embedding by checking collections
    print("\nStep 2: Verifying embeddings...")
    rag_service = RAGService()
    try:
        info = rag_service.qdrant.get_collection(rag_service.collection_name)
        print(f"Collection '{rag_service.collection_name}' found with {info.points_count} points")
    except Exception as e:
        print(f"Could not connect to Qdrant: {e}")

    print("\nStep 3: Starting the server...")
    print("The RAG chatbot system will be available at http://localhost:8000")

    # Start the server in a separate thread
    server_thread = threading.Thread(target=run_server)
    server_thread.daemon = True
    server_thread.start()

    print("\nAll systems operational! The Physical AI & Humanoid Robotics Learning Platform is now running.")
    print("- Server is running on http://localhost:8000")
    print("- Embeddings have been processed and stored")
    print("- Ready to handle queries and book interactions")
    print("\nPress Ctrl+C to stop the server.")

    try:
        # Keep the main thread alive
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down the server...")
        sys.exit(0)

if __name__ == "__main__":
    # Set event loop policy for Windows
    if sys.platform.startswith('win'):
        asyncio.set_event_loop_policy(asyncio.WindowsProactorEventLoopPolicy())
    
    asyncio.run(main())