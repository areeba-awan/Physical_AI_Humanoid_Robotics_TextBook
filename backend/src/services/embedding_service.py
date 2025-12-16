import httpx
import os
import asyncio
from typing import List, Union
from dotenv import load_dotenv
import numpy as np

load_dotenv()

class QwenEmbeddingService:
    def __init__(self):
        self.api_key = os.getenv("QWEN_API_KEY")  # Using a generic API key for Qwen
        if not self.api_key:
            # Fallback to Qwen-specific environment variable if needed
            self.api_key = os.getenv("QWEN_EMBEDDINGS_API_KEY")

        # Use the embedding model from environment variable, fallback to default
        self.model = os.getenv("QWEN_EMBEDDING_MODEL", "text-embedding-v1")
        # Use the configured Qwen embedding base URL from environment
        self.base_url = os.getenv("QWEN_EMBEDDING_BASE_URL", "https://dashscope.aliyuncs.com/api/v1/services/embeddings")

        # Using httpx for async HTTP requests
        self.client = httpx.AsyncClient(
            timeout=httpx.Timeout(30.0),
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json"
            }
        )

        # For fallback implementation if Qwen service is not available
        self.use_local_embeddings = os.getenv("USE_LOCAL_EMBEDDINGS", "false").lower() == "true"

        if self.use_local_embeddings:
            # Initialize local embedding model (e.g., using sentence transformers)
            try:
                from sentence_transformers import SentenceTransformer
                self.local_model = SentenceTransformer('all-MiniLM-L6-v2')
            except ImportError:
                print("Warning: sentence_transformers not installed. Using simplified embedding fallback.")
                self.local_model = None

    async def generate_embeddings(self, texts: Union[str, List[str]]) -> Union[List[float], List[List[float]]]:
        """
        Generate embeddings for the input text(s) using Qwen API or a fallback method.

        Args:
            texts: A single text string or a list of text strings to embed

        Returns:
            A list of embeddings (each embedding is a list of floats) if input was a list,
            or a single embedding (list of floats) if input was a single string
        """
        # Ensure texts is a list for consistent processing
        is_single_text = isinstance(texts, str)
        if is_single_text:
            texts = [texts]

        # Validate input
        if not texts:
            return [] if not is_single_text else []

        # Limit batch size for API calls
        max_batch_size = 10  # Adjust based on API limits
        all_embeddings = []

        # Process in batches if needed
        for i in range(0, len(texts), max_batch_size):
            batch = texts[i:i + max_batch_size]

            if self.use_local_embeddings and self.local_model:
                # Use local model for embeddings
                batch_embeddings = self.local_model.encode(batch).tolist()
            else:
                # Use Qwen API for embeddings
                batch_embeddings = await self._call_qwen_api(batch)

            all_embeddings.extend(batch_embeddings)

        # Return appropriate format based on input
        if is_single_text:
            return all_embeddings[0] if all_embeddings else []
        return all_embeddings

    async def _call_qwen_api(self, texts: List[str]) -> List[List[float]]:
        """
        Internal method to call Qwen API for embeddings.

        Args:
            texts: List of text strings to embed

        Returns:
            List of embeddings (each embedding is a list of floats)
        """
        try:
            # Prepare payload for API request
            payload = {
                "model": self.model,
                "input": texts
            }

            # Make the API request
            response = await self.client.post(
                self.base_url,
                json=payload
            )

            if response.status_code != 200:
                raise Exception(f"Qwen API error: {response.status_code} - {response.text}")

            result = response.json()

            # Extract embeddings from response
            # DashScope API returns data in a specific format
            embeddings = []
            if "output" in result and "embeddings" in result["output"]:
                for item in result["output"]["embeddings"]:
                    if "embedding" in item:
                        embeddings.append(item["embedding"])
                    else:
                        # Fallback: generate random embeddings based on configured dimensions
                        import sys
                        from pathlib import Path
                        # Add the backend directory to the Python path
                        backend_dir = Path(__file__).resolve().parent.parent.parent  # Go up to backend root
                        sys.path.insert(0, str(backend_dir))

                        from app.config import settings
                        # Use Qwen embedding dimensions if available, otherwise fallback to OpenAI dimensions
                        embedding_size = settings.qwen_embedding_dimensions if settings.qwen_embedding_dimensions else settings.openai_embedding_dimensions
                        embeddings.append([0.0] * embedding_size)
            elif "data" in result:  # Fallback for different API response formats
                for item in result["data"]:
                    if "embedding" in item:
                        embeddings.append(item["embedding"])
                    else:
                        # Fallback: generate random embeddings based on configured dimensions
                        import sys
                        from pathlib import Path
                        # Add the backend directory to the Python path
                        backend_dir = Path(__file__).resolve().parent.parent.parent  # Go up to backend root
                        sys.path.insert(0, str(backend_dir))

                        from app.config import settings
                        # Use Qwen embedding dimensions if available, otherwise fallback to OpenAI dimensions
                        embedding_size = settings.qwen_embedding_dimensions if settings.qwen_embedding_dimensions else settings.openai_embedding_dimensions
                        embeddings.append([0.0] * embedding_size)
            else:
                # Fallback: generate random embeddings based on configured dimensions
                import sys
                from pathlib import Path
                # Add the backend directory to the Python path
                backend_dir = Path(__file__).resolve().parent.parent.parent  # Go up to backend root
                sys.path.insert(0, str(backend_dir))

                from app.config import settings
                # Use Qwen embedding dimensions if available, otherwise fallback to OpenAI dimensions
                embedding_size = settings.qwen_embedding_dimensions if settings.qwen_embedding_dimensions else settings.openai_embedding_dimensions
                embeddings = [[0.0] * embedding_size for _ in texts]

            return embeddings
        except Exception as e:
            print(f"Error calling Qwen API: {str(e)}")
            # Fallback: return random embeddings based on configured dimensions
            import sys
            from pathlib import Path
            # Add the backend directory to the Python path
            backend_dir = Path(__file__).resolve().parent.parent.parent  # Go up to backend root
            sys.path.insert(0, str(backend_dir))

            from app.config import settings
            # Use Qwen embedding dimensions if available, otherwise fallback to OpenAI dimensions
            embedding_size = settings.qwen_embedding_dimensions if settings.qwen_embedding_dimensions else settings.openai_embedding_dimensions
            return [[0.0] * embedding_size for _ in texts]

    async def close(self):
        """
        Close the HTTP client.
        """
        await self.client.aclose()


# Alternative implementation using a local embedding model
class LocalEmbeddingService:
    def __init__(self):
        try:
            from sentence_transformers import SentenceTransformer
            self.model = SentenceTransformer('all-MiniLM-L6-v2')
        except ImportError:
            raise ImportError(
                "sentence-transformers is required for local embeddings. "
                "Install it with: pip install sentence-transformers"
            )

    def generate_embeddings(self, texts: Union[str, List[str]]) -> Union[List[float], List[List[float]]]:
        """
        Generate embeddings using a local model.

        Args:
            texts: A single text string or a list of text strings to embed

        Returns:
            A list of embeddings (each embedding is a list of floats) if input was a list,
            or a single embedding (list of floats) if input was a single string
        """
        is_single_text = isinstance(texts, str)
        if is_single_text:
            texts = [texts]

        embeddings = self.model.encode(texts).tolist()

        if is_single_text:
            return embeddings[0] if embeddings else []
        return embeddings


# Global instance for the application
# The actual service to use will depend on configuration
qwen_embedding_service = QwenEmbeddingService()