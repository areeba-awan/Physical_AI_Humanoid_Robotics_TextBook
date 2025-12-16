from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from typing import List, Optional, Dict, Any
import os
from dotenv import load_dotenv

load_dotenv()

class VectorDBManager:
    def __init__(self):
        # Initialize Qdrant client with environment configuration
        self.client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY"),
            prefer_grpc=True  # Use gRPC for better performance if available
        )
        
        # Get collection name from environment or use default
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content")
        
        # Get embedding dimension from environment or use default
        self.embedding_dimension = int(os.getenv("EMBEDDING_DIMENSION", "768"))
        
        # Initialize the collection if it doesn't exist
        self._initialize_collection()
    
    def _initialize_collection(self):
        """
        Create the collection if it doesn't exist.
        """
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
        except:
            # Collection doesn't exist, create it
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.embedding_dimension,
                    distance=Distance.COSINE
                )
            )
    
    def add_vectors(self, 
                    vectors: List[List[float]], 
                    payloads: List[Dict[str, Any]], 
                    ids: Optional[List[str]] = None):
        """
        Add vectors to the collection.
        
        Args:
            vectors: List of embedding vectors
            payloads: List of metadata associated with each vector
            ids: Optional list of IDs for the vectors
        """
        if ids is None:
            # Generate IDs if not provided
            import uuid
            ids = [str(uuid.uuid4()) for _ in range(len(vectors))]
        
        # Ensure all vectors have the correct dimension
        assert all(len(v) == self.embedding_dimension for v in vectors), \
            f"All vectors must have dimension {self.embedding_dimension}"
        
        # Upload points to Qdrant
        self.client.upsert(
            collection_name=self.collection_name,
            points=models.Batch(
                ids=ids,
                vectors=vectors,
                payloads=payloads
            )
        )
        
        return ids
    
    def search(self, 
               query_vector: List[float], 
               limit: int = 5, 
               filters: Optional[Dict] = None) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in the collection.
        
        Args:
            query_vector: The embedding vector to search for
            limit: Number of results to return
            filters: Optional filters to apply to the search
            
        Returns:
            List of matching points with payload and score
        """
        # Ensure query vector has correct dimension
        assert len(query_vector) == self.embedding_dimension, \
            f"Query vector must have dimension {self.embedding_dimension}"
        
        # Prepare filters if provided
        search_filter = None
        if filters:
            filter_conditions = []
            for key, value in filters.items():
                filter_conditions.append(
                    models.FieldCondition(
                        key=key,
                        match=models.MatchValue(value=value)
                    )
                )
            if filter_conditions:
                search_filter = models.Filter(must=filter_conditions)
        
        # Perform search
        results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=limit,
            query_filter=search_filter
        )
        
        # Format results
        formatted_results = []
        for result in results:
            formatted_results.append({
                "id": result.id,
                "payload": result.payload,
                "score": result.score
            })
        
        return formatted_results
    
    def delete_by_payload(self, field: str, value: Any):
        """
        Delete points from the collection based on payload field value.
        
        Args:
            field: The payload field to filter by
            value: The value to match in the field
        """
        # Create filter to identify points to delete
        filter_condition = models.Filter(
            must=[
                models.FieldCondition(
                    key=field,
                    match=models.MatchValue(value=value)
                )
            ]
        )
        
        # Get points matching the filter
        points = self.client.scroll(
            collection_name=self.collection_name,
            scroll_filter=filter_condition,
            limit=10000  # Adjust based on expected number of points to delete
        )
        
        # Extract IDs of points to delete
        ids_to_delete = [point.id for point, _ in points]
        
        if ids_to_delete:
            # Delete the points
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(points=ids_to_delete)
            )
    
    def get_point(self, point_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a specific point by ID.
        
        Args:
            point_id: The ID of the point to retrieve
            
        Returns:
            The point data or None if not found
        """
        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[point_id]
            )
            if records:
                record = records[0]
                return {
                    "id": record.id,
                    "payload": record.payload,
                    "vector": record.vector
                }
        except:
            return None
        return None


# Global instance for the application
vector_db = VectorDBManager()