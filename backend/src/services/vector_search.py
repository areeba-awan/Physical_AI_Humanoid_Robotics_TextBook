import os
from typing import List, Dict, Any, Optional
from backend.src.core.vector_db import vector_db
from backend.src.services.embedding_service import qwen_embedding_service
import logging

logger = logging.getLogger(__name__)

class VectorSearchService:
    def __init__(self):
        self.retrieval_limit = int(os.getenv("RETRIEVAL_LIMIT", "5"))
    
    async def search_similar_content(
        self, 
        query: str, 
        book_id: Optional[str] = None, 
        limit: Optional[int] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for content similar to the query in the vector database.
        
        Args:
            query: The text to search for similar content
            book_id: Optional book ID to filter search to a specific book
            limit: Maximum number of results to return (defaults to RETRIEVAL_LIMIT)
            
        Returns:
            List of similar content chunks with their metadata and similarity scores
        """
        limit = limit or self.retrieval_limit
        
        try:
            # Generate embedding for the query
            query_embedding = await qwen_embedding_service.generate_embeddings(query)
            
            # Prepare filters
            filters = {}
            if book_id:
                filters["book_id"] = book_id
            
            # Search in vector database
            results = vector_db.search(
                query_vector=query_embedding,
                limit=limit,
                filters=filters
            )
            
            return results
        except Exception as e:
            logger.error(f"Error in vector search: {str(e)}")
            raise e
    
    async def search_selected_text_content(
        self,
        query: str,
        selected_text: str,
        book_id: str,
        limit: Optional[int] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for content similar to the query, but constrained to the selected text.
        
        Args:
            query: The text to search for similar content
            selected_text: The text that has been specifically selected by the user
            book_id: The book ID to search within
            limit: Maximum number of results to return
            
        Returns:
            List of similar content chunks with their metadata and similarity scores
        """
        limit = limit or self.retrieval_limit
        
        try:
            # Generate embedding for the combined query and selected text
            query_embedding = await qwen_embedding_service.generate_embeddings(
                f"{query} {selected_text}"
            )
            
            # Search in vector database but filter to the specific context
            filters = {
                "book_id": book_id
            }
            
            # For selected-text-only mode, we need to ensure the search is
            # relevant to the selected text
            results = vector_db.search(
                query_vector=query_embedding,
                limit=limit,
                filters=filters
            )
            
            # Filter results to ensure they are relevant to the selected text
            filtered_results = []
            for result in results:
                # For selected-text-only mode, we only return results that are
                # somehow related to the selected text
                if self._is_relevant_to_selected_text(result, selected_text):
                    filtered_results.append(result)
            
            return filtered_results[:limit]
        except Exception as e:
            logger.error(f"Error in selected text vector search: {str(e)}")
            raise e
    
    def _is_relevant_to_selected_text(self, result: Dict[str, Any], selected_text: str) -> bool:
        """
        Determine if a search result is relevant to the selected text.
        In a real implementation, this would use more sophisticated logic.
        
        Args:
            result: A search result with payload
            selected_text: The user-selected text
            
        Returns:
            True if the result is relevant to the selected text
        """
        # Simple implementation: check if selected text appears in the result preview
        # or if they share significant keywords
        preview_text = result.get("payload", {}).get("text_preview", "").lower()
        selected_lower = selected_text.lower()
        
        # Check if any significant words from selected_text appear in the result
        selected_words = selected_lower.split()
        common_words = [word for word in selected_words if len(word) > 3 and word in preview_text]
        
        # If more than 20% of significant words are found, consider relevant
        if len(selected_words) > 0:
            return len(common_words) / len(selected_words) > 0.2
        else:
            return True  # If no significant words, we can't determine, so accept
    
    async def get_content_by_id(self, content_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific content chunk by its ID.
        
        Args:
            content_id: The ID of the content chunk to retrieve
            
        Returns:
            The content chunk data or None if not found
        """
        return vector_db.get_point(content_id)


# Global instance for the application
vector_search_service = VectorSearchService()