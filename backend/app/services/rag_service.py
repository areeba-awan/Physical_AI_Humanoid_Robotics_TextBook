"""
RAG Service for Retrieval-Augmented Generation

Handles:
- Semantic search in Qdrant vector database
- Embedding generation for queries using Qwen
- Context building with citations
- Two query modes: Global and Selected Text
- Strict no-hallucination policy
"""

from typing import List, Dict, Any, Optional, Tuple
from qdrant_client import QdrantClient
from qdrant_client.http.models import (
    Filter,
    FieldCondition,
    MatchValue,
    SearchParams,
)
import logging

from app.config import settings
import sys
from pathlib import Path
# Add the backend directory to the Python path
backend_dir = Path(__file__).resolve().parent.parent.parent  # Go up to backend
sys.path.insert(0, str(backend_dir))

from src.services.embedding_service import qwen_embedding_service

logger = logging.getLogger(__name__)


class RAGService:
    """Service for RAG (Retrieval-Augmented Generation) operations."""

    # Lower threshold to capture more potentially relevant results
    # Semantic similarity can be lower than expected for Q&A pairs
    DEFAULT_SCORE_THRESHOLD = 0.35

    # Minimum number of results before applying strict threshold
    MIN_RESULTS_SOFT = 3

    # Context window limits
    MAX_CONTEXT_TOKENS = 4000
    MAX_CONTEXT_CHARS = MAX_CONTEXT_TOKENS * 4  # Rough estimate

    def __init__(self):
        self.qdrant = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key if settings.qdrant_api_key else None,
        )
        self.collection_name = settings.qdrant_collection

    async def get_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for text using Qwen embedding service.

        IMPORTANT: Must be consistent with ingestion embedding dimensions.
        """
        try:
            # Use Qwen embedding service to generate embeddings
            embedding = await qwen_embedding_service.generate_embeddings(text)
            return embedding
        except Exception as e:
            logger.error(f"Embedding generation failed: {e}")
            raise

    async def search_documents(
        self,
        query: str,
        limit: int = 10,
        chapter_filter: Optional[str] = None,
        module_filter: Optional[str] = None,
        score_threshold: Optional[float] = None,
    ) -> List[Dict[str, Any]]:
        """
        Search for relevant documents in Qdrant.

        Uses a two-pass approach:
        1. First pass: Get more results with lower threshold
        2. Second pass: Filter and rank by relevance
        """
        if score_threshold is None:
            score_threshold = self.DEFAULT_SCORE_THRESHOLD

        # Generate query embedding
        query_embedding = await self.get_embedding(query)

        # Build filter conditions
        filter_conditions = []
        if chapter_filter:
            filter_conditions.append(
                FieldCondition(
                    key="chapter_id",
                    match=MatchValue(value=chapter_filter),
                )
            )
        if module_filter:
            filter_conditions.append(
                FieldCondition(
                    key="module_id",
                    match=MatchValue(value=module_filter),
                )
            )

        query_filter = None
        if filter_conditions:
            query_filter = Filter(must=filter_conditions)

        # Search in Qdrant with generous limit for filtering
        try:
            results = self.qdrant.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit * 2,  # Get extra results for filtering
                query_filter=query_filter,
                score_threshold=score_threshold * 0.8,  # Lower threshold initially
                search_params=SearchParams(
                    hnsw_ef=256,  # Higher accuracy for better results
                    exact=False,
                ),
            )
        except Exception as e:
            logger.error(f"Qdrant search error: {e}")
            return []

        if not results:
            logger.warning(f"No results found for query: {query[:100]}...")
            return []

        # Format and filter results
        documents = []
        for result in results:
            # Apply stricter threshold after getting results
            if result.score >= score_threshold or len(documents) < self.MIN_RESULTS_SOFT:
                documents.append({
                    "id": result.id,
                    "content": result.payload.get("content", ""),
                    "metadata": {
                        "chapter_id": result.payload.get("chapter_id", ""),
                        "section": result.payload.get("section", ""),
                        "module_id": result.payload.get("module_id", ""),
                        "page_number": result.payload.get("page_number", 0),
                        "chunk_index": result.payload.get("chunk_index", 0),
                    },
                    "score": result.score,
                })

            if len(documents) >= limit:
                break

        logger.info(f"Found {len(documents)} relevant documents for query")
        return documents

    async def get_context_for_query(
        self,
        query: str,
        chapter_filter: Optional[str] = None,
        max_tokens: int = None,
        min_results: int = 3,
    ) -> Tuple[str, List[Dict[str, Any]], bool]:
        """
        Get relevant context for a query (Global Book Query mode).

        Returns:
            - context: Combined text from relevant documents
            - citations: List of citation objects
            - has_context: Whether any relevant context was found
        """
        if max_tokens is None:
            max_tokens = self.MAX_CONTEXT_TOKENS

        max_chars = max_tokens * 4

        documents = await self.search_documents(
            query=query,
            limit=12,  # Get more documents for better context
            chapter_filter=chapter_filter,
        )

        if not documents:
            return "", [], False

        # Build context string with character limit
        context_parts = []
        citations = []
        current_chars = 0

        # Track unique sources to avoid duplicate citations
        seen_sources = set()

        for doc in documents:
            doc_content = doc["content"]
            doc_chars = len(doc_content)

            # Create source key for deduplication
            source_key = f"{doc['metadata']['chapter_id']}:{doc['metadata']['section']}"

            if current_chars + doc_chars > max_chars:
                # Check if we have minimum results
                if len(context_parts) >= min_results:
                    break
                # Otherwise, truncate this document
                remaining_chars = max_chars - current_chars
                if remaining_chars > 200:  # Only add if meaningful
                    truncated_content = doc_content[:remaining_chars]
                    context_parts.append(
                        f"[Source: {doc['metadata']['chapter_id']} - {doc['metadata']['section']}]\n"
                        f"{truncated_content}..."
                    )
                    if source_key not in seen_sources:
                        citations.append({
                            "chapter_id": doc["metadata"]["chapter_id"],
                            "section": doc["metadata"]["section"],
                            "relevance": round(doc["score"], 3),
                            "snippet": truncated_content[:150] + "...",
                        })
                        seen_sources.add(source_key)
                break

            context_parts.append(
                f"[Source: {doc['metadata']['chapter_id']} - {doc['metadata']['section']}]\n"
                f"{doc_content}"
            )

            if source_key not in seen_sources:
                snippet = doc_content[:200] + "..." if len(doc_content) > 200 else doc_content
                citations.append({
                    "chapter_id": doc["metadata"]["chapter_id"],
                    "section": doc["metadata"]["section"],
                    "relevance": round(doc["score"], 3),
                    "snippet": snippet,
                })
                seen_sources.add(source_key)

            current_chars += doc_chars

        context = "\n\n---\n\n".join(context_parts)
        has_context = len(context_parts) > 0

        logger.info(f"Built context with {len(context_parts)} chunks, {len(citations)} citations")
        return context, citations, has_context

    async def verify_answer_in_context(
        self,
        query: str,
        context: str,
        answer: str,
    ) -> Tuple[bool, float]:
        """
        Verify that the answer is grounded in the context.

        This is a critical anti-hallucination check.

        Returns:
            - is_grounded: Whether the answer appears grounded in context
            - confidence: Confidence score (0-1)
        """
        # Use embedding similarity as a quick check
        try:
            query_emb = await self.get_embedding(query)
            context_emb = await self.get_embedding(context[:2000])  # Limit context for embedding
            answer_emb = await self.get_embedding(answer)

            # Calculate similarities
            def cosine_sim(a: List[float], b: List[float]) -> float:
                dot = sum(x * y for x, y in zip(a, b))
                norm_a = sum(x * x for x in a) ** 0.5
                norm_b = sum(y * y for y in b) ** 0.5
                return dot / (norm_a * norm_b) if norm_a and norm_b else 0

            # Answer should be more similar to context than to query alone
            answer_context_sim = cosine_sim(answer_emb, context_emb)

            # Consider grounded if answer is related to context
            is_grounded = answer_context_sim > 0.4
            confidence = min(1.0, answer_context_sim + 0.3)

            return is_grounded, confidence
        except Exception as e:
            logger.warning(f"Grounding verification failed: {e}")
            return True, 0.5  # Default to trusting the answer if check fails

    async def analyze_selected_text_relevance(
        self,
        query: str,
        selected_text: str,
    ) -> Tuple[bool, str, float]:
        """
        Check if the selected text contains information to answer the query.

        Uses a more nuanced approach that considers:
        1. Text length (must be substantial)
        2. Semantic overlap between query concepts and text content
        3. Presence of key terms from the query

        Returns:
            - is_sufficient: Whether the text is sufficient to answer
            - analysis: Brief analysis of relevance
            - relevance_score: Numeric relevance score
        """
        # Basic length check
        word_count = len(selected_text.split())
        char_count = len(selected_text)

        if word_count < 5 or char_count < 20:
            return False, "Selected text is too short to contain meaningful information.", 0.0

        # Extract key terms from query (simple approach)
        query_terms = set(query.lower().split())
        text_terms = set(selected_text.lower().split())

        # Remove common words
        stop_words = {'what', 'is', 'the', 'a', 'an', 'how', 'do', 'does', 'can', 'will',
                      'this', 'that', 'which', 'when', 'where', 'why', 'to', 'of', 'in',
                      'for', 'on', 'with', 'as', 'by', 'at', 'from', 'or', 'and', 'be'}

        query_keywords = query_terms - stop_words
        text_keywords = text_terms - stop_words

        # Calculate keyword overlap
        overlap = len(query_keywords & text_keywords)
        overlap_ratio = overlap / len(query_keywords) if query_keywords else 0

        # Generate embeddings for semantic similarity
        try:
            query_embedding = await self.get_embedding(query)
            text_embedding = await self.get_embedding(selected_text)

            # Calculate cosine similarity
            dot_product = sum(a * b for a, b in zip(query_embedding, text_embedding))
            query_norm = sum(a * a for a in query_embedding) ** 0.5
            text_norm = sum(b * b for b in text_embedding) ** 0.5
            semantic_similarity = dot_product / (query_norm * text_norm)
        except Exception as e:
            logger.warning(f"Embedding failed for selected text analysis: {e}")
            semantic_similarity = 0.3  # Default moderate similarity

        # Combined relevance score
        # Weight semantic similarity higher for conceptual questions
        # Weight keyword overlap higher for specific term questions
        relevance_score = (semantic_similarity * 0.6) + (overlap_ratio * 0.4)

        # Determine sufficiency with lower threshold for longer text
        length_bonus = min(0.1, word_count / 500)  # Up to 0.1 bonus for longer text
        adjusted_score = relevance_score + length_bonus

        # More lenient threshold - if text is substantial, trust it
        is_sufficient = adjusted_score > 0.25 or (word_count > 50 and semantic_similarity > 0.2)

        analysis = (
            f"Relevance: {relevance_score:.2f}, "
            f"Semantic: {semantic_similarity:.2f}, "
            f"Keyword overlap: {overlap}/{len(query_keywords)}, "
            f"Words: {word_count}"
        )

        logger.info(f"Selected text analysis: {analysis}, sufficient={is_sufficient}")

        return is_sufficient, analysis, relevance_score

    async def get_context_for_selected_text(
        self,
        query: str,
        selected_text: str,
    ) -> Tuple[str, List[Dict[str, Any]], bool, str]:
        """
        Get context strictly from selected text (Selected Text Query mode).

        This mode ONLY uses the provided selected text.
        If the text is insufficient, returns a flag indicating so.

        Returns:
            - context: The selected text as context
            - citations: Empty list (no external citations)
            - is_sufficient: Whether the text is sufficient to answer
            - message: Additional message about sufficiency
        """
        # Check if selected text is relevant and sufficient
        is_sufficient, analysis, relevance_score = await self.analyze_selected_text_relevance(
            query, selected_text
        )

        if not is_sufficient:
            return (
                selected_text,
                [],
                False,
                "The selected text does not contain enough information to answer this question. "
                "Please select more relevant text or use the global search feature."
            )

        return (
            selected_text,
            [],
            True,
            f"Answering based on selected text. ({analysis})"
        )

    def check_collection_health(self) -> Dict[str, Any]:
        """Check if the Qdrant collection is healthy and has data."""
        try:
            info = self.qdrant.get_collection(self.collection_name)
            return {
                "healthy": info.status.value == "green",
                "status": info.status.value,
                "vectors_count": info.vectors_count,
                "points_count": info.points_count,
                "collection_name": self.collection_name,
            }
        except Exception as e:
            logger.error(f"Collection health check failed: {e}")
            return {
                "healthy": False,
                "status": "error",
                "error": str(e),
                "collection_name": self.collection_name,
            }

    async def get_collection_sample(self, limit: int = 5) -> List[Dict[str, Any]]:
        """Get sample documents from collection for debugging."""
        try:
            results = self.qdrant.scroll(
                collection_name=self.collection_name,
                limit=limit,
                with_payload=True,
                with_vectors=False,
            )

            samples = []
            for point in results[0]:
                samples.append({
                    "id": point.id,
                    "chapter_id": point.payload.get("chapter_id", ""),
                    "section": point.payload.get("section", ""),
                    "content_preview": point.payload.get("content", "")[:200],
                })
            return samples
        except Exception as e:
            logger.error(f"Failed to get collection sample: {e}")
            return []
