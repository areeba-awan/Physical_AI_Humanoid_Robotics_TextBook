import os
from typing import List, Dict, Any, Optional
from backend.src.services.embedding_service import qwen_embedding_service
from backend.src.services.vector_search import vector_search_service
from backend.src.core.llm_client import openrouter_client
from backend.src.models.models import UserQuery, GeneratedResponse, Session
from backend.src.core.database import get_db
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
import logging
import json

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self):
        self.max_context_chunks = int(os.getenv("MAX_CONTEXT_CHUNKS", "5"))
        self.system_prompt_template = os.getenv(
            "SYSTEM_PROMPT",
            (
                "You are a helpful assistant that answers questions based only on the provided context. "
                "Do not use any prior knowledge or information not present in the context. "
                "If the context doesn't contain information to answer the question, say so explicitly. "
                "Always cite which context items you used to form your answer. "
                "Respond with accuracy and precision."
            )
        )

    async def process_query(
        self,
        db: AsyncSession,
        session_id: str,
        query_text: str,
        include_context: bool = True,
        include_chat_history: bool = True,  # New parameter for chat memory
        current_page: Optional[int] = None,
        current_section: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Process a user query using the RAG pipeline.

        Args:
            db: Database session
            session_id: ID of the session
            query_text: The user's query
            include_context: Whether to include context in the response
            current_page: Current page the user is reading (used for context)
            current_section: Current section the user is reading (used for context)

        Returns:
            Dictionary with response and metadata
        """
        try:
            logger.info(f"Processing query for session {session_id}: {query_text[:100]}...")

            # Get chat history for the session if enabled
            chat_history = []
            if include_chat_history:
                from backend.src.services.chat_history_service import chat_message_service
                chat_messages = await chat_message_service.get_messages_by_session_id(
                    db=db,
                    session_id=session_id,
                    limit=10  # Get last 10 messages
                )

                # Format chat history for the LLM context
                for msg in chat_messages:
                    role = "assistant" if msg.message_type == "assistant" else "user"
                    chat_history.append({
                        "role": role,
                        "content": msg.content
                    })

            # 1. Store the user query in the database
            user_query = UserQuery(
                session_id=session_id,
                query_text=query_text
            )
            db.add(user_query)
            await db.commit()
            await db.refresh(user_query)
            logger.debug(f"Stored user query {user_query.id} in database")

            # 2. If we should include context, retrieve relevant content
            context_chunks = []
            if include_context:
                # Get the session to determine the book and reading context
                session_result = await db.execute(
                    select(Session).filter(Session.id == session_id)
                )
                session = session_result.scalar_one_or_none()

                if session:
                    logger.debug(f"Searching for context in book {session.book_id} with reading context")

                    # Search for relevant content in the vector database, considering reading context
                    search_results = await vector_search_service.search_similar_content(
                        query=query_text,
                        book_id=str(session.book_id),
                        limit=self.max_context_chunks
                    )

                    # If the user has reading context, prioritize results that match the current section/page
                    if current_section or current_page:
                        # Sort results by relevance to current reading position
                        # For now, we'll just log this context for future enhancement
                        logger.info(f"Current reading context: page={current_page}, section={current_section}")

                        # In the future, we could apply ranking algorithms to prioritize
                        # content that's closer to the current reading position
                        search_results = self._prioritize_contextual_results(
                            search_results, current_page, current_section
                        )

                    # Extract content texts from search results
                    context_chunks = [result.get("payload", {}).get("text_preview", "") for result in search_results]
                    logger.info(f"Found {len(context_chunks)} context chunks for query")
                else:
                    logger.warning(f"Session {session_id} not found for context retrieval")

            # 3. Generate response using LLM with enhanced context
            try:
                response_data = await self._generate_response_with_context(
                    query_text, context_chunks, chat_history
                )
                logger.debug("LLM response generated successfully")
            except Exception as e:
                logger.error(f"Error generating response with LLM: {str(e)}", exc_info=True)
                # Return a user-friendly error response
                response_data = {
                    "content": "Sorry, I'm having trouble generating a response right now. Please try again later.",
                    "confidence": 0.0,
                    "referenced_content": []
                }

            # 4. Store the generated response in the database with content references
            from backend.src.models.generated_response import generated_response_service

            content_references_data = []
            if "referenced_content" in response_data and response_data["referenced_content"]:
                # Extract content reference information from the search results
                # For now, we'll create reference data from the context chunks
                search_results = await vector_search_service.search_similar_content(
                    query=query_text,
                    book_id=str(session.book_id),
                    limit=self.max_context_chunks
                )

                for i, result in enumerate(search_results):
                    if i < len(response_data["referenced_content"]):
                        content_references_data.append({
                            "content_id": result.get("id", "unknown"),
                            "text_snippet": result.get("payload", {}).get("text_preview", "")[:200] + "...",
                            "page_number": result.get("payload", {}).get("page_number"),  # Extract from payload if available
                            "section_title": result.get("payload", {}).get("section_title")  # Extract from payload if available
                        })

            generated_response = await generated_response_service.create_generated_response(
                db=db,
                query_id=user_query.id,
                response_text=response_data.get("content", ""),
                referenced_content=json.dumps(response_data.get("referenced_content", [])),
                confidence_score=response_data.get("confidence", 0.8),  # Default confidence
                content_references=content_references_data
            )
            logger.debug(f"Stored generated response {generated_response.id} in database")

            # Store the user query as a chat message
            from backend.src.services.chat_history_service import chat_message_service
            await chat_message_service.create_chat_message(
                db=db,
                session_id=session_id,
                message_type="user",
                content=query_text
            )

            # Store the AI response as a chat message
            await chat_message_service.create_chat_message(
                db=db,
                session_id=session_id,
                message_type="assistant",
                content=response_data.get("content", "")
            )

            logger.info(f"Query processed successfully for session {session_id}")

            # 5. Format and return the response
            return {
                "query_id": str(user_query.id),
                "response": response_data.get("content", ""),
                "references": response_data.get("referenced_content", []),
                "confidence": response_data.get("confidence", 0.8),
                "session_id": session_id
            }
        except Exception as e:
            logger.error(f"Error processing query: {str(e)}", exc_info=True)
            raise e

    def _prioritize_contextual_results(self, search_results: List[Dict], current_page: Optional[int], current_section: Optional[str]) -> List[Dict]:
        """
        Prioritize search results based on the current reading context.

        Args:
            search_results: List of search results from vector database
            current_page: Current page the user is reading
            current_section: Current section the user is reading

        Returns:
            Prioritized list of results
        """
        # This is a basic implementation - in the future, we could implement more sophisticated
        # ranking algorithms based on proximity to current reading position
        if not current_page and not current_section:
            # No context to prioritize by, return original order
            return search_results

        # Create a list of results with calculated priority scores
        scored_results = []
        for result in search_results:
            payload = result.get("payload", {})
            score = 0.0

            # Boost results that match the current section
            if current_section and payload.get("section_title"):
                if current_section.lower() in payload["section_title"].lower():
                    score += 10.0  # Significant boost for matching section

            # Boost results that are near the current page
            if current_page and payload.get("page_number") is not None:
                page_diff = abs(int(payload["page_number"]) - current_page)
                # Add score based on proximity (closer pages get higher scores)
                score += max(0, 5 - page_diff)  # Scores from 0 to 5 based on page proximity

            scored_results.append((result, score))

        # Sort by score (descending) and return the results in priority order
        sorted_results = sorted(scored_results, key=lambda x: x[1], reverse=True)
        return [result for result, score in sorted_results]

    async def process_selected_text_query(
        self,
        db: AsyncSession,
        session_id: str,
        query_text: str,
        selected_text: str,
        current_page: Optional[int] = None,
        current_section: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Process a query that should only use the selected text for context.

        Args:
            db: Database session
            session_id: ID of the session
            query_text: The user's query
            selected_text: The text that has been selected by the user
            current_page: Current page the user is reading (used for context)
            current_section: Current section the user is reading (used for context)

        Returns:
            Dictionary with response and metadata
        """
        try:
            logger.info(f"Processing selected text query for session {session_id}")

            # 1. Store the user query in the database with context metadata
            user_query = UserQuery(
                session_id=session_id,
                query_text=query_text,
                query_metadata=json.dumps({
                    "selected_text": selected_text[:100] + "..." if len(selected_text) > 100 else selected_text,
                    "current_page": current_page,
                    "current_section": current_section
                })
            )
            db.add(user_query)
            await db.commit()
            await db.refresh(user_query)
            logger.debug(f"Stored selected text query {user_query.id} in database")

            # 2. For selected text mode, use the selected text as context
            context_chunks = [selected_text]
            logger.debug(f"Using selected text as context (length: {len(selected_text)})")

            # Get chat history for the session if enabled
            chat_history = []  # For now, we won't include chat history in selected text mode, but we could in the future
            if True:  # Always include chat history for selected text mode as well
                from backend.src.services.chat_history_service import chat_message_service
                chat_messages = await chat_message_service.get_messages_by_session_id(
                    db=db,
                    session_id=session_id,
                    limit=10  # Get last 10 messages
                )

                # Format chat history for the LLM context
                for msg in chat_messages:
                    role = "assistant" if msg.message_type == "assistant" else "user"
                    chat_history.append({
                        "role": role,
                        "content": msg.content
                    })

            # 3. Generate response using LLM with selected text as context
            try:
                response_data = await self._generate_response_with_context(
                    query_text, context_chunks, chat_history, mode="selected_text"
                )
                logger.debug("LLM response generated successfully for selected text query")
            except Exception as e:
                logger.error(f"Error generating response with LLM for selected text: {str(e)}", exc_info=True)
                # Return a user-friendly error response
                response_data = {
                    "content": "Sorry, I'm having trouble generating a response right now. Please try again later.",
                    "confidence": 0.0,
                    "referenced_content": []
                }

            # 4. Store the generated response in the database with content references
            from backend.src.models.generated_response import generated_response_service

            # For selected text queries, we reference the selected text itself
            content_references_data = [{
                "content_id": "selected_text",  # Placeholder ID for selected text
                "text_snippet": selected_text[:200] + "...",
                "page_number": current_page,
                "section_title": current_section
            }]

            generated_response = await generated_response_service.create_generated_response(
                db=db,
                query_id=user_query.id,
                response_text=response_data.get("content", ""),
                referenced_content=json.dumps(response_data.get("referenced_content", [])),
                confidence_score=response_data.get("confidence", 0.9),  # Higher confidence for selected text
                content_references=content_references_data
            )
            logger.debug(f"Stored generated response {generated_response.id} in database")

            # Store the user query as a chat message
            from backend.src.services.chat_history_service import chat_message_service
            await chat_message_service.create_chat_message(
                db=db,
                session_id=session_id,
                message_type="user",
                content=query_text
            )

            # Store the AI response as a chat message
            await chat_message_service.create_chat_message(
                db=db,
                session_id=session_id,
                message_type="assistant",
                content=response_data.get("content", "")
            )

            logger.info(f"Selected text query processed successfully for session {session_id}")

            # 5. Format and return the response
            return {
                "query_id": str(user_query.id),
                "response": response_data.get("content", ""),
                "references": response_data.get("referenced_content", []),
                "confidence": response_data.get("confidence", 0.9),
                "session_id": session_id
            }
        except Exception as e:
            logger.error(f"Error processing selected text query: {str(e)}", exc_info=True)
            raise e

    async def _generate_response_with_context(
        self,
        query: str,
        context_chunks: List[str],
        chat_history: Optional[List[dict]] = None,
        mode: str = "general"
    ) -> Dict[str, Any]:
        """
        Internal method to generate a response using the LLM with context.

        Args:
            query: The user's question
            context_chunks: List of context chunks to provide to the LLM
            chat_history: Previous conversation history (optional)
            mode: The query mode (general or selected_text)

        Returns:
            Dictionary with response content and metadata
        """

        if chat_history is None:
            chat_history = []
        try:
            if mode == "selected_text":
                # For selected text mode, emphasize using only the provided text
                system_prompt = (
                    "You are a helpful assistant that answers questions based ONLY on the provided selected text. "
                    "Do not use any prior knowledge or information not explicitly present in the selected text provided below. "
                    "If the selected text doesn't contain information to answer the question, explicitly state that the information is not available in the provided text. "
                    "Cite information from the selected text when forming your answer. "
                    "Be accurate and precise based solely on the provided text."
                )
            else:
                # For general mode, use the default system prompt
                system_prompt = self.system_prompt_template

            # Format the context for the LLM
            if context_chunks:
                formatted_context = "\n\n".join([f"Context {i+1}: {chunk}" for i, chunk in enumerate(context_chunks)])
                full_prompt = (
                    f"Context Information:\n{formatted_context}\n\n"
                    f"Question: {query}\n\n"
                    f"Please provide an answer based ONLY on the context information provided above. "
                    f"Do not use any external knowledge."
                )
            else:
                # If no context chunks found, inform the LLM
                full_prompt = (
                    f"Question: {query}\n\n"
                    f"I don't have relevant context from the book to answer this question. "
                    f"Please acknowledge that you don't have the necessary information from the book to answer this question."
                )
                
                # Use a different system prompt for this case
                system_prompt = (
                    "You are a helpful assistant. If you don't have relevant context to answer a question, "
                    "acknowledge that you don't have the necessary information from the book to answer this question."
                )

            # Generate response using the OpenRouter API
            response = await openrouter_client.generate_response(
                prompt=full_prompt,
                system_prompt=system_prompt,
                max_tokens=1000,
                temperature=0.3  # Lower temperature for more consistent, fact-based responses
            )
            
            # Calculate confidence based on context availability and response quality
            confidence = self._calculate_confidence(response, context_chunks)
            
            return {
                "content": response.get("content", ""),
                "model": response.get("model", "unknown"),
                "referenced_content": context_chunks[:2],  # Return top 2 context chunks as references
                "confidence": confidence
            }
        except Exception as e:
            logger.error(f"Error generating response with context: {str(e)}")
            raise e

    def _calculate_confidence(self, response: Dict[str, Any], context_chunks: List[str]) -> float:
        """
        Calculate confidence score for the response based on various factors.
        
        Args:
            response: The response from the LLM
            context_chunks: The context chunks used for the response
            
        Returns:
            Confidence score between 0 and 1
        """
        if not context_chunks:
            # If no context was available, return low confidence
            return 0.2
        
        content = response.get("content", "").lower()
        
        # Check if the response acknowledges lack of information
        if any(phrase in content for phrase in ["don't have", "no information", "not mentioned", "not in the text"]):
            return 0.3  # Low confidence if it couldn't answer
        
        # Base confidence on number of context chunks used
        num_chunks = len(context_chunks)
        base_confidence = min(0.7, 0.3 + (num_chunks * 0.1))  # 0.4 to 0.7 based on context
        
        # Additional factors could be added here in the future
        # such as semantic similarity between query and context, etc.
        
        return min(1.0, base_confidence)  # Cap at 1.0


# Global instance for the application
rag_service = RAGService()