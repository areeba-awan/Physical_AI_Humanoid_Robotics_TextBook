import httpx
import os
import json
from typing import Dict, List, Any, Optional
from dotenv import load_dotenv
import asyncio
from pydantic import BaseModel

load_dotenv()

class OpenRouterResponse(BaseModel):
    id: str
    choices: List[Dict[str, Any]]
    created: int
    model: str
    object: str
    usage: Dict[str, int]

class OpenRouterClient:
    def __init__(self):
        self.api_key = os.getenv("OPENROUTER_API_KEY")
        if not self.api_key:
            raise ValueError("OPENROUTER_API_KEY environment variable is required")
        
        self.base_url = "https://openrouter.ai/api/v1"
        self.default_model = os.getenv("OPENROUTER_MODEL", "openchat/openchat-7b")
        
        # Create async client with timeout
        self.client = httpx.AsyncClient(
            timeout=httpx.Timeout(30.0),
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json"
            }
        )
    
    async def generate_response(
        self,
        prompt: str,
        model: Optional[str] = None,
        max_tokens: int = 1000,
        temperature: float = 0.7,
        stream: bool = False,
        system_prompt: Optional[str] = None,
        context: Optional[List[Dict[str, str]]] = None
    ) -> Dict[str, Any]:
        """
        Generate a response from the OpenRouter API.
        
        Args:
            prompt: The user's input/prompt
            model: The model to use (defaults to environment variable)
            max_tokens: Maximum number of tokens to generate
            temperature: Sampling temperature (0.0 to 1.0)
            stream: Whether to stream the response
            system_prompt: System prompt to guide the model
            context: Conversation context (list of messages)
            
        Returns:
            Dict containing the generated response
        """
        model = model or self.default_model
        
        # Prepare messages for the API call
        messages = []
        
        # Add system prompt if provided
        if system_prompt:
            messages.append({
                "role": "system",
                "content": system_prompt
            })
        
        # Add context if provided (conversation history)
        if context:
            messages.extend(context)
        
        # Add the current user message
        messages.append({
            "role": "user",
            "content": prompt
        })
        
        # Prepare the payload
        payload = {
            "model": model,
            "messages": messages,
            "max_tokens": max_tokens,
            "temperature": temperature,
            "stream": stream
        }
        
        # Make the API request
        try:
            response = await self.client.post(
                f"{self.base_url}/chat/completions",
                json=payload
            )
            
            if response.status_code != 200:
                raise Exception(f"OpenRouter API error: {response.status_code} - {response.text}")
            
            result = response.json()
            
            # Extract the content from the response
            content = result["choices"][0]["message"]["content"]
            
            return {
                "content": content,
                "model": result.get("model", model),
                "usage": result.get("usage", {}),
                "id": result.get("id", "")
            }
        except Exception as e:
            raise Exception(f"Error calling OpenRouter API: {str(e)}")
    
    async def generate_response_with_context(
        self,
        query: str,
        context_chunks: List[str],
        system_prompt: Optional[str] = None,
        max_tokens: int = 1000,
        temperature: float = 0.7
    ) -> Dict[str, Any]:
        """
        Generate a response using provided context chunks.
        This method formats the context for the RAG system.
        
        Args:
            query: The user's question
            context_chunks: List of relevant text chunks used as context
            system_prompt: Optional system prompt to guide the model
            max_tokens: Maximum number of tokens to generate
            temperature: Sampling temperature
            
        Returns:
            Dict containing the generated response
        """
        # Combine context chunks into a single context string
        combined_context = "\n\n".join([f"Context {i+1}: {chunk}" for i, chunk in enumerate(context_chunks)])
        
        # Create a system prompt that emphasizes using only the provided context
        if not system_prompt:
            system_prompt = (
                "You are a helpful assistant that answers questions based only on the provided context. "
                "Do not use any prior knowledge or information not present in the context. "
                "If the context doesn't contain information to answer the question, say so explicitly. "
                "Always cite which context items you used to form your answer. "
                "Respond with accuracy and precision."
            )
        
        # Create the full prompt with context
        full_prompt = (
            f"Context Information:\n{combined_context}\n\n"
            f"Question: {query}\n\n"
            f"Please provide an answer based ONLY on the context information provided above. "
            f"Do not use any external knowledge."
        )
        
        # Generate response using the OpenRouter API
        return await self.generate_response(
            prompt=full_prompt,
            system_prompt=system_prompt,
            max_tokens=max_tokens,
            temperature=temperature
        )
    
    async def close(self):
        """
        Close the HTTP client.
        """
        await self.client.aclose()


# Global instance for the application
openrouter_client = OpenRouterClient()