# Data Model: RAG Chatbot Integration

## Entities

### BookContent
- **id**: UUID - Unique identifier for the chunk
- **book_id**: UUID - Reference to the book this content belongs to
- **chunk_text**: Text - The actual text content of the chunk
- **chunk_metadata**: JSON - Additional metadata (page number, section, etc.)
- **embedding_vector**: Vector - Qwen-generated embedding vector
- **created_at**: DateTime - When the chunk was created
- **updated_at**: DateTime - When the chunk was last updated

### UserQuery
- **id**: UUID - Unique identifier for the query
- **session_id**: UUID - Reference to the chat session
- **query_text**: Text - The user's original question
- **query_metadata**: JSON - Additional metadata (source page, selected text, etc.)
- **created_at**: DateTime - When the query was made

### GeneratedResponse
- **id**: UUID - Unique identifier for the response
- **query_id**: UUID - Reference to the original query
- **response_text**: Text - The AI-generated answer
- **referenced_content**: JSON - Links to book content referenced in response
- **confidence_score**: Float - Confidence level of the response
- **created_at**: DateTime - When the response was generated

### Session
- **id**: UUID - Unique identifier for the session
- **user_id**: UUID - Reference to the user (optional for anonymous sessions)
- **book_id**: UUID - Reference to the book being read
- **session_metadata**: JSON - Additional session data
- **created_at**: DateTime - When the session started
- **updated_at**: DateTime - When the session was last active
- **expires_at**: DateTime - When the session expires

### ChatMessage
- **id**: UUID - Unique identifier for the message
- **session_id**: UUID - Reference to the chat session
- **message_type**: Enum (user|assistant|system) - Type of message
- **content**: Text - The message content
- **timestamp**: DateTime - When the message was created
- **metadata**: JSON - Additional metadata (token counts, etc.)

## Relationships

- **Session** 1 → * **UserQuery**: A session contains multiple user queries
- **UserQuery** 1 → 1 **GeneratedResponse**: Each query generates one response
- **GeneratedResponse** * → * **BookContent**: A response can reference multiple content chunks
- **Session** 1 → * **ChatMessage**: A session contains multiple chat messages
- **BookContent** 1 → * **GeneratedResponse**: Book content can be referenced by multiple responses

## Validation Rules

- BookContent.chunk_text must not exceed 2000 characters
- UserQuery.query_text must not be empty
- Session.expires_at must be at least 30 minutes after created_at
- GeneratedResponse must reference at least one BookContent item
- BookContent.embedding_vector must have a fixed dimension matching the embedding model

## State Transitions

- Session: active → expired (after timeout or explicit end)
- UserQuery: pending → processed → completed (with response)