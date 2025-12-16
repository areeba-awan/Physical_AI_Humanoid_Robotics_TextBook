# Research: RAG Chatbot Integration

## Book Ingestion Flow

### Decision:
Design a multi-step book ingestion pipeline that converts book content into chunks suitable for RAG.

### Rationale:
The ingestion flow needs to convert book content (likely PDF, EPUB, or text) into a format that can be chunked, embedded, and stored in the vector database.

### Alternatives considered:
- Real-time parsing during queries: Would be too slow
- Manual chunking: Inefficient and inconsistent
- Pre-computed static chunks: Less flexible for updates

## Chunking and Embedding Strategy

### Decision:
Use semantic chunking with overlap to maintain context while optimizing retrieval.

### Rationale:
Semantic chunking maintains the meaning of content across chunk boundaries, and overlap ensures that context isn't lost at chunk edges. This will optimize retrieval quality.

### Alternatives considered:
- Fixed-size character chunking: Would break sentences and paragraphs
- Sentence-level chunking: Might be too granular and lose context
- Recursive chunking: More complex, with potentially suboptimal boundaries

## RAG Retrieval Pipeline

### Decision:
Implement a multi-stage retrieval pipeline with re-ranking for optimal results.

### Rationale:
This pipeline will first retrieve candidate chunks using vector similarity, then use a re-ranking model to improve relevance before sending to the LLM.

### Alternatives considered:
- Simple vector similarity retrieval: Less precise
- Keyword-based retrieval: Doesn't leverage semantic meaning
- Hybrid keyword/vector retrieval: More complex to tune

## Selected-Text Query Handling

### Decision:
Implement a mechanism to filter retrieved context to only include selected text.

### Rationale:
This ensures compliance with the requirement that answers for selected text queries only reference that specific content.

### Alternatives considered:
- Post-processing responses: Risky due to potential hallucinations
- Separate index for selected text: Overly complex
- Client-side filtering: Doesn't prevent hallucination during generation

## Chat Memory Handling

### Decision:
Store conversation history in Neon Postgres with automatic summarization for long sessions.

### Rationale:
This allows for persistent memory across sessions while preventing context window overflow.

### Alternatives considered:
- In-memory storage: Doesn't persist across sessions
- Client-side storage: Less reliable and secure
- Full session in LLM context: Would exceed token limits

## Frontend Embedding Approach

### Decision:
Integrate the chat interface as a component overlay on the existing book reader.

### Rationale:
This maintains the existing book reading experience while adding the RAG functionality.

### Alternatives considered:
- Separate interface: Poor user experience
- Modal/popup interface: Works well with existing layout
- Full-page replacement: Changes the core reading experience

## Technology Stack Research

### OpenRouter Integration

**Decision**: Use OpenRouter API for LLM calls to access powerful models.

**Rationale**: OpenRouter provides easy access to various state-of-the-art models with a simple API.

**Alternatives considered**: Self-hosted models, direct API calls to specific providers

### Qwen Embeddings

**Decision**: Use Qwen models for generating embeddings to ensure compatibility with the RAG framework.

**Rationale**: Consistent use of Qwen technology stack as specified in requirements.

**Alternatives considered**: OpenAI embeddings, sentence transformers, other embedding models

### Qdrant Vector Database

**Decision**: Use Qdrant Cloud for vector storage and similarity search.

**Rationale**: Qdrant provides efficient, scalable vector search capabilities required for RAG.

**Alternatives considered**: Pinecone, Weaviate, ChromaDB, custom Elasticsearch solution

### Neon Postgres Integration

**Decision**: Use Neon Serverless Postgres for metadata, session management, and query history.

**Rationale**: Serverless Postgres provides reliable persistence with auto-scaling capabilities.

**Alternatives considered**: MongoDB, other NoSQL databases, file-based storage