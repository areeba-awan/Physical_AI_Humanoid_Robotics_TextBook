# RAG Chatbot Fix Summary

## Why the Chatbot Was Failing

### Root Causes Identified

1. **No Ingestion Pipeline**
   - Vector database was empty - no book content was indexed
   - Missing document loader, chunking logic, and embedding generation
   - Qdrant collection existed but had zero vectors

2. **Wrong Embedding Model**
   - Config used `text-embedding-ada-002` (1536 dimensions)
   - Should use `text-embedding-3-large` (3072 dimensions)
   - Dimension mismatch would cause search failures

3. **RAG Agent Not Implemented**
   - `rag_chat_agent.py:113-123` returned hardcoded placeholder response
   - Never actually queried the vector database
   - Agent framework was defined but not integrated

4. **No Strict Context Mode**
   - Selected text queries searched the entire book
   - Should ONLY answer from user-selected text
   - Missing "insufficient information" response when text lacks answer

5. **No Streaming Support**
   - Synchronous responses only
   - Poor UX for longer responses

6. **No Session Memory**
   - Chat history stored but never used in prompts
   - Each query independent - no conversation continuity

7. **High Score Threshold**
   - `score_threshold: 0.7` too strict
   - Filtered out potentially relevant results

---

## Solution Implementation

### Files Created/Modified

#### New Services

**`backend/app/services/ingestion_service.py`** (NEW)
- Document loading from markdown files
- Text chunking with 500 tokens, 50 token overlap
- Metadata extraction (chapter, module, section)
- Batch embedding generation
- Qdrant vector storage

**`backend/app/services/rag_service.py`** (UPDATED)
- Uses `text-embedding-3-large` (3072 dimensions)
- Two query modes: Global and Selected Text
- Lower score threshold (0.5) for better recall
- Relevance analysis for selected text

**`backend/app/services/chat_service.py`** (UPDATED)
- Session-based chat memory (last 10 messages)
- Strict selected text mode with no-hallucination prompts
- Streaming response support (SSE)
- Clear "insufficient information" responses

#### New API Endpoints

**`backend/app/api/v1/ingest.py`** (NEW)
```
POST /api/v1/ingest           - Ingest raw text
POST /api/v1/ingest/directory - Ingest markdown directory
POST /api/v1/ingest/file      - Ingest uploaded file
GET  /api/v1/ingest/stats     - Collection statistics
DELETE /api/v1/ingest/collection - Delete collection
```

**`backend/app/api/v1/chat.py`** (UPDATED)
```
POST /api/v1/chat                    - Global book query
POST /api/v1/chat/selected-text      - Selected text query (STRICT)
POST /api/v1/chat/stream             - Streaming global query
POST /api/v1/chat/selected-text/stream - Streaming selected text
GET  /api/v1/chat/history            - Get chat history
DELETE /api/v1/chat/history          - Clear history
```

**`backend/app/main.py`** (UPDATED)
- Comprehensive health check with Qdrant/OpenAI status

#### Configuration

**`backend/app/config.py`** (UPDATED)
- Model: `gpt-4o`
- Embedding: `text-embedding-3-large`
- Dimensions: 3072

#### Schema Documentation

**`backend/schemas.sql`** (NEW)
- Neon Postgres tables: users, chat_messages, profiles, etc.

**`backend/qdrant_schema.json`** (NEW)
- Qdrant collection configuration
- Payload schema with examples

#### Frontend

**`frontend/lib/chat-api.ts`** (NEW)
- TypeScript API client
- Streaming support
- Both query modes

**`frontend/components/chat/ChatWidget.example.tsx`** (NEW)
- React component with streaming
- Selected text indicator
- Source citations display

---

## RAG Flow Explanation

### 1. Ingestion Flow
```
Book Content (.md files)
    ↓
Extract sections by headers (##, ###)
    ↓
Chunk text (500 tokens, 50 overlap)
    ↓
Generate embeddings (text-embedding-3-large)
    ↓
Store in Qdrant with metadata:
  - chapter_id, module_id, section
  - page_number, chunk_index
  - source_file
```

### 2. Global Book Query Flow
```
User Question
    ↓
Generate query embedding
    ↓
Search Qdrant (top 8, threshold 0.5)
    ↓
Build context with citations
    ↓
Add chat history (last 10 messages)
    ↓
GPT-4o generates response
    ↓
Stream to user with sources
```

### 3. Selected Text Query Flow (STRICT)
```
User Question + Selected Text
    ↓
Analyze text relevance (embedding similarity)
    ↓
If sufficient:
    → Answer ONLY from selected text
If insufficient:
    → Return "selected text does not contain enough information"
    ↓
Stream response to user
```

---

## API Usage Examples

### Ingest Book Content
```bash
# Ingest a directory of markdown files
curl -X POST "http://localhost:8000/api/v1/ingest/directory" \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"directory_path": "./book/docs"}'

# Ingest raw text
curl -X POST "http://localhost:8000/api/v1/ingest" \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "content": "ROS 2 is the next generation...",
    "chapter_id": "chapter-1-1",
    "module_id": "module-1",
    "section": "Introduction"
  }'
```

### Global Book Query
```bash
curl -X POST "http://localhost:8000/api/v1/chat" \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "message": "How do I create a ROS 2 node?",
    "context_chapter": "chapter-1-2"
  }'
```

### Selected Text Query (STRICT)
```bash
curl -X POST "http://localhost:8000/api/v1/chat/selected-text" \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "message": "Explain this code",
    "selected_text": "def create_node(): ..."
  }'
```

### Streaming Response
```javascript
const response = await fetch('/api/v1/chat/stream', {
  method: 'POST',
  headers: {
    'Authorization': `Bearer ${token}`,
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({ message: 'What is ROS 2?' })
});

const reader = response.body.getReader();
while (true) {
  const { done, value } = await reader.read();
  if (done) break;
  // Process SSE chunks
}
```

### Health Check
```bash
curl "http://localhost:8000/health"
# Returns:
# {
#   "status": "healthy",
#   "services": {
#     "qdrant": { "status": "healthy", "vectors_count": 1250 },
#     "openai": { "status": "configured", "model": "gpt-4o" },
#     "database": { "status": "configured" }
#   }
# }
```

---

## Key Design Decisions

### No Hallucinations Policy
- System prompts explicitly forbid making up information
- If context doesn't contain answer, model admits it
- Selected text mode is extra strict - only uses provided text

### Session Memory
- Last 10 messages included in context
- Truncated to 500 chars each to save tokens
- Enables follow-up questions

### Two Query Modes
1. **Global**: Searches entire book, returns citations
2. **Selected Text**: STRICT - only answers from selection

### Streaming
- Server-Sent Events (SSE) for real-time responses
- Chunks include type: 'content' or 'done'
- Final chunk includes sources

---

## Environment Variables Required

```env
# Database (Neon)
DATABASE_URL=postgresql+asyncpg://user:pass@host/db

# OpenAI
OPENAI_API_KEY=sk-...
OPENAI_MODEL=gpt-4o
OPENAI_EMBEDDING_MODEL=text-embedding-3-large

# Qdrant Cloud
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=...
QDRANT_COLLECTION=physicalai_textbook
```

---

## Quick Start

1. **Set environment variables** in `.env`

2. **Start the server**:
   ```bash
   cd backend
   uvicorn app.main:app --reload
   ```

3. **Ingest book content**:
   ```bash
   # Via API or script
   POST /api/v1/ingest/directory
   {"directory_path": "./book/docs"}
   ```

4. **Verify ingestion**:
   ```bash
   GET /api/v1/ingest/stats
   # Should show vectors_count > 0
   ```

5. **Test the chatbot**:
   ```bash
   POST /api/v1/chat
   {"message": "What is ROS 2?"}
   ```

---

## Summary

The chatbot was failing because:
1. No data was ingested into the vector database
2. Wrong embedding model configured
3. RAG pipeline not fully implemented

The fix includes:
1. Complete ingestion pipeline with chunking
2. Corrected embedding model (text-embedding-3-large)
3. Two query modes (Global + Strict Selected Text)
4. Streaming responses
5. Session-based memory
6. No-hallucination system prompts
7. Comprehensive health checks
