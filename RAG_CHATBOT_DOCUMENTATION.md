# RAG Chatbot - Complete Documentation

## Why the Chatbot Was Failing (Root Cause Analysis)

### Issue 1: RAG Agent Not Integrated
**File:** `agents/rag_chat_agent.py:101-123`
**Problem:** The `execute()` method returned placeholder responses instead of calling the actual RAG service.
**Fix:** Rewrote chat service to properly integrate with RAG retrieval.

### Issue 2: Score Threshold Too High
**File:** `backend/app/services/rag_service.py:28`
**Problem:** `DEFAULT_SCORE_THRESHOLD = 0.5` was too aggressive. Semantic similarity between questions and answers often scores 0.35-0.5.
**Fix:** Lowered to `0.35` with soft minimum results guarantee.

### Issue 3: Selected Text Relevance Check Too Strict
**File:** `backend/app/services/rag_service.py:211`
**Problem:** Simple `similarity > 0.3 and word_count >= 10` failed for conceptual questions.
**Fix:** Implemented multi-factor relevance scoring (semantic + keyword + length).

### Issue 4: No Anti-Hallucination Verification
**Problem:** System prompts suggested not hallucinating, but no programmatic check existed.
**Fix:** Added `verify_answer_in_context()` method using embedding similarity.

### Issue 5: Embedding Dimension Mismatch
**Problem:** Ingestion didn't specify dimensions, but retrieval did (3072).
**Fix:** All embedding calls now explicitly specify `dimensions=settings.openai_embedding_dimensions`.

### Issue 6: Chat History Not Session-Scoped
**Problem:** History fetched by `user_id` globally, causing context pollution.
**Fix:** Added 24-hour session window filter on chat history.

---

## RAG Flow Explanation

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         RAG CHATBOT ARCHITECTURE                         │
└─────────────────────────────────────────────────────────────────────────┘

                              INGESTION FLOW
┌─────────────┐     ┌──────────────┐     ┌─────────────┐     ┌──────────┐
│  Markdown   │────▶│   Chunking   │────▶│  Embedding  │────▶│  Qdrant  │
│    Files    │     │  (1500 char) │     │ text-embed- │     │  Vector  │
│             │     │  + metadata  │     │ 3-large     │     │    DB    │
└─────────────┘     └──────────────┘     └─────────────┘     └──────────┘

                               QUERY FLOW
┌─────────────┐     ┌──────────────┐     ┌─────────────┐     ┌──────────┐
│    User     │────▶│   Embed      │────▶│   Qdrant    │────▶│  Context │
│   Query     │     │   Query      │     │   Search    │     │  Builder │
└─────────────┘     └──────────────┘     └─────────────┘     └──────────┘
                                                                   │
                                                                   ▼
┌─────────────┐     ┌──────────────┐     ┌─────────────┐     ┌──────────┐
│  Response   │◀────│  Grounding   │◀────│   OpenAI    │◀────│  System  │
│  + Sources  │     │  Verification│     │   GPT-4o    │     │  Prompt  │
└─────────────┘     └──────────────┘     └─────────────┘     └──────────┘
```

### Step-by-Step Flow:

#### 1. Ingestion (One-time Setup)
```python
# 1. Load markdown files from book directory
# 2. Extract sections by headers (##, ###)
# 3. Chunk text intelligently (1500 chars, 200 overlap)
# 4. Generate embeddings with text-embedding-3-large (3072 dims)
# 5. Store in Qdrant with metadata (chapter, section, page)
```

#### 2. Global Book Query
```python
# 1. User asks: "What is ROS 2?"
# 2. Embed query → 3072-dim vector
# 3. Search Qdrant (cosine similarity, threshold 0.35)
# 4. Retrieve top 10 relevant chunks
# 5. Build context string with citations
# 6. Send to GPT-4o with strict system prompt
# 7. Verify answer is grounded in context
# 8. Return answer + sources
```

#### 3. Selected Text Query (STRICT Mode)
```python
# 1. User selects text + asks question
# 2. Analyze text relevance (semantic + keyword)
# 3. If insufficient → return "not enough information"
# 4. If sufficient → send to GPT-4o with STRICT prompt
# 5. Return answer (NO external sources)
```

---

## Database Schemas

### Neon Serverless Postgres

```sql
-- Chat Messages Table
CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL,  -- 'user' or 'assistant'
    content TEXT NOT NULL,
    sources JSONB,  -- Array of citation objects
    context_chapter VARCHAR(100),
    selected_text TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_chat_messages_user_id ON chat_messages(user_id);
CREATE INDEX idx_chat_messages_created_at ON chat_messages(created_at);

-- Users Table (simplified)
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    hashed_password VARCHAR(255) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

### Qdrant Vector Database Schema

```python
# Collection: physicalai_textbook
{
    "vectors_config": {
        "size": 3072,  # text-embedding-3-large dimensions
        "distance": "Cosine"
    },
    "payload_schema": {
        "content": "text",           # The actual chunk text
        "chapter_id": "keyword",     # e.g., "chapter-1-2"
        "module_id": "keyword",      # e.g., "module-1"
        "section": "text",           # e.g., "Introduction to ROS 2"
        "page_number": "integer",
        "chunk_index": "integer",
        "source_file": "text",
        "word_count": "integer"
    }
}
```

### Citation Object Schema

```typescript
interface Citation {
    chapter_id: string;      // "chapter-1-2"
    section: string;         // "Introduction to ROS 2"
    relevance: number;       // 0.0 - 1.0
    snippet?: string;        // First 200 chars of content
}
```

---

## API Endpoints

### POST /api/v1/ingest
Ingest raw text content into the RAG system.

**Request:**
```json
{
    "content": "Full text content to ingest...",
    "chapter_id": "chapter-1-1",
    "module_id": "module-1",
    "section": "Introduction"
}
```

**Response:**
```json
{
    "success": true,
    "chunks_created": 15,
    "message": "Successfully ingested content into chapter-1-1",
    "details": {
        "chapter_id": "chapter-1-1",
        "module_id": "module-1",
        "status": "success"
    }
}
```

### POST /api/v1/chat
Global book query (RAG-powered).

**Request:**
```json
{
    "message": "What is ROS 2 and how does it work?",
    "context_chapter": null  // Optional: filter by chapter
}
```

**Response:**
```json
{
    "id": "uuid",
    "role": "assistant",
    "content": "According to Chapter 1 - Introduction to ROS 2...",
    "sources": [
        {
            "chapter_id": "chapter-1-1",
            "section": "What is ROS 2",
            "relevance": 0.87,
            "snippet": "ROS 2 (Robot Operating System 2) is..."
        }
    ],
    "timestamp": "2025-01-15T10:30:00Z"
}
```

### POST /api/v1/chat/selected-text
Selected text query (STRICT mode).

**Request:**
```json
{
    "message": "Explain this code",
    "selected_text": "def create_publisher(self):\n    self.publisher = self.create_publisher(String, 'topic', 10)",
    "context_chapter": "chapter-1-2"
}
```

**Response:**
```json
{
    "id": "uuid",
    "role": "assistant",
    "content": "This code creates a ROS 2 publisher...",
    "sources": [],
    "timestamp": "2025-01-15T10:31:00Z"
}
```

### POST /api/v1/chat/stream
Streaming global query (SSE).

**Request:** Same as `/chat`

**Response (Server-Sent Events):**
```
data: {"type": "content", "content": "According to"}

data: {"type": "content", "content": " the textbook"}

data: {"type": "done", "sources": [...], "full_content": "..."}
```

### GET /health
Comprehensive health check.

**Response:**
```json
{
    "status": "healthy",
    "version": "1.0.0",
    "services": {
        "qdrant": {
            "status": "healthy",
            "vectors_count": 1250,
            "collection_status": "green"
        },
        "openai": {
            "status": "configured",
            "model": "gpt-4o",
            "embedding_model": "text-embedding-3-large"
        },
        "database": {
            "status": "configured"
        }
    }
}
```

---

## Frontend Integration Example

### React Hook for Chat

```typescript
// hooks/useRAGChat.ts
import { useState, useCallback } from 'react';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Citation[];
  isStreaming?: boolean;
}

interface Citation {
  chapter_id: string;
  section: string;
  relevance: number;
  snippet?: string;
}

export function useRAGChat() {
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);

  const sendMessage = useCallback(async (
    message: string,
    selectedText?: string,
    contextChapter?: string
  ) => {
    setIsLoading(true);

    // Add user message
    const userMessage: Message = {
      id: crypto.randomUUID(),
      role: 'user',
      content: message,
    };
    setMessages(prev => [...prev, userMessage]);

    // Determine endpoint based on selected text
    const endpoint = selectedText
      ? '/api/v1/chat/selected-text/stream'
      : '/api/v1/chat/stream';

    const body = selectedText
      ? { message, selected_text: selectedText, context_chapter: contextChapter }
      : { message, context_chapter: contextChapter };

    try {
      const response = await fetch(endpoint, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${getToken()}`,
        },
        body: JSON.stringify(body),
      });

      if (!response.ok) throw new Error('Chat request failed');

      // Add placeholder for assistant message
      const assistantId = crypto.randomUUID();
      setMessages(prev => [...prev, {
        id: assistantId,
        role: 'assistant',
        content: '',
        isStreaming: true,
      }]);

      // Process SSE stream
      const reader = response.body?.getReader();
      const decoder = new TextDecoder();
      let fullContent = '';
      let sources: Citation[] = [];

      while (reader) {
        const { done, value } = await reader.read();
        if (done) break;

        const chunk = decoder.decode(value);
        const lines = chunk.split('\n').filter(line => line.startsWith('data: '));

        for (const line of lines) {
          const data = JSON.parse(line.slice(6));

          if (data.type === 'content') {
            fullContent += data.content;
            setMessages(prev => prev.map(msg =>
              msg.id === assistantId
                ? { ...msg, content: fullContent }
                : msg
            ));
          } else if (data.type === 'done') {
            sources = data.sources || [];
            setMessages(prev => prev.map(msg =>
              msg.id === assistantId
                ? { ...msg, content: fullContent, sources, isStreaming: false }
                : msg
            ));
          } else if (data.type === 'error') {
            throw new Error(data.error);
          }
        }
      }
    } catch (error) {
      console.error('Chat error:', error);
      setMessages(prev => [...prev, {
        id: crypto.randomUUID(),
        role: 'assistant',
        content: 'Sorry, an error occurred. Please try again.',
      }]);
    } finally {
      setIsLoading(false);
    }
  }, []);

  const clearHistory = useCallback(async () => {
    await fetch('/api/v1/chat/history', {
      method: 'DELETE',
      headers: { 'Authorization': `Bearer ${getToken()}` },
    });
    setMessages([]);
  }, []);

  return { messages, isLoading, sendMessage, clearHistory };
}
```

### Chat Widget Component

```tsx
// components/ChatWidget.tsx
import React, { useState, useRef, useEffect } from 'react';
import { useRAGChat } from '../hooks/useRAGChat';
import ReactMarkdown from 'react-markdown';

interface ChatWidgetProps {
  selectedText?: string;
  contextChapter?: string;
}

export function ChatWidget({ selectedText, contextChapter }: ChatWidgetProps) {
  const { messages, isLoading, sendMessage, clearHistory } = useRAGChat();
  const [input, setInput] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (!input.trim() || isLoading) return;

    sendMessage(input, selectedText, contextChapter);
    setInput('');
  };

  return (
    <div className="chat-widget">
      {/* Header */}
      <div className="chat-header">
        <h3>AI Assistant</h3>
        {selectedText && (
          <span className="selected-text-badge">
            Answering from selected text
          </span>
        )}
        <button onClick={clearHistory}>Clear</button>
      </div>

      {/* Messages */}
      <div className="chat-messages">
        {messages.map((msg) => (
          <div key={msg.id} className={`message ${msg.role}`}>
            <ReactMarkdown>{msg.content}</ReactMarkdown>

            {msg.isStreaming && <span className="typing-indicator">●●●</span>}

            {msg.sources && msg.sources.length > 0 && (
              <div className="sources">
                <strong>Sources:</strong>
                {msg.sources.map((source, i) => (
                  <a key={i} href={`#${source.chapter_id}`}>
                    {source.chapter_id} - {source.section}
                  </a>
                ))}
              </div>
            )}
          </div>
        ))}
        <div ref={messagesEndRef} />
      </div>

      {/* Input */}
      <form onSubmit={handleSubmit} className="chat-input">
        <input
          value={input}
          onChange={(e) => setInput(e.target.value)}
          placeholder={selectedText
            ? "Ask about the selected text..."
            : "Ask about the textbook..."}
          disabled={isLoading}
        />
        <button type="submit" disabled={isLoading || !input.trim()}>
          Send
        </button>
      </form>
    </div>
  );
}
```

### Text Selection Handler

```tsx
// hooks/useTextSelection.ts
import { useState, useEffect, useCallback } from 'react';

export function useTextSelection(minLength = 10) {
  const [selectedText, setSelectedText] = useState<string | null>(null);

  const handleSelection = useCallback(() => {
    const selection = window.getSelection();
    const text = selection?.toString().trim();

    if (text && text.length >= minLength) {
      setSelectedText(text);
    } else {
      setSelectedText(null);
    }
  }, [minLength]);

  useEffect(() => {
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, [handleSelection]);

  const clearSelection = useCallback(() => {
    setSelectedText(null);
    window.getSelection()?.removeAllRanges();
  }, []);

  return { selectedText, clearSelection };
}
```

---

## Environment Configuration

```bash
# .env file
# Application
APP_NAME=PhysicalAI-Textbook
APP_VERSION=1.0.0
DEBUG=false

# Database (Neon Serverless)
DATABASE_URL=postgresql+asyncpg://user:password@ep-xxx.us-east-2.aws.neon.tech/physicalai

# OpenAI
OPENAI_API_KEY=sk-...
OPENAI_MODEL=gpt-4o
OPENAI_EMBEDDING_MODEL=text-embedding-3-large
OPENAI_EMBEDDING_DIMENSIONS=3072

# Qdrant Cloud
QDRANT_URL=https://xxx.us-east-1-0.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION=physicalai_textbook

# Authentication
SECRET_KEY=your-super-secret-key-change-in-production
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=15

# CORS
CORS_ORIGINS=["http://localhost:3000","https://your-frontend.com"]
```

---

## Running the Application

### Backend

```bash
cd backend

# Install dependencies
pip install -r requirements.txt

# Run with uvicorn
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

### Ingest Book Content

```bash
# Via API
curl -X POST "http://localhost:8000/api/v1/ingest/directory" \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"directory_path": "/path/to/book/markdown"}'
```

### Frontend

```bash
cd frontend

# Install dependencies
npm install

# Run development server
npm run dev
```

---

## Key Improvements Made

1. **Lower Score Threshold:** 0.5 → 0.35 for better recall
2. **Multi-Factor Relevance:** Semantic + keyword + length scoring
3. **Session-Based History:** 24-hour window prevents context pollution
4. **Explicit Embedding Dimensions:** Prevents dimension mismatch
5. **Grounding Verification:** Post-response check for hallucination
6. **Smart Chunking:** Paragraph/sentence boundaries, code block preservation
7. **Strict System Prompts:** Explicit rules for no-hallucination policy
8. **Error Handling:** Comprehensive logging and graceful failures
