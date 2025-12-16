# Implementation Plan: RAG Chatbot Integration

**Branch**: `001-rag-chatbot-integration` | **Date**: December 15, 2025 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-chatbot-integration/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Integrate a RAG (Retrieval Augmented Generation) chatbot into an existing published digital book. The system will use OpenRouter for LLM generation, Qwen for embeddings, Qdrant for vector search, and Neon Postgres for memory and metadata. The implementation includes book ingestion, chunking and embedding, RAG retrieval pipeline, selected-text query handling, chat memory management, and frontend embedding.

## Technical Context

**Language/Version**: Python 3.11, TypeScript 5.0
**Primary Dependencies**: FastAPI, OpenAPI, Qwen embeddings API, Qdrant vector database, Neon Postgres, OpenRouter API
**Storage**: Qdrant Cloud (vector DB), Neon Serverless Postgres (metadata and session storage)
**Testing**: pytest, integration tests for API contracts, end-to-end tests for RAG pipeline
**Target Platform**: Web application (frontend integration with existing book interface)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: Response time under 5 seconds for 90% of queries, support concurrent users during reading sessions
**Constraints**: Must prevent hallucinations, responses only from book content, selected-text-only answering capability
**Scale/Scope**: Support for one or more published books, handling user sessions and query history

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the project constitution, this implementation follows the core principles:
- Library-first approach: Core RAG functionality will be developed as reusable libraries
- CLI Interface: Backend services will expose functionality that can be accessed via CLI
- Test-First: All components will have comprehensive test coverage before implementation
- Integration Testing: Focus on testing the RAG pipeline, book ingestion, and LLM interaction
- Observability: Structured logging for debugging and monitoring the RAG system

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot-integration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

backend/
├── src/
│   ├── models/
│   │   ├── book_content.py
│   │   ├── user_query.py
│   │   ├── generated_response.py
│   │   └── session.py
│   ├── services/
│   │   ├── embedding_service.py
│   │   ├── rag_service.py
│   │   ├── book_ingestion.py
│   │   ├── vector_search.py
│   │   └── llm_service.py
│   ├── api/
│   │   ├── v1/
│   │   │   ├── book.py
│   │   │   ├── query.py
│   │   │   └── chat.py
│   │   └── dependencies.py
│   └── core/
│       ├── config.py
│       ├── database.py
│       └── middleware.py
└── tests/
    ├── unit/
    ├── integration/
    └── contract/

frontend/
├── src/
│   ├── components/
│   │   ├── BookReader/
│   │   ├── ChatInterface/
│   │   ├── QueryInput/
│   │   └── ResponseRenderer/
│   ├── services/
│   │   ├── api-client.ts
│   │   ├── book-content-service.ts
│   │   └── chat-service.ts
│   ├── types/
│   │   ├── book.ts
│   │   ├── query.ts
│   │   └── response.ts
│   └── utils/
│       ├── text-selection.ts
│       └── content-linking.ts
└── tests/
    ├── unit/
    └── integration/

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
