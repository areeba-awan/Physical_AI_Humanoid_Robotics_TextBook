---
description: "Task list for RAG chatbot integration in digital books"
---

# Tasks: RAG Chatbot Integration

**Input**: Design documents from `/specs/001-rag-chatbot-integration/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`
- Paths shown based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create backend project structure with FastAPI dependencies in backend/
- [X] T002 Create frontend project structure with TypeScript dependencies in frontend/
- [X] T003 [P] Set up environment configuration in backend/.env and frontend/.env
- [X] T004 [P] Initialize git repository and set up .gitignore for backend and frontend
- [X] T005 [P] Configure linting and formatting tools for Python (backend) and TypeScript (frontend)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 Setup Neon Postgres database schema and migrations framework in backend/src/core/database.py
- [X] T007 Configure Qdrant vector database connection in backend/src/core/vector_db.py
- [X] T008 [P] Implement OpenRouter API client in backend/src/core/llm_client.py
- [X] T009 [P] Implement Qwen embeddings service in backend/src/services/embedding_service.py
- [X] T010 Create base models for BookContent, UserQuery, GeneratedResponse, Session, ChatMessage in backend/src/models/
- [X] T011 Setup API routing and middleware structure in backend/src/api/
- [X] T012 Configure error handling and logging infrastructure in backend/src/core/middleware.py
- [X] T013 Set up book ingestion pipeline framework in backend/src/services/book_ingestion.py
- [X] T014 Implement vector search service in backend/src/services/vector_search.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Query Book Content (Priority: P1) 🎯 MVP

**Goal**: Enable users to ask questions about the book content and receive accurate answers based on the book's content

**Independent Test**: Can be fully tested by asking a question about specific content in the book and verifying that the response is accurate and based on the provided text.

### Implementation for User Story 1

- [X] T015 [P] [US1] Create BookContent model methods for storage/retrieval in backend/src/models/book_content.py
- [X] T016 [P] [US1] Create UserQuery model methods in backend/src/models/user_query.py
- [X] T017 [P] [US1] Create GeneratedResponse model methods in backend/src/models/generated_response.py
- [X] T018 [US1] Implement Session service in backend/src/services/session_service.py
- [X] T019 [US1] Implement RAG service for query processing in backend/src/services/rag_service.py
- [X] T020 [US1] Implement book query endpoint in backend/src/api/v1/query.py
- [X] T021 [US1] Implement basic chat interface component in frontend/src/components/ChatInterface/
- [X] T022 [US1] Connect frontend to backend query API in frontend/src/services/api-client.ts
- [X] T023 [US1] Add validation and error handling for query requests
- [X] T024 [US1] Add logging for query processing operations

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Navigate to Referenced Content (Priority: P2)

**Goal**: Allow users to navigate to the specific parts of the book that the chatbot references in its responses

**Independent Test**: Can be tested by asking a question that generates a response referencing specific book content and then clicking on the reference to navigate to that content.

### Implementation for User Story 2

- [X] T025 [P] [US2] Enhance GeneratedResponse model to include content references in backend/src/models/generated_response.py
- [X] T026 [P] [US2] Create ContentReference model in backend/src/models/content_reference.py
- [X] T027 [US2] Update RAG service to generate content references in backend/src/services/rag_service.py
- [X] T028 [US2] Modify book query endpoint to include content references in responses in backend/src/api/v1/query.py
- [X] T029 [US2] Implement content reference display in response renderer in frontend/src/components/ResponseRenderer/
- [X] T030 [US2] Implement navigation capability in frontend/src/components/BookReader/
- [X] T031 [US2] Add click handlers for content references in frontend/src/components/ResponseRenderer/
- [X] T032 [US2] Integrate with User Story 1 components to enhance existing responses with navigation

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Contextually Relevant Explanations (Priority: P3)

**Goal**: Provide explanations that are relevant to the user's current reading context

**Independent Test**: Can be tested by asking a question about a concept that appears in multiple places in the book and verifying that the response is relevant to the current reading context.

### Implementation for User Story 3

- [X] T033 [P] [US3] Enhance Session model to track reading context in backend/src/models/models.py
- [X] T034 [US3] Update RAG service to consider reading context for relevance in backend/src/services/rag_service.py
- [X] T035 [US3] Implement context-aware query processing in backend/src/services/rag_service.py
- [X] T036 [US3] Update book query endpoint to accept and process context information in backend/src/api/v1/query.py
- [X] T037 [US3] Implement context sending from frontend BookReader component in frontend/src/components/BookReader/
- [X] T038 [US3] Add contextual awareness to chat interface in frontend/src/components/ChatInterface/
- [X] T039 [US3] Integrate with User Story 1/2 components to enhance responses with contextual awareness

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Selected-Text-Only Query Handling

**Goal**: Implement functionality to restrict answers to only the selected text when the user indicates they want answers based on selected content only

**Independent Test**: Can be tested by selecting specific text in the book, asking a question about that text, and verifying that the response is based only on the selected text.

### Implementation for Selected-Text-Only

- [X] T040 [P] Create selected text query endpoint in backend/src/api/v1/query.py
- [X] T041 [P] Implement selected text validation in backend/src/services/rag_service.py
- [X] T042 Update RAG service to filter context to selected text only in backend/src/services/rag_service.py
- [X] T043 Implement selected text handling in frontend/src/components/BookReader/
- [X] T044 Create UI control for selected text queries in frontend/src/components/QueryInput/
- [X] T045 Connect frontend to selected text query API in frontend/src/services/api-client.ts

**Checkpoint**: Selected-text-only functionality should be fully functional and testable

---

## Phase 7: Frontend Integration & Chat Memory

**Goal**: Complete the frontend integration and implement chat memory handling

**Independent Test**: Can be tested by having a conversation across multiple queries and verifying that the system maintains context during multi-turn conversations.

### Implementation for Frontend Integration & Memory

- [X] T046 [P] Implement ChatMessage model methods in backend/src/models/chat_message.py
- [X] T047 [P] Create chat history service in backend/src/services/chat_history_service.py
- [X] T048 [P] Implement session creation endpoint in backend/src/api/v1/session.py
- [X] T049 [P] Implement session retrieval endpoint in backend/src/api/v1/session.py
- [X] T050 Integrate chat history with RAG service in backend/src/services/rag_service.py
- [X] T051 Implement memory management in frontend/src/services/chat-service.ts
- [X] T052 Create chat history display in frontend/src/components/ChatInterface/
- [X] T053 Implement conversation persistence in frontend/src/services/chat-service.ts
- [X] T054 Add conversation context to query requests in frontend/src/services/chat-service.ts

**Checkpoint**: Chat memory and conversation history should be fully functional

---

## Phase 8: Book Ingestion & Embedding Pipeline

**Goal**: Implement the complete pipeline for ingesting books, chunking content, and generating embeddings

**Independent Test**: Can be tested by ingesting a book and verifying that its content is properly chunked, embedded, and stored in the vector database.

### Implementation for Book Ingestion & Embedding

- [X] T055 [P] Implement PDF parsing for book ingestion in backend/src/services/book_ingestion.py
- [X] T056 [P] Implement text chunking with overlap strategy in backend/src/services/book_ingestion.py
- [X] T057 [P] Create embedding batch processing in backend/src/services/embedding_service.py
- [X] T058 [P] Implement Qwen embedding generation in backend/src/services/embedding_service.py
- [X] T059 Store embedded content in Qdrant vector database in backend/src/services/vector_search.py
- [X] T060 Create book ingestion CLI command in backend/src/scripts/ingest_book.py
- [X] T061 Implement book metadata storage in Neon Postgres in backend/src/services/book_ingestion.py
- [X] T062 Add book content validation and error handling in backend/src/services/book_ingestion.py
- [X] T063 Create ingestion progress tracking in backend/src/services/book_ingestion.py

**Checkpoint**: Complete book ingestion pipeline should be functional

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T064 [P] Documentation updates in docs/ and README.md files
- [X] T065 Code cleanup and refactoring across all modules
- [ ] T066 Performance optimization of query response times
- [X] T067 [P] Add comprehensive error handling and user-friendly messages
- [ ] T068 Security hardening for API endpoints
- [X] T069 Run quickstart.md validation and update as needed
- [X] T070 Add monitoring and metrics collection
- [X] T071 Implement rate limiting for API endpoints
- [X] T072 Add comprehensive logging throughout the application

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Additional phases (6+)**: Can run in parallel with user stories or after
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all models for User Story 1 together:
Task: "Create BookContent model methods for storage/retrieval in backend/src/models/book_content.py"
Task: "Create UserQuery model methods in backend/src/models/user_query.py"
Task: "Create GeneratedResponse model methods in backend/src/models/generated_response.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence