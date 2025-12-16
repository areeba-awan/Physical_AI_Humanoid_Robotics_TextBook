# SP.PLAN
## Physical AI & Humanoid Robotics - Implementation Plan

---

# PART 1: SYSTEM ARCHITECTURE

## 1.1 High-Level Architecture Diagram

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                              USER INTERFACE LAYER                              │
│                                                                                │
│  ┌────────────────────────┐     ┌────────────────────────┐                   │
│  │    Next.js Frontend    │     │   Docusaurus Book      │                   │
│  │    (Vercel)            │     │   (GitHub Pages)       │                   │
│  │                        │     │                        │                   │
│  │  ┌──────────────────┐  │     │  ┌──────────────────┐  │                   │
│  │  │ Auth Pages       │  │     │  │ Course Content   │  │                   │
│  │  │ • Signup         │  │     │  │ • 4 Modules      │  │                   │
│  │  │ • Signin         │  │     │  │ • 24 Chapters    │  │                   │
│  │  │ • Onboarding     │  │     │  │ • Labs/Quizzes   │  │                   │
│  │  └──────────────────┘  │     │  └──────────────────┘  │                   │
│  │                        │     │                        │                   │
│  │  ┌──────────────────┐  │     │  ┌──────────────────┐  │                   │
│  │  │ Dashboard        │  │     │  │ Interactive      │  │                   │
│  │  │ • Progress       │  │     │  │ • Personalize    │  │                   │
│  │  │ • Profile        │  │     │  │ • Translate      │  │                   │
│  │  │ • Settings       │  │     │  │ • Chat Widget    │  │                   │
│  │  └──────────────────┘  │     │  └──────────────────┘  │                   │
│  └────────────────────────┘     └────────────────────────┘                   │
│              │                              │                                 │
└──────────────┼──────────────────────────────┼─────────────────────────────────┘
               │                              │
               └──────────────┬───────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                              API GATEWAY LAYER                                 │
│                                                                                │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │                     FastAPI Backend (Railway/Fly.io)                     │ │
│  │                                                                          │ │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐       │ │
│  │  │ Auth API    │ │ Profile API │ │ Content API │ │ Chat API    │       │ │
│  │  │             │ │             │ │             │ │             │       │ │
│  │  │ /signup     │ │ /profile    │ │ /chapters   │ │ /chat       │       │ │
│  │  │ /signin     │ │ /background │ │ /personalize│ │ /context    │       │ │
│  │  │ /session    │ │ /progress   │ │ /translate  │ │ /history    │       │ │
│  │  │ /signout    │ │ /settings   │ │ /summary    │ │             │       │ │
│  │  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘       │ │
│  │                                                                          │ │
│  │  ┌─────────────────────────────────────────────────────────────────┐   │ │
│  │  │                        Agent API                                 │   │ │
│  │  │                                                                  │   │ │
│  │  │  POST /agents/summarize    POST /agents/translate               │   │ │
│  │  │  POST /agents/personalize  POST /agents/generate                │   │ │
│  │  └─────────────────────────────────────────────────────────────────┘   │ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
│                                    │                                          │
└────────────────────────────────────┼──────────────────────────────────────────┘
                                     │
                                     ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                           INTELLIGENCE LAYER                                   │
│                                                                                │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │                    Claude Code Subagents Framework                       │ │
│  │                                                                          │ │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐         │ │
│  │  │  RAGChatAgent   │  │ SummarizerAgent │  │ TranslatorAgent │         │ │
│  │  │                 │  │                 │  │                 │         │ │
│  │  │  • Query RAG    │  │  • Extract key  │  │  • EN → UR      │         │ │
│  │  │  • Context chat │  │  • Gen summary  │  │  • Preserve     │         │ │
│  │  │  • Citations    │  │  • Objectives   │  │    terms        │         │ │
│  │  └─────────────────┘  └─────────────────┘  └─────────────────┘         │ │
│  │                                                                          │ │
│  │  ┌─────────────────┐  ┌─────────────────┐                              │ │
│  │  │ PersonalizerAgent│  │ ContentGenAgent │                              │ │
│  │  │                 │  │                 │                              │ │
│  │  │  • User profile │  │  • Quizzes      │                              │ │
│  │  │  • Adapt content│  │  • Exercises    │                              │ │
│  │  │  • Examples     │  │  • Code samples │                              │ │
│  │  └─────────────────┘  └─────────────────┘                              │ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
│                                    │                                          │
└────────────────────────────────────┼──────────────────────────────────────────┘
                                     │
                                     ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                              DATA LAYER                                        │
│                                                                                │
│  ┌────────────────────┐  ┌────────────────────┐  ┌────────────────────┐      │
│  │   Neon Postgres    │  │   Qdrant Cloud     │  │   OpenAI API       │      │
│  │                    │  │                    │  │                    │      │
│  │  • Users           │  │  • Document        │  │  • GPT-4 Turbo    │      │
│  │  • Profiles        │  │    Embeddings      │  │  • Ada-002        │      │
│  │  • Sessions        │  │  • 1536-dim        │  │    Embeddings     │      │
│  │  • Chat History    │  │  • Cosine search   │  │  • Completions    │      │
│  │  • Progress        │  │                    │  │                    │      │
│  │  • Content Cache   │  │                    │  │                    │      │
│  └────────────────────┘  └────────────────────┘  └────────────────────┘      │
│                                                                                │
└──────────────────────────────────────────────────────────────────────────────┘
```

## 1.2 Data Flow Diagrams

### 1.2.1 Authentication Flow
```
┌───────────┐         ┌───────────┐         ┌───────────┐         ┌───────────┐
│   User    │         │  Next.js  │         │  FastAPI  │         │   Neon    │
└─────┬─────┘         └─────┬─────┘         └─────┬─────┘         └─────┬─────┘
      │                     │                     │                     │
      │  1. Submit signup   │                     │                     │
      │────────────────────>│                     │                     │
      │                     │  2. POST /signup    │                     │
      │                     │────────────────────>│                     │
      │                     │                     │  3. Create user     │
      │                     │                     │────────────────────>│
      │                     │                     │                     │
      │                     │                     │  4. Return user     │
      │                     │                     │<────────────────────│
      │                     │  5. JWT + Session   │                     │
      │                     │<────────────────────│                     │
      │  6. Redirect to     │                     │                     │
      │     onboarding      │                     │                     │
      │<────────────────────│                     │                     │
      │                     │                     │                     │
      │  7. Submit          │                     │                     │
      │     background      │                     │                     │
      │────────────────────>│                     │                     │
      │                     │  8. PUT /profile    │                     │
      │                     │────────────────────>│                     │
      │                     │                     │  9. Update profile  │
      │                     │                     │────────────────────>│
      │                     │                     │                     │
      │  10. Redirect to    │                     │                     │
      │      dashboard      │                     │                     │
      │<────────────────────│                     │                     │
```

### 1.2.2 RAG Chat Flow
```
┌───────────┐    ┌───────────┐    ┌───────────┐    ┌───────────┐    ┌───────────┐
│   User    │    │  Chat     │    │  FastAPI  │    │  Qdrant   │    │  OpenAI   │
│           │    │  Widget   │    │           │    │           │    │           │
└─────┬─────┘    └─────┬─────┘    └─────┬─────┘    └─────┬─────┘    └─────┬─────┘
      │                │                │                │                │
      │ 1. Ask "What   │                │                │                │
      │    is ROS 2?"  │                │                │                │
      │───────────────>│                │                │                │
      │                │ 2. POST /chat  │                │                │
      │                │───────────────>│                │                │
      │                │                │ 3. Generate    │                │
      │                │                │    embedding   │                │
      │                │                │───────────────────────────────>│
      │                │                │                │                │
      │                │                │ 4. Query       │ 5. Return top  │
      │                │                │    vectors     │    chunks      │
      │                │                │───────────────>│───────────────>│
      │                │                │                │                │
      │                │                │ 6. Generate response with      │
      │                │                │    context from chunks         │
      │                │                │───────────────────────────────>│
      │                │                │                │                │
      │                │                │ 7. Return      │                │
      │                │                │    response    │                │
      │                │                │<───────────────────────────────│
      │                │ 8. Display     │                │                │
      │                │    with cites  │                │                │
      │                │<───────────────│                │                │
      │ 9. Show answer │                │                │                │
      │<───────────────│                │                │                │
```

### 1.2.3 Personalization Flow
```
┌───────────┐    ┌───────────┐    ┌───────────┐    ┌───────────┐    ┌───────────┐
│   User    │    │ Docusaurus│    │  FastAPI  │    │  Neon DB  │    │  OpenAI   │
└─────┬─────┘    └─────┬─────┘    └─────┬─────┘    └─────┬─────┘    └─────┬─────┘
      │                │                │                │                │
      │ 1. Click       │                │                │                │
      │    Personalize │                │                │                │
      │───────────────>│                │                │                │
      │                │ 2. POST        │                │                │
      │                │    /personalize│                │                │
      │                │───────────────>│                │                │
      │                │                │ 3. Get user    │                │
      │                │                │    profile     │                │
      │                │                │───────────────>│                │
      │                │                │                │                │
      │                │                │ 4. Profile     │                │
      │                │                │    data        │                │
      │                │                │<───────────────│                │
      │                │                │                │                │
      │                │                │ 5. PersonalizerAgent            │
      │                │                │    adapts content               │
      │                │                │───────────────────────────────>│
      │                │                │                │                │
      │                │                │ 6. Personalized│                │
      │                │                │    content     │                │
      │                │                │<───────────────────────────────│
      │                │                │                │                │
      │                │                │ 7. Cache       │                │
      │                │                │    result      │                │
      │                │                │───────────────>│                │
      │                │                │                │                │
      │                │ 8. Return      │                │                │
      │                │    content     │                │                │
      │                │<───────────────│                │                │
      │ 9. Display     │                │                │                │
      │    personalized│                │                │                │
      │<───────────────│                │                │                │
```

---

# PART 2: MODULE BREAKDOWN

## 2.1 System Modules

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           PHYSICAL AI TEXTBOOK PORTAL                        │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                        MODULE 1: FRONTEND                            │   │
│  │                                                                      │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │   │
│  │  │ M1.1: Auth   │  │ M1.2: Dashboard│ │ M1.3: Chat  │              │   │
│  │  │              │  │              │  │   Widget     │              │   │
│  │  │ • Signup     │  │ • Progress   │  │              │              │   │
│  │  │ • Signin     │  │ • Profile    │  │ • Messages   │              │   │
│  │  │ • Onboarding │  │ • Settings   │  │ • Context    │              │   │
│  │  │ • Session    │  │ • Recommend  │  │ • Sources    │              │   │
│  │  └──────────────┘  └──────────────┘  └──────────────┘              │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                        MODULE 2: BACKEND                             │   │
│  │                                                                      │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │   │
│  │  │ M2.1: Auth   │  │ M2.2: Content │  │ M2.3: RAG   │              │   │
│  │  │   Service    │  │   Service    │  │   Service    │              │   │
│  │  │              │  │              │  │              │              │   │
│  │  │ • JWT/Session│  │ • CRUD       │  │ • Ingest     │              │   │
│  │  │ • BetterAuth │  │ • Transform  │  │ • Search     │              │   │
│  │  │ • OAuth      │  │ • Cache      │  │ • Retrieve   │              │   │
│  │  └──────────────┘  └──────────────┘  └──────────────┘              │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                        MODULE 3: AGENTS                              │   │
│  │                                                                      │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │   │
│  │  │ M3.1: RAG    │  │ M3.2: Content │  │ M3.3: Trans │              │   │
│  │  │   Agent      │  │   Agents     │  │   Agents    │              │   │
│  │  │              │  │              │  │              │              │   │
│  │  │ • Chat       │  │ • Summarizer │  │ • Urdu      │              │   │
│  │  │ • Context    │  │ • Generator  │  │ • Personalize│              │   │
│  │  │ • Citations  │  │              │  │              │              │   │
│  │  └──────────────┘  └──────────────┘  └──────────────┘              │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                        MODULE 4: BOOK                                │   │
│  │                                                                      │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │   │
│  │  │ M4.1: Content│  │ M4.2: UI     │  │ M4.3: Deploy │              │   │
│  │  │              │  │ Components   │  │              │              │   │
│  │  │ • 4 Modules  │  │              │  │ • GitHub     │              │   │
│  │  │ • 24 Chapters│  │ • Buttons    │  │   Pages      │              │   │
│  │  │ • Labs       │  │ • ChatWidget │  │ • CI/CD      │              │   │
│  │  │ • Quizzes    │  │              │  │              │              │   │
│  │  └──────────────┘  └──────────────┘  └──────────────┘              │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                        MODULE 5: DATA                                │   │
│  │                                                                      │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │   │
│  │  │ M5.1: Schema │  │ M5.2: Vector │  │ M5.3: Cache  │              │   │
│  │  │              │  │   Store      │  │              │              │   │
│  │  │ • Users      │  │              │  │ • Content    │              │   │
│  │  │ • Profiles   │  │ • Embeddings │  │ • Sessions   │              │   │
│  │  │ • Progress   │  │ • Search     │  │ • Responses  │              │   │
│  │  │ • Chat       │  │              │  │              │              │   │
│  │  └──────────────┘  └──────────────┘  └──────────────┘              │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

# PART 3: FEATURE BREAKDOWN

## 3.1 Feature Matrix

| Feature | Module | Priority | Complexity | Dependencies |
|---------|--------|----------|------------|--------------|
| User Signup | M1.1, M2.1 | P0 | Medium | Neon DB |
| User Signin | M1.1, M2.1 | P0 | Medium | Neon DB |
| Background Questions | M1.1, M2.1 | P0 | Medium | Auth |
| Dashboard | M1.2 | P1 | Medium | Auth |
| Progress Tracking | M1.2, M2.2 | P1 | Medium | Auth, DB |
| RAG Chat | M1.3, M2.3, M3.1 | P0 | High | Qdrant, OpenAI |
| Context Chat | M1.3, M3.1 | P1 | High | RAG |
| Personalize Button | M4.2, M3.3 | P0 | High | Auth, OpenAI |
| Translate Button | M4.2, M3.2 | P0 | High | OpenAI |
| Chapter Summary | M3.2 | P1 | Medium | OpenAI |
| Content Generation | M3.2 | P2 | Medium | OpenAI |
| Book Content | M4.1 | P0 | High | None |
| GitHub Pages Deploy | M4.3 | P0 | Low | GitHub |
| Vercel Deploy | M1 | P0 | Low | Vercel |
| Backend Deploy | M2 | P0 | Medium | Railway |

## 3.2 Feature Dependencies Graph

```
                                    ┌─────────────┐
                                    │   Neon DB   │
                                    │   Setup     │
                                    └──────┬──────┘
                                           │
                    ┌──────────────────────┼──────────────────────┐
                    │                      │                      │
                    ▼                      ▼                      ▼
            ┌───────────────┐      ┌───────────────┐      ┌───────────────┐
            │   User Auth   │      │   Qdrant      │      │   Book        │
            │   (BetterAuth)│      │   Setup       │      │   Content     │
            └───────┬───────┘      └───────┬───────┘      └───────┬───────┘
                    │                      │                      │
                    ▼                      ▼                      │
            ┌───────────────┐      ┌───────────────┐              │
            │  Background   │      │   Document    │              │
            │  Questions    │      │   Ingestion   │              │
            └───────┬───────┘      └───────┬───────┘              │
                    │                      │                      │
                    ├──────────────────────┤                      │
                    │                      │                      │
                    ▼                      ▼                      ▼
            ┌───────────────┐      ┌───────────────┐      ┌───────────────┐
            │   Dashboard   │      │   RAG Chat    │      │   Docusaurus  │
            │               │      │   Agent       │      │   Deploy      │
            └───────┬───────┘      └───────┬───────┘      └───────┬───────┘
                    │                      │                      │
                    │                      │                      │
                    └──────────────────────┼──────────────────────┘
                                           │
                    ┌──────────────────────┼──────────────────────┐
                    │                      │                      │
                    ▼                      ▼                      ▼
            ┌───────────────┐      ┌───────────────┐      ┌───────────────┐
            │ Personalization│     │   Translate   │      │   Chat        │
            │   Agent       │      │   Agent       │      │   Widget      │
            └───────────────┘      └───────────────┘      └───────────────┘
                    │                      │                      │
                    └──────────────────────┼──────────────────────┘
                                           │
                                           ▼
                                    ┌─────────────┐
                                    │   Complete  │
                                    │   Portal    │
                                    └─────────────┘
```

---

# PART 4: MILESTONES

## 4.1 Milestone Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              PROJECT MILESTONES                              │
│                                                                              │
│  M1: Foundation        M2: Core          M3: Features      M4: Polish       │
│  ──────────────       ──────────        ──────────────    ────────────      │
│                                                                              │
│  ▓▓▓▓▓▓░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░     │
│  │                    │                 │                 │          │      │
│  │                    │                 │                 │          │      │
│  Start              Auth             RAG Chat        All Features   Deploy  │
│                   + Book             + Agents         + Polish      + QA    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 4.2 Detailed Milestones

### Milestone 1: Foundation Setup
```yaml
name: Foundation Setup
status: pending
deliverables:
  - Project scaffolding complete
  - Database schema created
  - Basic API structure
  - Environment configuration
  - CI/CD pipelines

tasks:
  - Setup Next.js project with TypeScript
  - Setup FastAPI project with SQLAlchemy
  - Setup Docusaurus project
  - Configure Neon Postgres database
  - Configure Qdrant vector database
  - Create environment configurations
  - Setup GitHub repository structure
  - Create Docker development environment
```

### Milestone 2: Authentication & Book
```yaml
name: Authentication & Book Content
status: pending
deliverables:
  - Complete auth flow (signup, signin, signout)
  - Background questions flow
  - User profile management
  - All 24 chapter content written
  - Docusaurus deployed to GitHub Pages

tasks:
  - Implement BetterAuth on backend
  - Create auth UI components
  - Implement background questions form
  - Create user profile endpoints
  - Write Module 1 content (6 chapters)
  - Write Module 2 content (6 chapters)
  - Write Module 3 content (6 chapters)
  - Write Module 4 content (6 chapters)
  - Configure Docusaurus theme
  - Deploy to GitHub Pages
```

### Milestone 3: RAG & Agents
```yaml
name: RAG System & AI Agents
status: pending
deliverables:
  - Document ingestion pipeline
  - RAG chat fully functional
  - All 5 agents implemented
  - Chat widget in Docusaurus

tasks:
  - Create document chunking pipeline
  - Generate embeddings for all content
  - Implement Qdrant search
  - Create RAGChatAgent
  - Create ChapterSummarizerAgent
  - Create UrduTranslatorAgent
  - Create PersonalizationAgent
  - Create ContentGeneratorAgent
  - Build chat widget component
  - Integrate chat in Docusaurus
```

### Milestone 4: Integration & Polish
```yaml
name: Feature Integration & Polish
status: pending
deliverables:
  - Personalize button working
  - Translate button working
  - Dashboard fully functional
  - All features integrated
  - Performance optimized

tasks:
  - Implement personalization in chapters
  - Implement translation in chapters
  - Build progress tracking
  - Create recommendation system
  - Cache optimization
  - Error handling improvements
  - UI/UX polish
  - Mobile responsiveness
  - Accessibility audit
```

### Milestone 5: Deployment & QA
```yaml
name: Production Deployment & QA
status: pending
deliverables:
  - All services deployed to production
  - Load testing complete
  - Security audit complete
  - Documentation complete

tasks:
  - Deploy frontend to Vercel
  - Deploy backend to Railway
  - Deploy book to GitHub Pages
  - Configure production databases
  - Setup monitoring and logging
  - Perform load testing
  - Security vulnerability scan
  - Write API documentation
  - Create user guide
  - Final QA testing
```

---

# PART 5: REPOSITORY STRUCTURE

## 5.1 Monorepo Structure

```
physicalai-textbook/
│
├── .github/
│   ├── workflows/
│   │   ├── frontend-ci.yml
│   │   ├── backend-ci.yml
│   │   ├── book-deploy.yml
│   │   └── test.yml
│   ├── ISSUE_TEMPLATE/
│   │   ├── bug_report.md
│   │   └── feature_request.md
│   └── PULL_REQUEST_TEMPLATE.md
│
├── frontend/                    # Next.js Application
│   ├── app/
│   ├── components/
│   ├── lib/
│   ├── hooks/
│   ├── store/
│   ├── types/
│   ├── public/
│   ├── __tests__/
│   ├── next.config.js
│   ├── tailwind.config.ts
│   ├── tsconfig.json
│   ├── package.json
│   ├── Dockerfile
│   └── README.md
│
├── backend/                     # FastAPI Application
│   ├── app/
│   │   ├── api/
│   │   ├── core/
│   │   ├── models/
│   │   ├── schemas/
│   │   ├── services/
│   │   ├── agents/
│   │   └── utils/
│   ├── alembic/
│   ├── tests/
│   ├── scripts/
│   ├── requirements.txt
│   ├── Dockerfile
│   └── README.md
│
├── book/                        # Docusaurus Book
│   ├── docs/
│   │   ├── intro.md
│   │   ├── module-1-ros2/
│   │   ├── module-2-simulation/
│   │   ├── module-3-isaac/
│   │   └── module-4-vla/
│   ├── src/
│   │   ├── components/
│   │   ├── pages/
│   │   ├── css/
│   │   └── theme/
│   ├── static/
│   ├── docusaurus.config.ts
│   ├── sidebars.ts
│   ├── package.json
│   └── README.md
│
├── agents/                      # Claude Code Subagents
│   ├── __init__.py
│   ├── base_agent.py
│   ├── rag_chat_agent.py
│   ├── summarizer_agent.py
│   ├── translator_agent.py
│   ├── personalizer_agent.py
│   ├── content_generator_agent.py
│   └── README.md
│
├── specs/                       # Specification Documents
│   ├── sp.constitution.md
│   ├── sp.specs.md
│   ├── sp.plan.md
│   └── sp.tasks.md
│
├── scripts/                     # Utility Scripts
│   ├── setup.sh
│   ├── seed-database.py
│   ├── ingest-documents.py
│   └── generate-embeddings.py
│
├── docker-compose.yml
├── docker-compose.dev.yml
├── .env.example
├── .gitignore
├── LICENSE
└── README.md
```

---

# PART 6: API ARCHITECTURE

## 6.1 API Versioning Strategy

```
/api/v1/                         # Current stable version
├── auth/                        # Authentication endpoints
├── profile/                     # User profile endpoints
├── content/                     # Content management
├── chat/                        # Chat/RAG endpoints
└── agents/                      # Agent endpoints

/api/v2/                         # Future version (when needed)
└── ...
```

## 6.2 API Response Format

```typescript
// Success Response
{
  "success": true,
  "data": { ... },
  "meta": {
    "requestId": "uuid",
    "timestamp": "ISO8601"
  }
}

// Error Response
{
  "success": false,
  "error": {
    "code": "ERROR_CODE",
    "message": "Human readable message",
    "details": { ... }
  },
  "meta": {
    "requestId": "uuid",
    "timestamp": "ISO8601"
  }
}

// Paginated Response
{
  "success": true,
  "data": [...],
  "pagination": {
    "page": 1,
    "limit": 20,
    "total": 100,
    "hasMore": true
  },
  "meta": { ... }
}
```

---

# PART 7: SECURITY ARCHITECTURE

## 7.1 Authentication Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           AUTHENTICATION FLOW                                │
│                                                                              │
│  ┌───────────┐     ┌───────────┐     ┌───────────┐     ┌───────────┐       │
│  │  Client   │     │  Next.js  │     │ BetterAuth│     │   Neon    │       │
│  │           │     │  Server   │     │  (FastAPI)│     │ Database  │       │
│  └─────┬─────┘     └─────┬─────┘     └─────┬─────┘     └─────┬─────┘       │
│        │                 │                 │                 │              │
│        │ 1. Login        │                 │                 │              │
│        │────────────────>│                 │                 │              │
│        │                 │ 2. Forward      │                 │              │
│        │                 │────────────────>│                 │              │
│        │                 │                 │ 3. Verify       │              │
│        │                 │                 │────────────────>│              │
│        │                 │                 │ 4. User data    │              │
│        │                 │                 │<────────────────│              │
│        │                 │ 5. JWT Token    │                 │              │
│        │                 │<────────────────│                 │              │
│        │ 6. Set Cookie   │                 │                 │              │
│        │<────────────────│                 │                 │              │
│        │                 │                 │                 │              │
│        │ 7. API Request  │                 │                 │              │
│        │────────────────>│ 8. Verify JWT   │                 │              │
│        │                 │────────────────>│                 │              │
│        │                 │ 9. Valid        │                 │              │
│        │                 │<────────────────│                 │              │
│        │ 10. Response    │                 │                 │              │
│        │<────────────────│                 │                 │              │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘

Token Structure:
├── Access Token (JWT)
│   ├── Header: { alg: "HS256", typ: "JWT" }
│   ├── Payload: { sub: userId, email, exp, iat }
│   └── Signature: HMAC(header + payload, secret)
│
└── Refresh Token
    ├── Stored in: HTTP-only cookie
    ├── Expires: 7 days
    └── Used for: Silent token refresh
```

## 7.2 Rate Limiting Strategy

```yaml
rate_limits:
  auth:
    signup: 5 requests/hour/IP
    signin: 10 requests/minute/IP
    refresh: 30 requests/hour/user

  api:
    default: 100 requests/minute/user
    chat: 20 requests/minute/user
    agents: 10 requests/minute/user

  public:
    default: 50 requests/minute/IP
```

---

# PART 8: PERFORMANCE ARCHITECTURE

## 8.1 Caching Strategy

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           CACHING LAYERS                                     │
│                                                                              │
│  Layer 1: Browser Cache                                                      │
│  ├── Static assets: 1 year (with hash)                                      │
│  ├── API responses: No cache (dynamic)                                      │
│  └── Images: 30 days                                                        │
│                                                                              │
│  Layer 2: CDN Cache (Vercel Edge)                                           │
│  ├── Static pages: Until revalidation                                       │
│  ├── ISR pages: Configurable stale time                                     │
│  └── API routes: No cache                                                   │
│                                                                              │
│  Layer 3: Application Cache                                                  │
│  ├── Personalized content: 24 hours per user                                │
│  ├── Translated content: 7 days                                             │
│  ├── Chapter summaries: 7 days                                              │
│  └── Chat responses: No cache                                               │
│                                                                              │
│  Layer 4: Database Cache                                                     │
│  ├── Query results: Connection pooling                                      │
│  ├── Frequent queries: Indexed                                              │
│  └── Vector search: Optimized collections                                   │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 8.2 Performance Targets

```yaml
frontend:
  LCP: < 2.5s
  FID: < 100ms
  CLS: < 0.1
  TTI: < 3.5s
  Bundle size: < 200KB (initial)

backend:
  API response (p50): < 100ms
  API response (p95): < 200ms
  API response (p99): < 500ms
  Chat response (p50): < 2s
  Chat response (p95): < 3s

database:
  Query time (p50): < 10ms
  Query time (p95): < 50ms
  Connection pool: 10 connections

vector_search:
  Search time (p50): < 100ms
  Search time (p95): < 200ms
```

---

## PLAN COMPLETE

```
Document: SP.PLAN
Version: 1.0.0
Created: 2025-12-12
Milestones: 5
Modules: 5 major, 15 sub-modules
Features: 15 core features
```
