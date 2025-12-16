# SP.CONSTITUTION
## Physical AI & Humanoid Robotics - AI-Native Textbook Portal

---

## 1. SYSTEM IDENTITY

### 1.1 Project Name
**PhysicalAI-TextBook-Portal** (PATP)

### 1.2 Mission Statement
Build a production-ready, AI-native educational platform for Physical AI & Humanoid Robotics that provides personalized learning experiences through intelligent agents, RAG-powered assistance, and adaptive content delivery.

### 1.3 Core Values
- **Accessibility**: Content available in English and Urdu
- **Personalization**: Adaptive learning based on user background
- **Intelligence**: AI-powered assistance at every step
- **Practicality**: Hands-on labs and real-world applications
- **Security**: Authenticated, personalized experiences

---

## 2. SYSTEM ARCHITECTURE CONSTITUTION

### 2.1 Architecture Pattern
```
┌─────────────────────────────────────────────────────────────────┐
│                    PRESENTATION LAYER                           │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │   Next.js App   │  │  Docusaurus     │  │  Floating       │ │
│  │   (Dashboard)   │  │  (Book Site)    │  │  RAG Chatbot    │ │
│  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘ │
└───────────┼────────────────────┼────────────────────┼──────────┘
            │                    │                    │
┌───────────┴────────────────────┴────────────────────┴──────────┐
│                    API GATEWAY LAYER                            │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                    FastAPI Backend                          ││
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐      ││
│  │  │ Auth API │ │ RAG API  │ │ Agent API│ │Content API│      ││
│  │  └──────────┘ └──────────┘ └──────────┘ └──────────┘      ││
│  └─────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
            │                    │                    │
┌───────────┴────────────────────┴────────────────────┴──────────┐
│                    INTELLIGENCE LAYER                           │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                 Claude Code Subagents                       ││
│  │  ┌────────────┐ ┌────────────┐ ┌────────────┐             ││
│  │  │RAGChatAgent│ │Personalizer│ │UrduTranslator│            ││
│  │  └────────────┘ └────────────┘ └────────────┘             ││
│  │  ┌────────────┐ ┌────────────┐                             ││
│  │  │Summarizer  │ │ContentGen  │                             ││
│  │  └────────────┘ └────────────┘                             ││
│  └─────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
            │                    │                    │
┌───────────┴────────────────────┴────────────────────┴──────────┐
│                    DATA LAYER                                   │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│  │ Neon Postgres│  │   Qdrant     │  │  OpenAI API  │         │
│  │ (Users/Auth) │  │ (Vectors)    │  │ (Embeddings) │         │
│  └──────────────┘  └──────────────┘  └──────────────┘         │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 Technology Stack Constitution

#### Frontend Stack
| Component | Technology | Version | Purpose |
|-----------|------------|---------|---------|
| Framework | Next.js | 14.x | App Router, RSC |
| Styling | Tailwind CSS | 3.x | Utility-first CSS |
| UI Components | shadcn/ui | Latest | Accessible components |
| State | Zustand | 4.x | Client state management |
| Forms | React Hook Form | 7.x | Form handling |
| Validation | Zod | 3.x | Schema validation |
| Auth Client | BetterAuth | 1.x | Authentication |

#### Documentation Stack
| Component | Technology | Version | Purpose |
|-----------|------------|---------|---------|
| Framework | Docusaurus | 3.x | Static site generator |
| Theme | Classic | 3.x | Default theme + custom |
| MDX | MDX v2 | 2.x | Enhanced markdown |
| Search | Algolia/Local | Latest | Full-text search |

#### Backend Stack
| Component | Technology | Version | Purpose |
|-----------|------------|---------|---------|
| Framework | FastAPI | 0.109+ | Async Python API |
| Auth Server | BetterAuth | 1.x | JWT/Session auth |
| ORM | SQLAlchemy | 2.x | Database ORM |
| Migrations | Alembic | 1.x | Schema migrations |
| Validation | Pydantic | 2.x | Data validation |

#### AI/ML Stack
| Component | Technology | Version | Purpose |
|-----------|------------|---------|---------|
| LLM | OpenAI GPT-4 | Latest | Text generation |
| Embeddings | OpenAI Ada-002 | Latest | Vector embeddings |
| Vector DB | Qdrant | 1.7+ | Vector storage |
| RAG | LangChain | 0.1+ | RAG orchestration |
| Agents | OpenAI Agents | Latest | Agent framework |

#### Database Stack
| Component | Technology | Purpose |
|-----------|------------|---------|
| Primary DB | Neon Postgres | User data, sessions, profiles |
| Vector DB | Qdrant Cloud | Document embeddings |
| Cache | Redis (optional) | Session cache |

#### Deployment Stack
| Component | Platform | Purpose |
|-----------|----------|---------|
| Frontend | Vercel | Next.js hosting |
| Documentation | GitHub Pages | Docusaurus hosting |
| Backend | Railway/Fly.io | FastAPI hosting |
| Vector DB | Qdrant Cloud | Managed vectors |
| Database | Neon | Managed Postgres |

---

## 3. AGENT CONSTITUTION

### 3.1 Agent Hierarchy
```
┌─────────────────────────────────────────────────────────┐
│                   ORCHESTRATOR AGENT                     │
│            (Routes requests to specialized agents)       │
└─────────────────────────┬───────────────────────────────┘
                          │
    ┌─────────────────────┼─────────────────────┐
    │                     │                     │
    ▼                     ▼                     ▼
┌───────────┐      ┌───────────┐      ┌───────────┐
│RAGChat    │      │Content    │      │Translator │
│Agent      │      │Generator  │      │Agent      │
└───────────┘      └───────────┘      └───────────┘
    │                     │                     │
    ▼                     ▼                     ▼
┌───────────┐      ┌───────────┐      ┌───────────┐
│Summarizer │      │Personalizer│     │Assessment │
│Agent      │      │Agent       │     │Agent      │
└───────────┘      └───────────┘      └───────────┘
```

### 3.2 Agent Definitions

#### 3.2.1 RAGChatAgent
```yaml
name: RAGChatAgent
purpose: Answer questions about the textbook using RAG
capabilities:
  - Semantic search across book content
  - Context-aware responses
  - Citation of sources
  - Selected text analysis
inputs:
  - user_query: string
  - selected_text: string (optional)
  - chapter_context: string (optional)
outputs:
  - answer: string
  - sources: list[Citation]
  - confidence: float
tools:
  - qdrant_search
  - openai_completion
  - context_retriever
```

#### 3.2.2 ChapterSummarizerAgent
```yaml
name: ChapterSummarizerAgent
purpose: Generate concise summaries of chapters
capabilities:
  - Extract key concepts
  - Generate bullet points
  - Create learning objectives
inputs:
  - chapter_content: string
  - summary_type: enum[brief, detailed, bullets]
outputs:
  - summary: string
  - key_concepts: list[string]
  - learning_objectives: list[string]
tools:
  - text_splitter
  - openai_completion
```

#### 3.2.3 UrduTranslatorAgent
```yaml
name: UrduTranslatorAgent
purpose: Translate content to Urdu while preserving technical terms
capabilities:
  - Accurate Urdu translation
  - Technical term preservation
  - Formatting preservation
  - RTL text handling
inputs:
  - content: string
  - preserve_terms: list[string] (optional)
outputs:
  - translated_content: string
  - preserved_terms: list[dict]
tools:
  - openai_completion
  - term_extractor
```

#### 3.2.4 PersonalizationAgent
```yaml
name: PersonalizationAgent
purpose: Adapt content based on user background
capabilities:
  - Analyze user profile
  - Adjust content complexity
  - Add relevant examples
  - Skip/expand sections
inputs:
  - chapter_content: string
  - user_profile: UserProfile
outputs:
  - personalized_content: string
  - adaptations_made: list[string]
tools:
  - profile_analyzer
  - content_adapter
  - openai_completion
```

#### 3.2.5 ContentGeneratorAgent
```yaml
name: ContentGeneratorAgent
purpose: Generate new educational content
capabilities:
  - Create exercises
  - Generate quizzes
  - Write explanations
  - Create code examples
inputs:
  - topic: string
  - content_type: enum[exercise, quiz, explanation, code]
  - difficulty: enum[beginner, intermediate, advanced]
outputs:
  - generated_content: string
  - metadata: dict
tools:
  - openai_completion
  - code_generator
  - quiz_formatter
```

---

## 4. DATA CONSTITUTION

### 4.1 User Profile Schema
```typescript
interface UserProfile {
  id: string;
  email: string;
  name: string;
  createdAt: DateTime;
  updatedAt: DateTime;

  // Background Questions
  background: {
    // Software Background
    softwareExperience: 'none' | 'beginner' | 'intermediate' | 'advanced';
    programmingLanguages: string[];
    rosExperience: boolean;
    linuxExperience: 'none' | 'basic' | 'comfortable' | 'expert';

    // Hardware Background
    hardwareExperience: 'none' | 'hobbyist' | 'professional';
    roboticsExperience: boolean;
    previousProjects: string[];

    // Equipment
    hasGpuWorkstation: boolean;
    gpuModel?: string;
    hasJetsonKit: boolean;
    jetsonModel?: string;
    hasRobotHardware: boolean;
    robotDescription?: string;
  };

  // Learning Progress
  progress: {
    completedChapters: string[];
    currentChapter: string;
    quizScores: Record<string, number>;
    timeSpent: Record<string, number>;
  };

  // Preferences
  preferences: {
    language: 'en' | 'ur';
    theme: 'light' | 'dark' | 'system';
    notificationsEnabled: boolean;
  };
}
```

### 4.2 Content Schema
```typescript
interface Chapter {
  id: string;
  moduleId: string;
  title: string;
  slug: string;
  content: string;
  order: number;

  metadata: {
    estimatedTime: number; // minutes
    difficulty: 'beginner' | 'intermediate' | 'advanced';
    prerequisites: string[];
    learningObjectives: string[];
    keywords: string[];
  };

  resources: {
    codeExamples: CodeExample[];
    exercises: Exercise[];
    quizzes: Quiz[];
    videos: Video[];
  };
}

interface Module {
  id: string;
  title: string;
  description: string;
  order: number;
  chapters: Chapter[];
  icon: string;
  color: string;
}
```

### 4.3 RAG Document Schema
```typescript
interface RAGDocument {
  id: string;
  chapterId: string;
  content: string;
  embedding: number[];
  metadata: {
    title: string;
    section: string;
    pageNumber: number;
    wordCount: number;
  };
}
```

---

## 5. API CONSTITUTION

### 5.1 API Design Principles
1. **RESTful**: Follow REST conventions for CRUD operations
2. **Versioned**: All APIs versioned (v1, v2, etc.)
3. **Authenticated**: JWT-based authentication
4. **Rate Limited**: Protect against abuse
5. **Documented**: OpenAPI/Swagger documentation

### 5.2 Core API Endpoints

#### Authentication
```
POST   /api/v1/auth/signup          # Create account
POST   /api/v1/auth/signin          # Login
POST   /api/v1/auth/signout         # Logout
GET    /api/v1/auth/session         # Get session
POST   /api/v1/auth/refresh         # Refresh token
```

#### User Profile
```
GET    /api/v1/profile              # Get profile
PUT    /api/v1/profile              # Update profile
PUT    /api/v1/profile/background   # Update background
GET    /api/v1/profile/progress     # Get learning progress
PUT    /api/v1/profile/progress     # Update progress
```

#### Content
```
GET    /api/v1/modules              # List modules
GET    /api/v1/modules/:id          # Get module
GET    /api/v1/chapters/:id         # Get chapter
POST   /api/v1/chapters/:id/personalize  # Personalize chapter
POST   /api/v1/chapters/:id/translate    # Translate chapter
GET    /api/v1/chapters/:id/summary      # Get summary
```

#### RAG Chat
```
POST   /api/v1/chat                 # Send message
POST   /api/v1/chat/context         # Chat with selected text
GET    /api/v1/chat/history         # Get chat history
DELETE /api/v1/chat/history         # Clear history
```

#### Agents
```
POST   /api/v1/agents/summarize     # Summarize content
POST   /api/v1/agents/translate     # Translate content
POST   /api/v1/agents/personalize   # Personalize content
POST   /api/v1/agents/generate      # Generate content
```

---

## 6. SECURITY CONSTITUTION

### 6.1 Authentication Requirements
- **BetterAuth** for authentication framework
- **JWT** tokens with 15-minute expiry
- **Refresh tokens** with 7-day expiry
- **HTTP-only cookies** for token storage
- **CSRF protection** enabled
- **Rate limiting** on auth endpoints

### 6.2 Authorization Rules
```yaml
roles:
  - anonymous:
      - read: public_chapters
      - access: demo_chat (limited)

  - authenticated:
      - read: all_chapters
      - write: own_profile
      - access: full_chat
      - access: personalization
      - access: translation

  - admin:
      - all: *
      - manage: users
      - manage: content
```

### 6.3 Data Protection
- **Encryption at rest**: All sensitive data encrypted
- **Encryption in transit**: TLS 1.3 required
- **PII handling**: GDPR compliant
- **Password hashing**: Argon2id
- **API keys**: Environment variables only

---

## 7. DEPLOYMENT CONSTITUTION

### 7.1 Environment Configuration
```yaml
environments:
  development:
    frontend: localhost:3000
    backend: localhost:8000
    docusaurus: localhost:3001
    database: local_postgres
    vector_db: local_qdrant

  staging:
    frontend: staging.physicalai.dev
    backend: api-staging.physicalai.dev
    docusaurus: book-staging.physicalai.dev
    database: neon_staging
    vector_db: qdrant_staging

  production:
    frontend: physicalai.dev
    backend: api.physicalai.dev
    docusaurus: book.physicalai.dev
    database: neon_production
    vector_db: qdrant_production
```

### 7.2 CI/CD Pipeline
```yaml
pipeline:
  on_push:
    - lint
    - type_check
    - unit_tests
    - build

  on_pr:
    - lint
    - type_check
    - unit_tests
    - integration_tests
    - preview_deploy

  on_merge_main:
    - lint
    - type_check
    - all_tests
    - build
    - deploy_staging
    - smoke_tests
    - deploy_production
```

---

## 8. QUALITY CONSTITUTION

### 8.1 Code Quality Standards
- **TypeScript strict mode** for all frontend code
- **Python type hints** for all backend code
- **ESLint + Prettier** for JavaScript/TypeScript
- **Ruff + Black** for Python
- **100% type coverage** goal
- **80% test coverage** minimum

### 8.2 Performance Standards
- **LCP** < 2.5s (Largest Contentful Paint)
- **FID** < 100ms (First Input Delay)
- **CLS** < 0.1 (Cumulative Layout Shift)
- **API response** < 200ms (p95)
- **Chat response** < 3s (p95)

### 8.3 Accessibility Standards
- **WCAG 2.1 AA** compliance
- **Keyboard navigation** support
- **Screen reader** compatible
- **Color contrast** ratios met
- **RTL support** for Urdu

---

## 9. CONTENT CONSTITUTION

### 9.1 Module Structure
```
Module 1: The Robotic Nervous System (ROS 2)
├── Chapter 1.1: Introduction to ROS 2
├── Chapter 1.2: Nodes, Topics, and Services
├── Chapter 1.3: Actions and Parameters
├── Chapter 1.4: Launch Files and Configuration
├── Chapter 1.5: Building Custom Packages
└── Chapter 1.6: Lab - Building a ROS 2 Robot

Module 2: The Digital Twin (Gazebo & Unity)
├── Chapter 2.1: Introduction to Simulation
├── Chapter 2.2: Gazebo Basics
├── Chapter 2.3: URDF and Robot Models
├── Chapter 2.4: Unity Robotics Hub
├── Chapter 2.5: Sensor Simulation
└── Chapter 2.6: Lab - Digital Twin Pipeline

Module 3: The AI-Robot Brain (NVIDIA Isaac)
├── Chapter 3.1: Introduction to Isaac
├── Chapter 3.2: Isaac Sim Fundamentals
├── Chapter 3.3: Perception with Isaac
├── Chapter 3.4: Manipulation Planning
├── Chapter 3.5: Isaac ROS Integration
└── Chapter 3.6: Lab - AI-Powered Manipulation

Module 4: Vision-Language-Action (VLA)
├── Chapter 4.1: Introduction to VLA
├── Chapter 4.2: Vision Transformers for Robots
├── Chapter 4.3: Language Models for Robotics
├── Chapter 4.4: Action Generation
├── Chapter 4.5: End-to-End VLA Systems
└── Chapter 4.6: Capstone Project
```

### 9.2 Chapter Template
Each chapter MUST include:
1. Learning Objectives
2. Prerequisites Check
3. Concept Introduction
4. Detailed Explanation
5. Code Examples (with ROS 2/Python)
6. Hands-on Lab
7. Knowledge Check Quiz
8. Summary
9. Further Reading
10. Hardware Requirements (if applicable)

---

## 10. GOVERNANCE

### 10.1 Version Control
- **Git flow** branching strategy
- **Conventional commits** required
- **PR reviews** mandatory
- **Protected main branch**

### 10.2 Documentation Requirements
- **README** for every directory
- **API documentation** auto-generated
- **Architecture decisions** recorded
- **Changelog** maintained

### 10.3 Change Management
- **Breaking changes** require RFC
- **Feature flags** for gradual rollout
- **Deprecation policy** (2 versions)

---

## CONSTITUTION SIGNATURES

```
Document: SP.CONSTITUTION
Version: 1.0.0
Created: 2025-12-12
Status: ACTIVE

This constitution governs all development and operations
of the Physical AI & Humanoid Robotics Textbook Portal.
```
