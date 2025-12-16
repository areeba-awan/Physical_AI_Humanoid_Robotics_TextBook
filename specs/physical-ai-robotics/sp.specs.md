# SP.SPECS
## Physical AI & Humanoid Robotics - Technical Specifications

---

# PART 1: FRONTEND SPECIFICATIONS

## 1.1 Next.js Application Structure

```
frontend/
├── app/
│   ├── (auth)/
│   │   ├── signin/
│   │   │   └── page.tsx
│   │   ├── signup/
│   │   │   └── page.tsx
│   │   └── layout.tsx
│   ├── (dashboard)/
│   │   ├── dashboard/
│   │   │   └── page.tsx
│   │   ├── profile/
│   │   │   └── page.tsx
│   │   ├── progress/
│   │   │   └── page.tsx
│   │   └── layout.tsx
│   ├── api/
│   │   ├── auth/
│   │   │   └── [...betterauth]/
│   │   │       └── route.ts
│   │   ├── chat/
│   │   │   └── route.ts
│   │   └── proxy/
│   │       └── [...path]/
│   │           └── route.ts
│   ├── layout.tsx
│   ├── page.tsx
│   └── globals.css
├── components/
│   ├── ui/
│   │   ├── button.tsx
│   │   ├── card.tsx
│   │   ├── dialog.tsx
│   │   ├── dropdown-menu.tsx
│   │   ├── form.tsx
│   │   ├── input.tsx
│   │   ├── label.tsx
│   │   ├── select.tsx
│   │   ├── skeleton.tsx
│   │   ├── tabs.tsx
│   │   ├── textarea.tsx
│   │   └── toast.tsx
│   ├── auth/
│   │   ├── signin-form.tsx
│   │   ├── signup-form.tsx
│   │   ├── background-questions.tsx
│   │   └── auth-provider.tsx
│   ├── chat/
│   │   ├── chat-widget.tsx
│   │   ├── chat-message.tsx
│   │   ├── chat-input.tsx
│   │   └── chat-context-menu.tsx
│   ├── dashboard/
│   │   ├── sidebar.tsx
│   │   ├── header.tsx
│   │   ├── progress-card.tsx
│   │   └── module-card.tsx
│   ├── layout/
│   │   ├── navbar.tsx
│   │   ├── footer.tsx
│   │   └── mobile-nav.tsx
│   └── shared/
│       ├── loading.tsx
│       ├── error-boundary.tsx
│       └── theme-toggle.tsx
├── lib/
│   ├── auth.ts
│   ├── api.ts
│   ├── utils.ts
│   └── validators.ts
├── hooks/
│   ├── use-auth.ts
│   ├── use-chat.ts
│   ├── use-profile.ts
│   └── use-progress.ts
├── store/
│   ├── auth-store.ts
│   ├── chat-store.ts
│   └── ui-store.ts
├── types/
│   ├── auth.ts
│   ├── chat.ts
│   ├── content.ts
│   └── user.ts
├── public/
│   ├── images/
│   ├── icons/
│   └── fonts/
├── next.config.js
├── tailwind.config.ts
├── tsconfig.json
└── package.json
```

## 1.2 Component Specifications

### 1.2.1 SignupForm Component
```typescript
// components/auth/signup-form.tsx

interface SignupFormProps {
  onSuccess?: () => void;
  redirectTo?: string;
}

interface SignupFormData {
  email: string;
  password: string;
  confirmPassword: string;
  name: string;
  acceptTerms: boolean;
}

// Features:
// - Email validation (RFC 5322)
// - Password strength indicator
// - Real-time validation
// - Loading states
// - Error handling
// - Success redirect to background questions

// UI Elements:
// - Email input with icon
// - Password input with show/hide toggle
// - Confirm password input
// - Name input
// - Terms checkbox with link
// - Submit button with loading state
// - OAuth providers (Google, GitHub)
// - Link to signin page
```

### 1.2.2 BackgroundQuestions Component
```typescript
// components/auth/background-questions.tsx

interface BackgroundQuestionsProps {
  userId: string;
  onComplete: () => void;
}

interface BackgroundFormData {
  // Software Background
  softwareExperience: 'none' | 'beginner' | 'intermediate' | 'advanced';
  programmingLanguages: string[];
  rosExperience: boolean;
  linuxExperience: 'none' | 'basic' | 'comfortable' | 'expert';

  // Hardware Background
  hardwareExperience: 'none' | 'hobbyist' | 'professional';
  roboticsExperience: boolean;
  previousProjects: string;

  // Equipment
  hasGpuWorkstation: boolean;
  gpuModel?: string;
  hasJetsonKit: boolean;
  jetsonModel?: string;
  hasRobotHardware: boolean;
  robotDescription?: string;
}

// Multi-step form with 4 steps:
// Step 1: Software Background
// Step 2: Hardware Background
// Step 3: Equipment Check
// Step 4: Review & Submit

// UI Elements:
// - Progress indicator (1/4, 2/4, etc.)
// - Radio groups for experience levels
// - Checkbox groups for languages
// - Conditional fields (show GPU model if hasGpuWorkstation)
// - Back/Next navigation
// - Skip option
// - Submit with loading state
```

### 1.2.3 ChatWidget Component
```typescript
// components/chat/chat-widget.tsx

interface ChatWidgetProps {
  position?: 'bottom-right' | 'bottom-left';
  initialOpen?: boolean;
  contextMode?: boolean;
}

interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Citation[];
  timestamp: Date;
}

interface ChatState {
  messages: ChatMessage[];
  isLoading: boolean;
  error: string | null;
  selectedText: string | null;
}

// Features:
// - Floating button to open/close
// - Expandable chat window
// - Message history with scroll
// - Markdown rendering
// - Code syntax highlighting
// - Source citations
// - Selected text context mode
// - Clear history option
// - Minimize/maximize

// UI Elements:
// - Floating action button (robot icon)
// - Chat header with title and controls
// - Message list with avatars
// - Input field with send button
// - Loading indicator
// - Error toast
// - Context indicator when text selected
```

### 1.2.4 PersonalizeButton Component
```typescript
// components/content/personalize-button.tsx

interface PersonalizeButtonProps {
  chapterId: string;
  chapterContent: string;
  onPersonalized: (content: string) => void;
}

// Features:
// - One-click personalization
// - Loading state with progress
// - Preview before applying
// - Revert option
// - Diff view option

// UI Elements:
// - Button with user icon
// - "Personalize for Me" label
// - Loading spinner with "Adapting content..."
// - Modal with preview
// - Apply/Cancel actions
// - Success toast
```

### 1.2.5 TranslateButton Component
```typescript
// components/content/translate-button.tsx

interface TranslateButtonProps {
  chapterId: string;
  chapterContent: string;
  onTranslated: (content: string) => void;
  targetLanguage?: 'ur' | 'en';
}

// Features:
// - Toggle translation
// - Preserve technical terms
// - RTL support for Urdu
// - Loading state
// - Cache translations

// UI Elements:
// - Button with translate icon
// - "Translate to Urdu" / "View Original"
// - Loading spinner
// - Language indicator badge
// - RTL layout switch
```

## 1.3 Page Specifications

### 1.3.1 Home Page (/)
```
┌─────────────────────────────────────────────────────────────┐
│ [Logo]  Modules  Book Overview      [GitHub] [Chat] [Login] │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│     Physical AI & Humanoid Robotics                        │
│     ════════════════════════════════                        │
│     A Comprehensive Guide to Embodied AI                   │
│                                                             │
│     Hands-on labs and ready-to-run examples                │
│                                                             │
│     [Get Started]        [View Book]                       │
│                                                             │
├─────────────────────────────────────────────────────────────┤
│                    QUICK START CARDS                        │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐       │
│  │ Quickstart   │ │ Simulation   │ │ Capstone     │       │
│  │ Guide        │ │ Templates    │ │ Recipes      │       │
│  │              │ │              │ │              │       │
│  │ Set up in    │ │ Pre-built    │ │ End-to-end   │       │
│  │ minutes      │ │ environments │ │ projects     │       │
│  └──────────────┘ └──────────────┘ └──────────────┘       │
├─────────────────────────────────────────────────────────────┤
│                    COURSE MODULES                           │
│  ┌──────────────────────┐ ┌──────────────────────┐        │
│  │ Module 1             │ │ Module 2             │        │
│  │ The Robotic Nervous  │ │ The Digital Twin     │        │
│  │ System (ROS 2)       │ │ (Gazebo & Unity)     │        │
│  │                      │ │                      │        │
│  │ [Start Module →]     │ │ [Start Module →]     │        │
│  └──────────────────────┘ └──────────────────────┘        │
│  ┌──────────────────────┐ ┌──────────────────────┐        │
│  │ Module 3             │ │ Module 4             │        │
│  │ The AI-Robot Brain   │ │ Vision-Language-     │        │
│  │ (NVIDIA Isaac)       │ │ Action (VLA)         │        │
│  │                      │ │                      │        │
│  │ [Start Module →]     │ │ [Start Module →]     │        │
│  └──────────────────────┘ └──────────────────────┘        │
├─────────────────────────────────────────────────────────────┤
│                    KEY FEATURES                             │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐           │
│  │ 🔬         │  │ 📊         │  │ 🤖         │           │
│  │ Hands-on   │  │ Interactive│  │ VLA        │           │
│  │ Labs       │  │ Visuals    │  │ Integration│           │
│  └────────────┘  └────────────┘  └────────────┘           │
├─────────────────────────────────────────────────────────────┤
│                    CALL TO ACTION                           │
│                                                             │
│     Ready to build intelligent robots?                     │
│                                                             │
│     [Get Started]        [Contribute]                      │
│                                                             │
├─────────────────────────────────────────────────────────────┤
│ Book    Community    Social                                │
│ ────    ─────────    ──────                                │
│ Overview Stack Overflow X                                  │
│ Modules  Discord      GitHub                               │
│                                                             │
│ © 2025 Physical AI Textbook                                │
└─────────────────────────────────────────────────────────────┘
```

### 1.3.2 Signup Page (/signup)
```
┌─────────────────────────────────────────────────────────────┐
│ [Logo]                                          [Sign In]   │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │                                                     │   │
│  │     Create your account                            │   │
│  │     ═══════════════════                            │   │
│  │                                                     │   │
│  │     Start your Physical AI journey                 │   │
│  │                                                     │   │
│  │     ┌─────────────────────────────────────────┐   │   │
│  │     │ Full Name                               │   │   │
│  │     └─────────────────────────────────────────┘   │   │
│  │                                                     │   │
│  │     ┌─────────────────────────────────────────┐   │   │
│  │     │ Email                                   │   │   │
│  │     └─────────────────────────────────────────┘   │   │
│  │                                                     │   │
│  │     ┌─────────────────────────────────────────┐   │   │
│  │     │ Password                            👁  │   │   │
│  │     └─────────────────────────────────────────┘   │   │
│  │     Password strength: ████░░░░ Medium             │   │
│  │                                                     │   │
│  │     ┌─────────────────────────────────────────┐   │   │
│  │     │ Confirm Password                    👁  │   │   │
│  │     └─────────────────────────────────────────┘   │   │
│  │                                                     │   │
│  │     ☑ I agree to Terms and Privacy Policy         │   │
│  │                                                     │   │
│  │     ┌─────────────────────────────────────────┐   │   │
│  │     │          Create Account                 │   │   │
│  │     └─────────────────────────────────────────┘   │   │
│  │                                                     │   │
│  │     ─────────── OR ───────────                    │   │
│  │                                                     │   │
│  │     [🔵 Continue with Google]                     │   │
│  │     [⚫ Continue with GitHub]                     │   │
│  │                                                     │   │
│  │     Already have an account? Sign In              │   │
│  │                                                     │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 1.3.3 Background Questions Page (/onboarding)
```
┌─────────────────────────────────────────────────────────────┐
│ [Logo]                                    Step 1 of 4  [Skip]│
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │                                                     │   │
│  │     Tell us about your background                  │   │
│  │     ═════════════════════════════                  │   │
│  │                                                     │   │
│  │     This helps us personalize your learning        │   │
│  │                                                     │   │
│  │     ████░░░░░░░░░░░░ 25% complete                 │   │
│  │                                                     │   │
│  │     ─────────────────────────────────────────────  │   │
│  │                                                     │   │
│  │     SOFTWARE EXPERIENCE                            │   │
│  │                                                     │   │
│  │     How would you rate your programming            │   │
│  │     experience?                                    │   │
│  │                                                     │   │
│  │     ○ None - I'm completely new                   │   │
│  │     ○ Beginner - Basic understanding              │   │
│  │     ● Intermediate - Comfortable coding           │   │
│  │     ○ Advanced - Professional developer           │   │
│  │                                                     │   │
│  │     Which programming languages do you know?      │   │
│  │                                                     │   │
│  │     ☑ Python     ☑ C++      ☐ Rust               │   │
│  │     ☐ JavaScript ☐ Go       ☐ Other              │   │
│  │                                                     │   │
│  │     Do you have experience with ROS?              │   │
│  │                                                     │   │
│  │     ○ Yes  ● No                                   │   │
│  │                                                     │   │
│  │     Linux experience level?                       │   │
│  │                                                     │   │
│  │     ○ None  ● Basic  ○ Comfortable  ○ Expert     │   │
│  │                                                     │   │
│  │     ┌────────────┐ ┌────────────────────────┐    │   │
│  │     │    Back    │ │         Next           │    │   │
│  │     └────────────┘ └────────────────────────┘    │   │
│  │                                                     │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 1.3.4 Dashboard Page (/dashboard)
```
┌─────────────────────────────────────────────────────────────┐
│ [Logo]  Dashboard  Book  Profile     [Theme] [Notifications]│
├───────────┬─────────────────────────────────────────────────┤
│           │                                                 │
│  MENU     │  Welcome back, Ahmed!                          │
│  ────     │  ═══════════════════                           │
│           │                                                 │
│  📊 Dash  │  Your Learning Progress                        │
│  📚 Book  │  ──────────────────────                        │
│  👤 Profile│                                                │
│  ⚙ Settings│ ┌─────────────────────────────────────────┐  │
│           │  │                                         │  │
│           │  │  Overall: ████████░░░░░░░░ 52%         │  │
│           │  │                                         │  │
│           │  │  Module 1: █████████████░░ 80%         │  │
│           │  │  Module 2: ████████░░░░░░░ 50%         │  │
│           │  │  Module 3: ████░░░░░░░░░░░ 25%         │  │
│           │  │  Module 4: ░░░░░░░░░░░░░░░ 0%          │  │
│           │  │                                         │  │
│           │  └─────────────────────────────────────────┘  │
│           │                                                 │
│           │  Continue Learning                             │
│           │  ─────────────────                             │
│           │                                                 │
│           │  ┌──────────────────────────────────────────┐ │
│           │  │ 📖 Chapter 2.3: URDF and Robot Models   │ │
│           │  │    Module 2 • 45 min remaining          │ │
│           │  │                          [Continue →]    │ │
│           │  └──────────────────────────────────────────┘ │
│           │                                                 │
│           │  Recommended for You                           │
│           │  ───────────────────                           │
│           │                                                 │
│           │  Based on your background (Intermediate Python,│
│           │  No ROS experience), we suggest:              │
│           │                                                 │
│           │  • Complete ROS 2 basics before Isaac         │
│           │  • Try the beginner simulation lab first      │
│           │                                                 │
├───────────┴─────────────────────────────────────────────────┤
│ [💬 Ask AI Assistant]                               [🤖]    │
└─────────────────────────────────────────────────────────────┘
```

## 1.4 State Management Specifications

### 1.4.1 Auth Store (Zustand)
```typescript
// store/auth-store.ts

interface AuthState {
  user: User | null;
  session: Session | null;
  isLoading: boolean;
  error: string | null;
}

interface AuthActions {
  setUser: (user: User | null) => void;
  setSession: (session: Session | null) => void;
  setLoading: (loading: boolean) => void;
  setError: (error: string | null) => void;
  logout: () => Promise<void>;
  refreshSession: () => Promise<void>;
}

type AuthStore = AuthState & AuthActions;
```

### 1.4.2 Chat Store (Zustand)
```typescript
// store/chat-store.ts

interface ChatState {
  messages: ChatMessage[];
  isOpen: boolean;
  isLoading: boolean;
  selectedText: string | null;
  contextChapter: string | null;
}

interface ChatActions {
  addMessage: (message: ChatMessage) => void;
  setMessages: (messages: ChatMessage[]) => void;
  toggleOpen: () => void;
  setLoading: (loading: boolean) => void;
  setSelectedText: (text: string | null) => void;
  setContextChapter: (chapter: string | null) => void;
  clearHistory: () => void;
  sendMessage: (content: string) => Promise<void>;
}

type ChatStore = ChatState & ChatActions;
```

---

# PART 2: BACKEND SPECIFICATIONS

## 2.1 FastAPI Application Structure

```
backend/
├── app/
│   ├── __init__.py
│   ├── main.py
│   ├── config.py
│   ├── database.py
│   ├── api/
│   │   ├── __init__.py
│   │   ├── v1/
│   │   │   ├── __init__.py
│   │   │   ├── router.py
│   │   │   ├── auth.py
│   │   │   ├── profile.py
│   │   │   ├── chat.py
│   │   │   ├── content.py
│   │   │   └── agents.py
│   │   └── deps.py
│   ├── core/
│   │   ├── __init__.py
│   │   ├── security.py
│   │   ├── auth.py
│   │   └── exceptions.py
│   ├── models/
│   │   ├── __init__.py
│   │   ├── user.py
│   │   ├── profile.py
│   │   ├── chat.py
│   │   └── content.py
│   ├── schemas/
│   │   ├── __init__.py
│   │   ├── user.py
│   │   ├── profile.py
│   │   ├── chat.py
│   │   ├── content.py
│   │   └── agent.py
│   ├── services/
│   │   ├── __init__.py
│   │   ├── auth_service.py
│   │   ├── profile_service.py
│   │   ├── chat_service.py
│   │   ├── content_service.py
│   │   ├── rag_service.py
│   │   └── agent_service.py
│   ├── agents/
│   │   ├── __init__.py
│   │   ├── base_agent.py
│   │   ├── rag_chat_agent.py
│   │   ├── summarizer_agent.py
│   │   ├── translator_agent.py
│   │   ├── personalizer_agent.py
│   │   └── content_generator_agent.py
│   └── utils/
│       ├── __init__.py
│       ├── embeddings.py
│       ├── qdrant.py
│       └── openai_client.py
├── alembic/
│   ├── versions/
│   ├── env.py
│   └── alembic.ini
├── tests/
│   ├── __init__.py
│   ├── conftest.py
│   ├── test_auth.py
│   ├── test_chat.py
│   └── test_agents.py
├── scripts/
│   ├── ingest_documents.py
│   ├── seed_database.py
│   └── create_embeddings.py
├── requirements.txt
├── Dockerfile
└── docker-compose.yml
```

## 2.2 Database Schema Specifications

### 2.2.1 Users Table
```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    name VARCHAR(255) NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    email_verified BOOLEAN DEFAULT FALSE,
    email_verified_at TIMESTAMP,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    last_login_at TIMESTAMP
);

CREATE INDEX idx_users_email ON users(email);
```

### 2.2.2 User Profiles Table
```sql
CREATE TABLE user_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID UNIQUE REFERENCES users(id) ON DELETE CASCADE,

    -- Software Background
    software_experience VARCHAR(20) DEFAULT 'none',
    programming_languages TEXT[] DEFAULT '{}',
    ros_experience BOOLEAN DEFAULT FALSE,
    linux_experience VARCHAR(20) DEFAULT 'none',

    -- Hardware Background
    hardware_experience VARCHAR(20) DEFAULT 'none',
    robotics_experience BOOLEAN DEFAULT FALSE,
    previous_projects TEXT,

    -- Equipment
    has_gpu_workstation BOOLEAN DEFAULT FALSE,
    gpu_model VARCHAR(100),
    has_jetson_kit BOOLEAN DEFAULT FALSE,
    jetson_model VARCHAR(100),
    has_robot_hardware BOOLEAN DEFAULT FALSE,
    robot_description TEXT,

    -- Preferences
    preferred_language VARCHAR(5) DEFAULT 'en',
    theme VARCHAR(10) DEFAULT 'system',
    notifications_enabled BOOLEAN DEFAULT TRUE,

    -- Progress
    current_chapter VARCHAR(100),
    completed_chapters TEXT[] DEFAULT '{}',

    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_user_profiles_user_id ON user_profiles(user_id);
```

### 2.2.3 Sessions Table
```sql
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    token VARCHAR(255) UNIQUE NOT NULL,
    refresh_token VARCHAR(255) UNIQUE NOT NULL,
    expires_at TIMESTAMP NOT NULL,
    refresh_expires_at TIMESTAMP NOT NULL,
    ip_address VARCHAR(45),
    user_agent TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_sessions_token ON sessions(token);
CREATE INDEX idx_sessions_user_id ON sessions(user_id);
```

### 2.2.4 Chat History Table
```sql
CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL, -- 'user' or 'assistant'
    content TEXT NOT NULL,
    sources JSONB, -- Array of citation objects
    context_chapter VARCHAR(100),
    selected_text TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_chat_messages_user_id ON chat_messages(user_id);
CREATE INDEX idx_chat_messages_created_at ON chat_messages(created_at);
```

### 2.2.5 Content Cache Table
```sql
CREATE TABLE content_cache (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chapter_id VARCHAR(100) NOT NULL,
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    cache_type VARCHAR(20) NOT NULL, -- 'personalized', 'translated'
    content TEXT NOT NULL,
    language VARCHAR(5),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    expires_at TIMESTAMP,

    UNIQUE(chapter_id, user_id, cache_type, language)
);

CREATE INDEX idx_content_cache_lookup ON content_cache(chapter_id, user_id, cache_type);
```

### 2.2.6 Learning Progress Table
```sql
CREATE TABLE learning_progress (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    chapter_id VARCHAR(100) NOT NULL,
    status VARCHAR(20) DEFAULT 'not_started', -- 'not_started', 'in_progress', 'completed'
    progress_percent INTEGER DEFAULT 0,
    time_spent INTEGER DEFAULT 0, -- seconds
    quiz_score INTEGER,
    completed_at TIMESTAMP,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    UNIQUE(user_id, chapter_id)
);

CREATE INDEX idx_learning_progress_user ON learning_progress(user_id);
```

## 2.3 API Endpoint Specifications

### 2.3.1 Authentication Endpoints

```python
# POST /api/v1/auth/signup
Request:
{
    "email": "user@example.com",
    "password": "securePassword123",
    "name": "Ahmed Khan"
}

Response (201):
{
    "user": {
        "id": "uuid",
        "email": "user@example.com",
        "name": "Ahmed Khan",
        "emailVerified": false
    },
    "session": {
        "token": "jwt_token",
        "expiresAt": "2025-12-12T12:00:00Z"
    },
    "requiresOnboarding": true
}

# POST /api/v1/auth/signin
Request:
{
    "email": "user@example.com",
    "password": "securePassword123"
}

Response (200):
{
    "user": { ... },
    "session": { ... }
}

# GET /api/v1/auth/session
Headers: Authorization: Bearer <token>

Response (200):
{
    "user": { ... },
    "session": { ... }
}

# POST /api/v1/auth/signout
Headers: Authorization: Bearer <token>

Response (200):
{
    "success": true
}
```

### 2.3.2 Profile Endpoints

```python
# GET /api/v1/profile
Headers: Authorization: Bearer <token>

Response (200):
{
    "id": "uuid",
    "userId": "uuid",
    "softwareExperience": "intermediate",
    "programmingLanguages": ["python", "cpp"],
    "rosExperience": false,
    "linuxExperience": "basic",
    "hardwareExperience": "hobbyist",
    "roboticsExperience": true,
    "previousProjects": "Built a line-following robot",
    "hasGpuWorkstation": true,
    "gpuModel": "RTX 3080",
    "hasJetsonKit": false,
    "jetsonModel": null,
    "hasRobotHardware": false,
    "robotDescription": null,
    "preferredLanguage": "en",
    "theme": "dark",
    "currentChapter": "chapter-2-3",
    "completedChapters": ["chapter-1-1", "chapter-1-2"]
}

# PUT /api/v1/profile/background
Headers: Authorization: Bearer <token>
Request:
{
    "softwareExperience": "intermediate",
    "programmingLanguages": ["python", "cpp"],
    "rosExperience": false,
    "linuxExperience": "basic",
    "hardwareExperience": "hobbyist",
    "roboticsExperience": true,
    "previousProjects": "Built a line-following robot",
    "hasGpuWorkstation": true,
    "gpuModel": "RTX 3080",
    "hasJetsonKit": false,
    "hasRobotHardware": false
}

Response (200):
{
    "success": true,
    "profile": { ... }
}
```

### 2.3.3 Chat Endpoints

```python
# POST /api/v1/chat
Headers: Authorization: Bearer <token>
Request:
{
    "message": "What is ROS 2?",
    "contextChapter": "chapter-1-1",
    "selectedText": null
}

Response (200):
{
    "id": "uuid",
    "role": "assistant",
    "content": "ROS 2 (Robot Operating System 2) is...",
    "sources": [
        {
            "chapterId": "chapter-1-1",
            "section": "Introduction to ROS 2",
            "relevance": 0.95
        }
    ],
    "timestamp": "2025-12-12T10:30:00Z"
}

# POST /api/v1/chat/context
Headers: Authorization: Bearer <token>
Request:
{
    "message": "Explain this in simpler terms",
    "selectedText": "The DDS middleware provides...",
    "contextChapter": "chapter-1-2"
}

Response (200):
{
    "id": "uuid",
    "role": "assistant",
    "content": "In simpler terms, DDS is like...",
    "sources": [...],
    "timestamp": "..."
}

# GET /api/v1/chat/history?limit=50&offset=0
Headers: Authorization: Bearer <token>

Response (200):
{
    "messages": [...],
    "total": 150,
    "hasMore": true
}
```

### 2.3.4 Content Endpoints

```python
# POST /api/v1/chapters/{chapter_id}/personalize
Headers: Authorization: Bearer <token>
Request:
{
    "content": "# Introduction to ROS 2\n\nROS 2 is..."
}

Response (200):
{
    "personalizedContent": "# Introduction to ROS 2\n\nSince you have experience with Python...",
    "adaptations": [
        "Added Python-specific examples",
        "Simplified Linux commands explanation",
        "Included beginner ROS tips"
    ],
    "cached": true
}

# POST /api/v1/chapters/{chapter_id}/translate
Headers: Authorization: Bearer <token>
Request:
{
    "content": "# Introduction to ROS 2\n\nROS 2 is...",
    "targetLanguage": "ur"
}

Response (200):
{
    "translatedContent": "# ROS 2 کا تعارف\n\nROS 2 ایک...",
    "preservedTerms": ["ROS 2", "DDS", "Node"],
    "cached": true
}

# GET /api/v1/chapters/{chapter_id}/summary
Headers: Authorization: Bearer <token>

Response (200):
{
    "summary": "This chapter covers the fundamentals of ROS 2...",
    "keyConceptds": [
        "ROS 2 Architecture",
        "Nodes and Topics",
        "DDS Middleware"
    ],
    "learningObjectives": [
        "Understand ROS 2 basics",
        "Set up a ROS 2 workspace",
        "Create your first node"
    ]
}
```

### 2.3.5 Agent Endpoints

```python
# POST /api/v1/agents/summarize
Headers: Authorization: Bearer <token>
Request:
{
    "content": "...",
    "summaryType": "detailed"
}

Response (200):
{
    "summary": "...",
    "keyPoints": [...],
    "agentUsed": "ChapterSummarizerAgent"
}

# POST /api/v1/agents/translate
Headers: Authorization: Bearer <token>
Request:
{
    "content": "...",
    "targetLanguage": "ur",
    "preserveTerms": ["ROS 2", "Node"]
}

Response (200):
{
    "translatedContent": "...",
    "preservedTerms": [...],
    "agentUsed": "UrduTranslatorAgent"
}

# POST /api/v1/agents/personalize
Headers: Authorization: Bearer <token>
Request:
{
    "content": "...",
    "chapterId": "chapter-1-1"
}

Response (200):
{
    "personalizedContent": "...",
    "adaptations": [...],
    "agentUsed": "PersonalizationAgent"
}

# POST /api/v1/agents/generate
Headers: Authorization: Bearer <token>
Request:
{
    "topic": "ROS 2 Publishers",
    "contentType": "quiz",
    "difficulty": "intermediate",
    "count": 5
}

Response (200):
{
    "generatedContent": {
        "questions": [...]
    },
    "agentUsed": "ContentGeneratorAgent"
}
```

## 2.4 RAG Pipeline Specifications

### 2.4.1 Document Ingestion Pipeline
```python
Pipeline Steps:
1. Load markdown files from Docusaurus content
2. Split into chunks (500 tokens, 50 token overlap)
3. Extract metadata (chapter, section, page)
4. Generate embeddings (OpenAI ada-002)
5. Store in Qdrant with metadata

Chunk Schema:
{
    "id": "uuid",
    "content": "chunk text...",
    "embedding": [0.1, 0.2, ...],  # 1536 dimensions
    "metadata": {
        "chapterId": "chapter-1-1",
        "moduleId": "module-1",
        "section": "Introduction",
        "pageNumber": 1,
        "wordCount": 150,
        "source": "docs/module-1/chapter-1.md"
    }
}

Qdrant Collection Config:
{
    "name": "physicalai_textbook",
    "vectors": {
        "size": 1536,
        "distance": "Cosine"
    }
}
```

### 2.4.2 Retrieval Pipeline
```python
Pipeline Steps:
1. Receive user query
2. Generate query embedding
3. Search Qdrant (top-k=5)
4. Re-rank results
5. Format context for LLM
6. Generate response with citations

Search Parameters:
{
    "limit": 5,
    "score_threshold": 0.7,
    "with_payload": true,
    "filter": {
        "must": [
            {"key": "chapterId", "match": {"value": context_chapter}}
        ]
    }
}
```

---

# PART 3: DOCUSAURUS BOOK SPECIFICATIONS

## 3.1 Docusaurus Structure

```
book/
├── docs/
│   ├── intro.md
│   ├── quickstart.md
│   ├── module-1-ros2/
│   │   ├── _category_.json
│   │   ├── chapter-1-intro.md
│   │   ├── chapter-2-nodes-topics.md
│   │   ├── chapter-3-actions-params.md
│   │   ├── chapter-4-launch-files.md
│   │   ├── chapter-5-custom-packages.md
│   │   └── chapter-6-lab.md
│   ├── module-2-simulation/
│   │   ├── _category_.json
│   │   ├── chapter-1-intro.md
│   │   ├── chapter-2-gazebo.md
│   │   ├── chapter-3-urdf.md
│   │   ├── chapter-4-unity.md
│   │   ├── chapter-5-sensors.md
│   │   └── chapter-6-lab.md
│   ├── module-3-isaac/
│   │   ├── _category_.json
│   │   ├── chapter-1-intro.md
│   │   ├── chapter-2-sim.md
│   │   ├── chapter-3-perception.md
│   │   ├── chapter-4-manipulation.md
│   │   ├── chapter-5-ros-integration.md
│   │   └── chapter-6-lab.md
│   └── module-4-vla/
│       ├── _category_.json
│       ├── chapter-1-intro.md
│       ├── chapter-2-vision.md
│       ├── chapter-3-language.md
│       ├── chapter-4-action.md
│       ├── chapter-5-systems.md
│       └── chapter-6-capstone.md
├── src/
│   ├── components/
│   │   ├── HomepageFeatures/
│   │   │   ├── index.tsx
│   │   │   └── styles.module.css
│   │   ├── ModuleCard/
│   │   │   └── index.tsx
│   │   ├── PersonalizeButton/
│   │   │   └── index.tsx
│   │   ├── TranslateButton/
│   │   │   └── index.tsx
│   │   └── ChatWidget/
│   │       └── index.tsx
│   ├── pages/
│   │   ├── index.tsx
│   │   └── index.module.css
│   ├── css/
│   │   └── custom.css
│   └── theme/
│       └── MDXComponents.tsx
├── static/
│   ├── img/
│   │   ├── logo.svg
│   │   ├── favicon.ico
│   │   └── modules/
│   │       ├── ros2.png
│   │       ├── simulation.png
│   │       ├── isaac.png
│   │       └── vla.png
│   └── fonts/
├── docusaurus.config.ts
├── sidebars.ts
├── package.json
└── tsconfig.json
```

## 3.2 Chapter Content Template

```markdown
---
sidebar_position: 1
title: "Chapter Title"
description: "Brief description for SEO"
keywords: [keyword1, keyword2, keyword3]
---

import PersonalizeButton from '@site/src/components/PersonalizeButton';
import TranslateButton from '@site/src/components/TranslateButton';

# Chapter Title

<div className="chapter-actions">
  <PersonalizeButton chapterId="chapter-id" />
  <TranslateButton chapterId="chapter-id" />
</div>

## Learning Objectives

By the end of this chapter, you will be able to:

- Objective 1
- Objective 2
- Objective 3

## Prerequisites

Before starting this chapter, ensure you have:

- [ ] Prerequisite 1
- [ ] Prerequisite 2
- [ ] Prerequisite 3

## Introduction

Opening paragraph that introduces the topic and its importance...

## Main Content

### Section 1

Content...

```python
# Code example
def example_function():
    pass
```

### Section 2

Content...

:::tip Pro Tip
Helpful tip for learners
:::

:::warning
Important warning
:::

:::info Hardware Requirement
Specific hardware needed for this section
:::

## Hands-on Lab

### Lab Objectives

- Lab objective 1
- Lab objective 2

### Lab Steps

1. Step 1
2. Step 2
3. Step 3

### Expected Output

Description of expected results...

## Knowledge Check

### Quiz

1. Question 1?
   - [ ] Option A
   - [ ] Option B
   - [x] Option C (correct)
   - [ ] Option D

2. Question 2?
   ...

## Summary

Key takeaways from this chapter...

## Further Reading

- [Resource 1](url)
- [Resource 2](url)

## Next Steps

Preview of what's coming in the next chapter...
```

## 3.3 Custom Components Specifications

### 3.3.1 PersonalizeButton Component
```typescript
// src/components/PersonalizeButton/index.tsx

interface PersonalizeButtonProps {
  chapterId: string;
}

// Functionality:
// 1. Check if user is authenticated
// 2. If not, show login prompt
// 3. If yes, call personalization API
// 4. Replace chapter content with personalized version
// 5. Show loading state during API call
// 6. Cache personalized content locally
// 7. Toggle back to original with "View Original" button
```

### 3.3.2 TranslateButton Component
```typescript
// src/components/TranslateButton/index.tsx

interface TranslateButtonProps {
  chapterId: string;
  targetLanguage?: 'ur' | 'en';
}

// Functionality:
// 1. Toggle between English and Urdu
// 2. Call translation API
// 3. Apply RTL styles for Urdu
// 4. Preserve code blocks and technical terms
// 5. Cache translations locally
// 6. Show loading state
```

### 3.3.3 ChatWidget Component
```typescript
// src/components/ChatWidget/index.tsx

// Functionality:
// 1. Floating chat button
// 2. Expandable chat window
// 3. Auto-detect current chapter context
// 4. Support text selection for "Ask about this"
// 5. Message history persistence
// 6. Markdown rendering in responses
// 7. Source citations with chapter links
```

---

# PART 4: AGENT SPECIFICATIONS

## 4.1 Base Agent Architecture

```python
# agents/base_agent.py

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional
from pydantic import BaseModel

class AgentInput(BaseModel):
    """Base input model for agents"""
    pass

class AgentOutput(BaseModel):
    """Base output model for agents"""
    success: bool
    error: Optional[str] = None

class BaseAgent(ABC):
    """Abstract base class for all agents"""

    name: str
    description: str
    version: str = "1.0.0"

    def __init__(self, config: Dict[str, Any] = None):
        self.config = config or {}
        self._initialize()

    @abstractmethod
    def _initialize(self) -> None:
        """Initialize agent-specific resources"""
        pass

    @abstractmethod
    async def execute(self, input: AgentInput) -> AgentOutput:
        """Execute the agent's main task"""
        pass

    @abstractmethod
    def get_tools(self) -> List[Dict[str, Any]]:
        """Return list of tools this agent can use"""
        pass

    def get_system_prompt(self) -> str:
        """Return the system prompt for this agent"""
        return f"You are {self.name}. {self.description}"
```

## 4.2 RAGChatAgent Specification

```python
# agents/rag_chat_agent.py

class RAGChatInput(AgentInput):
    query: str
    selected_text: Optional[str] = None
    chapter_context: Optional[str] = None
    chat_history: List[Dict[str, str]] = []

class Citation(BaseModel):
    chapter_id: str
    section: str
    relevance: float
    snippet: str

class RAGChatOutput(AgentOutput):
    answer: str
    sources: List[Citation]
    confidence: float

class RAGChatAgent(BaseAgent):
    name = "RAGChatAgent"
    description = "Answers questions about the Physical AI textbook using RAG"

    SYSTEM_PROMPT = """You are an expert AI assistant for the Physical AI & Humanoid Robotics textbook.

Your role is to:
1. Answer questions accurately based on the provided context
2. Cite your sources from the textbook
3. Explain complex concepts in an accessible way
4. Guide learners through difficult topics
5. Recommend relevant chapters for further reading

When answering:
- Always ground your answers in the provided context
- If the context doesn't contain enough information, say so
- Use code examples when helpful
- Be encouraging and supportive

If the user provides selected text, focus your answer on explaining that specific content.
"""

    def get_tools(self) -> List[Dict]:
        return [
            {
                "name": "search_textbook",
                "description": "Search the textbook for relevant content",
                "parameters": {
                    "query": {"type": "string", "description": "Search query"},
                    "chapter_filter": {"type": "string", "description": "Optional chapter ID to filter"}
                }
            }
        ]

    async def execute(self, input: RAGChatInput) -> RAGChatOutput:
        # 1. Search Qdrant for relevant chunks
        # 2. Format context with citations
        # 3. Generate response with OpenAI
        # 4. Extract and return citations
        pass
```

## 4.3 PersonalizationAgent Specification

```python
# agents/personalizer_agent.py

class PersonalizationInput(AgentInput):
    content: str
    chapter_id: str
    user_profile: Dict[str, Any]

class PersonalizationOutput(AgentOutput):
    personalized_content: str
    adaptations: List[str]

class PersonalizationAgent(BaseAgent):
    name = "PersonalizationAgent"
    description = "Adapts chapter content based on user background and experience"

    SYSTEM_PROMPT = """You are an expert educational content adapter.

Your role is to personalize textbook content based on the learner's background:

User Profile Considerations:
- Software Experience: {software_experience}
- Programming Languages: {languages}
- ROS Experience: {ros_experience}
- Linux Experience: {linux_experience}
- Hardware Experience: {hardware_experience}
- Has GPU Workstation: {has_gpu}
- Has Jetson Kit: {has_jetson}

Adaptation Guidelines:
1. For beginners: Add more explanations, simpler examples, more context
2. For advanced users: Skip basics, add advanced tips, more efficient approaches
3. Adjust code examples to match known languages
4. Add relevant warnings/tips based on equipment
5. Reference prior knowledge when applicable

Rules:
- Maintain all technical accuracy
- Keep the same overall structure
- Don't remove critical information
- Add helpful callouts for the user's level
- Include equipment-specific notes when relevant
"""

    async def execute(self, input: PersonalizationInput) -> PersonalizationOutput:
        # 1. Analyze user profile
        # 2. Determine adaptation strategy
        # 3. Generate personalized content
        # 4. List adaptations made
        pass
```

## 4.4 UrduTranslatorAgent Specification

```python
# agents/translator_agent.py

class TranslationInput(AgentInput):
    content: str
    preserve_terms: List[str] = []
    source_language: str = "en"
    target_language: str = "ur"

class TranslationOutput(AgentOutput):
    translated_content: str
    preserved_terms: List[str]

class UrduTranslatorAgent(BaseAgent):
    name = "UrduTranslatorAgent"
    description = "Translates content to Urdu while preserving technical terms"

    SYSTEM_PROMPT = """You are an expert Urdu translator specializing in technical content.

Your role is to translate educational content about robotics and AI from English to Urdu.

Translation Guidelines:
1. Maintain technical accuracy
2. Preserve these terms in English (do not translate):
   - Programming keywords (def, class, import, etc.)
   - Technology names (ROS 2, Gazebo, Isaac, etc.)
   - Standard abbreviations (API, SDK, GPU, etc.)
   - Code snippets (keep entirely in English)

3. Use proper Urdu technical vocabulary where established translations exist
4. Maintain markdown formatting
5. Keep code blocks unchanged
6. Preserve URLs and links
7. Use RTL-appropriate punctuation

Output should be natural, fluent Urdu suitable for Pakistani engineering students.
"""

    PRESERVE_TERMS = [
        "ROS 2", "ROS", "Gazebo", "Unity", "NVIDIA", "Isaac", "Isaac Sim",
        "VLA", "DDS", "URDF", "SDF", "Node", "Topic", "Service", "Action",
        "Publisher", "Subscriber", "Message", "Package", "Workspace",
        "Python", "C++", "Linux", "Ubuntu", "Docker", "Git", "GitHub",
        "API", "SDK", "GPU", "CPU", "CUDA", "TensorRT", "PyTorch",
        "RGB", "RGBD", "LiDAR", "IMU", "SLAM", "MoveIt", "Nav2"
    ]

    async def execute(self, input: TranslationInput) -> TranslationOutput:
        # 1. Extract code blocks and preserve
        # 2. Identify technical terms to preserve
        # 3. Translate remaining content
        # 4. Reassemble with preserved elements
        pass
```

## 4.5 ChapterSummarizerAgent Specification

```python
# agents/summarizer_agent.py

class SummaryInput(AgentInput):
    content: str
    summary_type: str = "detailed"  # "brief", "detailed", "bullets"

class SummaryOutput(AgentOutput):
    summary: str
    key_concepts: List[str]
    learning_objectives: List[str]

class ChapterSummarizerAgent(BaseAgent):
    name = "ChapterSummarizerAgent"
    description = "Generates concise summaries of textbook chapters"

    SYSTEM_PROMPT = """You are an expert at summarizing technical educational content.

Your role is to create helpful summaries of robotics and AI textbook chapters.

Summary Types:
- brief: 2-3 sentences capturing the main point
- detailed: Full paragraph with key information
- bullets: Bullet-point list of main takeaways

Always extract:
1. Key concepts covered
2. Learning objectives achieved
3. Prerequisites referenced
4. Practical applications mentioned

Keep summaries:
- Accurate to the source
- Accessible to learners
- Actionable (what can they do with this knowledge)
"""

    async def execute(self, input: SummaryInput) -> SummaryOutput:
        # 1. Parse chapter content
        # 2. Extract key sections
        # 3. Generate summary based on type
        # 4. Extract concepts and objectives
        pass
```

## 4.6 ContentGeneratorAgent Specification

```python
# agents/content_generator_agent.py

class GenerationInput(AgentInput):
    topic: str
    content_type: str  # "exercise", "quiz", "explanation", "code"
    difficulty: str = "intermediate"
    count: int = 5
    context: Optional[str] = None

class GenerationOutput(AgentOutput):
    generated_content: Dict[str, Any]
    content_type: str

class ContentGeneratorAgent(BaseAgent):
    name = "ContentGeneratorAgent"
    description = "Generates educational content like quizzes, exercises, and explanations"

    SYSTEM_PROMPT = """You are an expert educational content creator for robotics and AI.

Content Types:
1. quiz: Multiple choice questions with explanations
2. exercise: Hands-on coding or practical tasks
3. explanation: Detailed concept explanations
4. code: Working code examples with comments

Difficulty Levels:
- beginner: Basic concepts, simple examples, lots of guidance
- intermediate: Applied concepts, real scenarios, some complexity
- advanced: Complex problems, optimization, edge cases

All content must:
- Be technically accurate
- Include proper ROS 2 / Python / C++ syntax
- Reference actual tools and libraries
- Be testable/verifiable
"""

    async def execute(self, input: GenerationInput) -> GenerationOutput:
        # 1. Determine content structure
        # 2. Generate based on type
        # 3. Validate technical accuracy
        # 4. Format output
        pass
```

---

# PART 5: DEPLOYMENT SPECIFICATIONS

## 5.1 Environment Variables

```bash
# .env.example

# ===================
# Application
# ===================
NODE_ENV=development
APP_NAME=PhysicalAI-Textbook
APP_URL=http://localhost:3000

# ===================
# Database (Neon)
# ===================
DATABASE_URL=postgresql://user:password@host:5432/database?sslmode=require
DATABASE_POOL_SIZE=10

# ===================
# Authentication
# ===================
BETTER_AUTH_SECRET=your-secret-key-min-32-chars
BETTER_AUTH_URL=http://localhost:3000
JWT_EXPIRES_IN=15m
REFRESH_TOKEN_EXPIRES_IN=7d

# ===================
# OAuth Providers
# ===================
GOOGLE_CLIENT_ID=your-google-client-id
GOOGLE_CLIENT_SECRET=your-google-client-secret
GITHUB_CLIENT_ID=your-github-client-id
GITHUB_CLIENT_SECRET=your-github-client-secret

# ===================
# OpenAI
# ===================
OPENAI_API_KEY=sk-your-openai-api-key
OPENAI_MODEL=gpt-4-turbo-preview
OPENAI_EMBEDDING_MODEL=text-embedding-ada-002

# ===================
# Qdrant
# ===================
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION=physicalai_textbook

# ===================
# Backend API
# ===================
API_URL=http://localhost:8000
API_V1_PREFIX=/api/v1
CORS_ORIGINS=http://localhost:3000,http://localhost:3001

# ===================
# Redis (Optional)
# ===================
REDIS_URL=redis://localhost:6379

# ===================
# Rate Limiting
# ===================
RATE_LIMIT_REQUESTS=100
RATE_LIMIT_WINDOW=60

# ===================
# Logging
# ===================
LOG_LEVEL=INFO
```

## 5.2 Docker Configuration

```dockerfile
# backend/Dockerfile

FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    libpq-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application
COPY . .

# Run application
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

```yaml
# docker-compose.yml

version: '3.8'

services:
  frontend:
    build:
      context: ./frontend
      dockerfile: Dockerfile
    ports:
      - "3000:3000"
    environment:
      - NODE_ENV=development
    depends_on:
      - backend
    volumes:
      - ./frontend:/app
      - /app/node_modules

  backend:
    build:
      context: ./backend
      dockerfile: Dockerfile
    ports:
      - "8000:8000"
    environment:
      - DATABASE_URL=${DATABASE_URL}
      - OPENAI_API_KEY=${OPENAI_API_KEY}
      - QDRANT_URL=${QDRANT_URL}
      - QDRANT_API_KEY=${QDRANT_API_KEY}
    depends_on:
      - postgres
      - qdrant

  docusaurus:
    build:
      context: ./book
      dockerfile: Dockerfile
    ports:
      - "3001:3000"
    volumes:
      - ./book:/app
      - /app/node_modules

  postgres:
    image: postgres:15
    environment:
      - POSTGRES_DB=physicalai
      - POSTGRES_USER=postgres
      - POSTGRES_PASSWORD=postgres
    ports:
      - "5432:5432"
    volumes:
      - postgres_data:/var/lib/postgresql/data

  qdrant:
    image: qdrant/qdrant:latest
    ports:
      - "6333:6333"
    volumes:
      - qdrant_data:/qdrant/storage

volumes:
  postgres_data:
  qdrant_data:
```

## 5.3 Vercel Configuration (Frontend)

```json
// vercel.json

{
  "buildCommand": "npm run build",
  "outputDirectory": ".next",
  "framework": "nextjs",
  "regions": ["iad1"],
  "env": {
    "NEXT_PUBLIC_API_URL": "@api_url",
    "NEXT_PUBLIC_BOOK_URL": "@book_url"
  },
  "headers": [
    {
      "source": "/api/(.*)",
      "headers": [
        { "key": "Access-Control-Allow-Credentials", "value": "true" },
        { "key": "Access-Control-Allow-Origin", "value": "*" },
        { "key": "Access-Control-Allow-Methods", "value": "GET,POST,PUT,DELETE,OPTIONS" },
        { "key": "Access-Control-Allow-Headers", "value": "Content-Type, Authorization" }
      ]
    }
  ]
}
```

## 5.4 GitHub Pages Configuration (Docusaurus)

```yaml
# .github/workflows/deploy-docs.yml

name: Deploy Docusaurus to GitHub Pages

on:
  push:
    branches:
      - main
    paths:
      - 'book/**'

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: '20'
          cache: 'npm'
          cache-dependency-path: book/package-lock.json

      - name: Install dependencies
        working-directory: ./book
        run: npm ci

      - name: Build
        working-directory: ./book
        run: npm run build
        env:
          NEXT_PUBLIC_API_URL: ${{ secrets.API_URL }}

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./book/build
```

## 5.5 Railway Configuration (Backend)

```toml
# railway.toml

[build]
builder = "DOCKERFILE"
dockerfilePath = "backend/Dockerfile"

[deploy]
startCommand = "uvicorn app.main:app --host 0.0.0.0 --port $PORT"
healthcheckPath = "/health"
healthcheckTimeout = 100
restartPolicyType = "ON_FAILURE"
restartPolicyMaxRetries = 10
```

---

# PART 6: TESTING SPECIFICATIONS

## 6.1 Frontend Testing

```typescript
// Frontend test structure
frontend/
├── __tests__/
│   ├── components/
│   │   ├── auth/
│   │   │   ├── signin-form.test.tsx
│   │   │   ├── signup-form.test.tsx
│   │   │   └── background-questions.test.tsx
│   │   ├── chat/
│   │   │   ├── chat-widget.test.tsx
│   │   │   └── chat-message.test.tsx
│   │   └── content/
│   │       ├── personalize-button.test.tsx
│   │       └── translate-button.test.tsx
│   ├── hooks/
│   │   ├── use-auth.test.ts
│   │   └── use-chat.test.ts
│   └── utils/
│       └── validators.test.ts
├── jest.config.js
└── jest.setup.js
```

## 6.2 Backend Testing

```python
# Backend test structure
backend/
├── tests/
│   ├── conftest.py
│   ├── test_auth.py
│   ├── test_profile.py
│   ├── test_chat.py
│   ├── test_content.py
│   ├── test_agents/
│   │   ├── test_rag_agent.py
│   │   ├── test_personalizer.py
│   │   ├── test_translator.py
│   │   └── test_summarizer.py
│   └── test_integration/
│       ├── test_auth_flow.py
│       └── test_chat_flow.py
├── pytest.ini
└── .coveragerc
```

---

## SPECIFICATION COMPLETE

```
Document: SP.SPECS
Version: 1.0.0
Created: 2025-12-12
Total Pages: ~50 equivalent
Coverage: Frontend, Backend, Database, API, Agents, Deployment, Testing
```
