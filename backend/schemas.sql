-- =============================================================================
-- NEON SERVERLESS POSTGRES SCHEMA
-- Physical AI Textbook RAG Chatbot
-- =============================================================================

-- Enable UUID extension
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- =============================================================================
-- USERS TABLE
-- =============================================================================
CREATE TABLE IF NOT EXISTS users (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    email VARCHAR(255) UNIQUE NOT NULL,
    name VARCHAR(255),
    password_hash VARCHAR(255) NOT NULL,
    is_active BOOLEAN DEFAULT true,
    is_verified BOOLEAN DEFAULT false,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX idx_users_email ON users(email);

-- =============================================================================
-- USER PROFILES TABLE
-- =============================================================================
CREATE TABLE IF NOT EXISTS user_profiles (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID UNIQUE NOT NULL REFERENCES users(id) ON DELETE CASCADE,

    -- Background questions
    programming_experience VARCHAR(50), -- 'beginner', 'intermediate', 'advanced'
    robotics_experience VARCHAR(50),
    preferred_language VARCHAR(50), -- 'python', 'cpp', 'both'
    learning_goals TEXT[],

    -- Preferences
    theme VARCHAR(20) DEFAULT 'light',
    notifications_enabled BOOLEAN DEFAULT true,

    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX idx_user_profiles_user_id ON user_profiles(user_id);

-- =============================================================================
-- CHAT MESSAGES TABLE (Session-based memory)
-- =============================================================================
CREATE TABLE IF NOT EXISTS chat_messages (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,

    -- Message content
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,

    -- RAG metadata
    sources JSONB, -- Array of citation objects
    context_chapter VARCHAR(100),
    selected_text TEXT, -- For selected text queries

    -- Timestamps
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX idx_chat_messages_user_id ON chat_messages(user_id);
CREATE INDEX idx_chat_messages_created_at ON chat_messages(created_at DESC);
CREATE INDEX idx_chat_messages_user_created ON chat_messages(user_id, created_at DESC);

-- =============================================================================
-- SESSIONS TABLE
-- =============================================================================
CREATE TABLE IF NOT EXISTS sessions (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token VARCHAR(500) NOT NULL,
    refresh_token VARCHAR(500),
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX idx_sessions_user_id ON sessions(user_id);
CREATE INDEX idx_sessions_token ON sessions(token);

-- =============================================================================
-- CONTENT CACHE TABLE (For personalized/translated content)
-- =============================================================================
CREATE TABLE IF NOT EXISTS content_cache (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,

    -- Cache key
    cache_type VARCHAR(50) NOT NULL, -- 'summary', 'translation', 'personalized'
    chapter_id VARCHAR(100) NOT NULL,

    -- Cached content
    content TEXT NOT NULL,
    metadata JSONB,

    -- TTL
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    expires_at TIMESTAMP WITH TIME ZONE
);

CREATE INDEX idx_content_cache_lookup ON content_cache(user_id, cache_type, chapter_id);

-- =============================================================================
-- LEARNING PROGRESS TABLE
-- =============================================================================
CREATE TABLE IF NOT EXISTS learning_progress (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,

    chapter_id VARCHAR(100) NOT NULL,
    module_id VARCHAR(100) NOT NULL,

    -- Progress tracking
    progress_percent INTEGER DEFAULT 0 CHECK (progress_percent >= 0 AND progress_percent <= 100),
    completed_at TIMESTAMP WITH TIME ZONE,
    time_spent_seconds INTEGER DEFAULT 0,

    -- Quiz/exercise data
    quiz_scores JSONB,

    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),

    UNIQUE(user_id, chapter_id)
);

CREATE INDEX idx_learning_progress_user ON learning_progress(user_id);

-- =============================================================================
-- EXAMPLE SOURCES JSONB STRUCTURE
-- =============================================================================
-- The `sources` column in chat_messages stores citation data:
-- [
--   {
--     "chapter_id": "chapter-1-1",
--     "section": "Introduction to ROS 2",
--     "relevance": 0.92,
--     "snippet": "ROS 2 is the next generation..."
--   }
-- ]

-- =============================================================================
-- HELPFUL QUERIES
-- =============================================================================

-- Get user's recent chat history with context
-- SELECT * FROM chat_messages
-- WHERE user_id = $1
-- ORDER BY created_at DESC
-- LIMIT 10;

-- Get chat history for a specific chapter context
-- SELECT * FROM chat_messages
-- WHERE user_id = $1 AND context_chapter = $2
-- ORDER BY created_at DESC;

-- Get user's learning progress across all chapters
-- SELECT lp.*, u.email
-- FROM learning_progress lp
-- JOIN users u ON u.id = lp.user_id
-- WHERE lp.user_id = $1
-- ORDER BY lp.module_id, lp.chapter_id;
