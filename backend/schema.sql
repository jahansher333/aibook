-- Neon Postgres schema for RAG chatbot

-- Sessions table
CREATE TABLE IF NOT EXISTS sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID DEFAULT NULL,
    created_at TIMESTAMP NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP NOT NULL DEFAULT NOW(),
    last_message_at TIMESTAMP DEFAULT NULL,
    total_messages INTEGER NOT NULL DEFAULT 0,
    CONSTRAINT check_total_messages CHECK (total_messages >= 0)
);

CREATE INDEX IF NOT EXISTS idx_sessions_created_at ON sessions(created_at DESC);
CREATE INDEX IF NOT EXISTS idx_sessions_last_message_at ON sessions(last_message_at DESC);

-- Messages table
CREATE TABLE IF NOT EXISTS messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant', 'system')),
    content TEXT NOT NULL,
    citations JSONB DEFAULT '[]'::jsonb,
    timestamp TIMESTAMP NOT NULL DEFAULT NOW(),
    token_count INTEGER DEFAULT NULL,
    latency_ms INTEGER DEFAULT NULL,
    CONSTRAINT check_token_count CHECK (token_count IS NULL OR token_count > 0),
    CONSTRAINT check_latency CHECK (latency_ms IS NULL OR latency_ms >= 0)
);

CREATE INDEX IF NOT EXISTS idx_messages_session_id ON messages(session_id, timestamp ASC);
CREATE INDEX IF NOT EXISTS idx_messages_timestamp ON messages(timestamp DESC);

-- Queries table (for analytics)
CREATE TABLE IF NOT EXISTS queries (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    query TEXT NOT NULL,
    selected_text TEXT DEFAULT NULL,
    retrieval_time_ms INTEGER NOT NULL,
    llm_time_ms INTEGER NOT NULL,
    total_time_ms INTEGER NOT NULL,
    top_chunks JSONB NOT NULL,
    max_similarity FLOAT DEFAULT NULL,
    success BOOLEAN NOT NULL,
    error_message TEXT DEFAULT NULL,
    timestamp TIMESTAMP NOT NULL DEFAULT NOW(),
    CONSTRAINT check_retrieval_time CHECK (retrieval_time_ms >= 0),
    CONSTRAINT check_llm_time CHECK (llm_time_ms >= 0)
);

CREATE INDEX IF NOT EXISTS idx_queries_session_id ON queries(session_id, timestamp ASC);
CREATE INDEX IF NOT EXISTS idx_queries_success ON queries(success);
CREATE INDEX IF NOT EXISTS idx_queries_max_similarity ON queries(max_similarity DESC);

-- Feedback table
CREATE TABLE IF NOT EXISTS feedback (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID NOT NULL REFERENCES messages(id) ON DELETE CASCADE,
    feedback VARCHAR(20) NOT NULL CHECK (feedback IN ('helpful', 'unhelpful')),
    timestamp TIMESTAMP NOT NULL DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_feedback_message_id ON feedback(message_id);

-- Ingestion metadata (for tracking book versions)
CREATE TABLE IF NOT EXISTS ingestion_metadata (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    collection_name VARCHAR(255) NOT NULL,
    total_chunks INTEGER NOT NULL,
    total_tokens INTEGER NOT NULL,
    ingestion_timestamp TIMESTAMP NOT NULL DEFAULT NOW()
);
