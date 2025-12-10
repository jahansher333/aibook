# Data Model: RAG Chatbot

**Feature**: 002-rag-chatbot-groq
**Date**: 2025-12-09

## Neon Postgres Schema

### Tables

#### sessions
Conversation threads for tracking user interactions.

```sql
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID DEFAULT NULL,  -- Nullable for MVP (no auth)
    created_at TIMESTAMP NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP NOT NULL DEFAULT NOW(),
    last_message_at TIMESTAMP DEFAULT NULL,
    total_messages INTEGER NOT NULL DEFAULT 0,
    CONSTRAINT check_total_messages CHECK (total_messages >= 0)
);

CREATE INDEX idx_sessions_created_at ON sessions(created_at DESC);
CREATE INDEX idx_sessions_last_message_at ON sessions(last_message_at DESC);
```

#### messages
Individual chat messages within sessions.

```sql
CREATE TABLE messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant', 'system')),
    content TEXT NOT NULL,
    citations JSONB DEFAULT '[]'::jsonb,  -- Array of {chapter_id, similarity}
    timestamp TIMESTAMP NOT NULL DEFAULT NOW(),
    token_count INTEGER DEFAULT NULL,
    latency_ms INTEGER DEFAULT NULL,  -- Time to first token (for assistant messages)
    CONSTRAINT check_token_count CHECK (token_count IS NULL OR token_count > 0),
    CONSTRAINT check_latency CHECK (latency_ms IS NULL OR latency_ms >= 0)
);

CREATE INDEX idx_messages_session_id ON messages(session_id, timestamp ASC);
CREATE INDEX idx_messages_timestamp ON messages(timestamp DESC);
```

#### queries
Analytics table for query performance tracking.

```sql
CREATE TABLE queries (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    query TEXT NOT NULL,
    selected_text TEXT DEFAULT NULL,  -- For context-aware queries
    retrieval_time_ms INTEGER NOT NULL,
    llm_time_ms INTEGER NOT NULL,
    total_time_ms INTEGER NOT NULL,
    top_chunks JSONB NOT NULL,  -- Array of {chunk_id, similarity, chapter_id}
    max_similarity FLOAT DEFAULT NULL,  -- Highest similarity score
    success BOOLEAN NOT NULL,
    error_message TEXT DEFAULT NULL,
    timestamp TIMESTAMP NOT NULL DEFAULT NOW(),
    CONSTRAINT check_retrieval_time CHECK (retrieval_time_ms >= 0),
    CONSTRAINT check_llm_time CHECK (llm_time_ms >= 0),
    CONSTRAINT check_total_time CHECK (total_time_ms = retrieval_time_ms + llm_time_ms)
);

CREATE INDEX idx_queries_session_id ON queries(session_id, timestamp ASC);
CREATE INDEX idx_queries_success ON queries(success);
CREATE INDEX idx_queries_max_similarity ON queries(max_similarity DESC);
CREATE INDEX idx_queries_total_time ON queries(total_time_ms ASC);
```

#### feedback (Optional - for SC-006)
User feedback on message quality.

```sql
CREATE TABLE feedback (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID NOT NULL REFERENCES messages(id) ON DELETE CASCADE,
    feedback VARCHAR(20) NOT NULL CHECK (feedback IN ('helpful', 'unhelpful')),
    timestamp TIMESTAMP NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_feedback_message_id ON feedback(message_id);
CREATE INDEX idx_feedback_timestamp ON feedback(timestamp DESC);
```

---

## Qdrant Vector Database

### Collection: textbook_chunks

**Configuration**:
```python
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance

client.create_collection(
    collection_name="textbook_chunks",
    vectors_config=VectorParams(
        size=1024,  # Cohere embed-english-v3.0
        distance=Distance.COSINE
    )
)
```

**Vector Point Structure**:
```json
{
  "id": "ch01-section1-chunk0",
  "vector": [0.1, 0.2, ...],  // 1024 dimensions
  "payload": {
    "chapter_id": "ch01",
    "chapter_title": "Introduction to Physical AI",
    "section_title": "What is Physical AI?",
    "content": "Physical AI refers to...",
    "token_count": 487,
    "metadata": {
      "chunk_index": 0,
      "heading_level": 2,
      "has_code": false,
      "has_diagram": true
    }
  }
}
```

**Index Type**: HNSW (Hierarchical Navigable Small World)
- **m**: 16 (number of edges per node)
- **ef_construct**: 100 (search depth during indexing)
- **ef_search**: 64 (search depth during querying)

---

## Entity Relationships

```
sessions (1) ----< (many) messages
sessions (1) ----< (many) queries
messages (1) ----< (many) feedback

textbook_chunks (Qdrant) referenced by:
  - messages.citations (JSONB array)
  - queries.top_chunks (JSONB array)
```

**Relationship Notes**:
- **CASCADE DELETE**: Deleting a session removes all associated messages, queries
- **No FK to Qdrant**: chunk_id stored as string, validated at application layer
- **No users table**: MVP has no auth, user_id always NULL

---

## Data Access Patterns

### Write Patterns

1. **New Conversation**:
```sql
INSERT INTO sessions (id) VALUES (gen_random_uuid()) RETURNING id;
```

2. **User Query**:
```sql
-- Insert user message
INSERT INTO messages (session_id, role, content)
VALUES (:session_id, 'user', :query);

-- Log query metrics
INSERT INTO queries (session_id, query, retrieval_time_ms, llm_time_ms, total_time_ms, top_chunks, success)
VALUES (:session_id, :query, :ret_time, :llm_time, :total, :chunks, true);
```

3. **Assistant Response**:
```sql
INSERT INTO messages (session_id, role, content, citations, token_count, latency_ms)
VALUES (:session_id, 'assistant', :response, :citations, :tokens, :latency);

-- Update session metadata
UPDATE sessions
SET last_message_at = NOW(), total_messages = total_messages + 2
WHERE id = :session_id;
```

### Read Patterns

1. **Retrieve Conversation History**:
```sql
SELECT m.* FROM messages m
WHERE m.session_id = :session_id
ORDER BY m.timestamp ASC
LIMIT 50;
```

2. **Performance Analytics** (SC-001):
```sql
-- 95th percentile latency
SELECT PERCENTILE_CONT(0.95) WITHIN GROUP (ORDER BY llm_time_ms) AS p95_latency
FROM queries
WHERE timestamp >= NOW() - INTERVAL '24 hours';
```

3. **Citation Quality** (SC-002):
```sql
-- Percentage with high-quality citations
SELECT AVG(CASE WHEN max_similarity > 0.7 THEN 1.0 ELSE 0.0 END) AS citation_quality
FROM queries
WHERE success = true;
```

4. **Reliability** (SC-003):
```sql
-- Session success rate (5+ messages)
WITH session_stats AS (
    SELECT session_id, COUNT(*) as msg_count, AVG(CASE WHEN success THEN 1.0 ELSE 0.0 END) as success_rate
    FROM queries
    GROUP BY session_id
    HAVING COUNT(*) >= 5
)
SELECT AVG(success_rate) FROM session_stats;
```

---

## Storage Estimates

**Neon Postgres** (0.5GB free tier):
- **sessions**: 100 bytes × 1,000 sessions = 100KB
- **messages**: 500 bytes × 5,000 messages = 2.5MB
- **queries**: 1KB × 5,000 queries = 5MB
- **feedback**: 100 bytes × 2,000 feedbacks = 200KB
- **Total**: ~7.8MB (1.6% of 0.5GB)

**Qdrant** (1GB free tier):
- **Vectors**: 60 chunks × 4KB = 240KB
- **Payload**: 60 chunks × 1KB = 60KB
- **Index overhead**: 10% = 30KB
- **Total**: ~330KB (0.03% of 1GB)

**Growth Capacity**:
- Neon: Can handle ~64x growth (1,000 → 64,000 sessions)
- Qdrant: Can handle ~3,000x growth (60 → 180,000 chunks)

---

## Validation Rules

**Application Layer**:
1. **session_id**: Must exist before creating messages
2. **chunk_id** in citations: Must match Qdrant collection
3. **similarity**: Must be between 0.0 and 1.0
4. **timestamps**: Must be <= NOW()
5. **token_count**: Must match tiktoken calculation
6. **latency_ms**: Only non-NULL for assistant messages

**Database Constraints**:
- CHECK constraints on enums (role, feedback)
- CHECK constraints on non-negative integers
- Foreign keys with CASCADE DELETE

---

**Data Model Status**: ✅ COMPLETE
**Next Artifact**: contracts/openapi.yaml
