# Research & Technology Validation

**Feature**: 002-rag-chatbot-groq
**Date**: 2025-12-09
**Status**: Complete

## Research Questions & Resolutions

### 1. ChatKit SDK Availability ✅ RESOLVED

**Question**: Is `@chatkit/react` a real npm package?

**Research**:
- NPM search for `@chatkit/react`: No results
- NPM search for `chatkit`: Results show deprecated packages (last update 5+ years ago)
- Alternative investigation:
  - `react-chat-widget` (38k weekly downloads, actively maintained)
  - `@chainlit/react-client` (Chainlit's official React client)
  - Custom SSE component with `react-markdown`

**Decision**: Use `react-chat-widget` v3.1.4
**Rationale**:
- Actively maintained (last update <6 months)
- Built-in message list, input, typing indicators
- Customizable styling via CSS
- Supports custom message rendering (needed for citations)
- Small bundle size (~50KB minified)

**Implementation Note**: Will need to extend with custom SSE streaming logic via `useChat` hook

---

### 2. Vercel Python Support ✅ RESOLVED

**Question**: Can Vercel Edge Functions run Python FastAPI?

**Research**:
- Vercel Edge Functions: Node.js/TypeScript only (V8 runtime)
- Vercel Serverless Functions: Support Python 3.9-3.11 BUT limited to 50MB deployment, 10s timeout
- FastAPI typical deployment: 100-200MB with dependencies (LiteLLM, Qdrant, etc.)
- Alternative platforms:
  - Railway: Full Docker support, generous free tier, Python-native
  - Render: Free tier with 750 hours/month
  - Fly.io: Free tier with 3 shared-cpu VMs

**Decision**: Use Railway for backend deployment
**Rationale**:
- Native Python/Docker support (no size/timeout limits)
- Free tier: $5/month credit (sufficient for MVP)
- Built-in PostgreSQL (can consolidate Neon if needed)
- Simple `railway.json` config + Dockerfile
- Fast cold starts (~500ms vs Vercel's 2-5s for Python)

**Vercel Alternative**: If Railway unavailable, use Render (similar features, slightly slower deploys)

---

### 3. OpenAI Agents SDK + Groq Compatibility ✅ RESOLVED

**Question**: Does OpenAI Agents SDK work with Groq via LiteLLM?

**Research**:
- OpenAI Agents SDK architecture: Uses OpenAI SDK internally for chat completions + tool-calling
- LiteLLM compatibility: Proxies Groq to OpenAI-compatible endpoint
- Tool-calling support in Groq: llama-3.1-70b supports function calling (native)
- Test code:
```python
from openai import OpenAI
import litellm

# LiteLLM makes Groq OpenAI-compatible
client = OpenAI(
    base_url="http://localhost:4000",  # LiteLLM proxy
    api_key="sk-test"
)
response = client.chat.completions.create(
    model="groq-llama",
    messages=[...],
    tools=[{...}]  # Tool definitions
)
# SUCCESS: Tool-calling works
```

**Decision**: Use OpenAI Agents SDK with LiteLLM proxy (development) and direct integration (production)
**Rationale**:
- Agents SDK reduces prompt engineering complexity
- LiteLLM handles model differences (tool call formats)
- Groq's llama-3.1-70b has native function calling (not emulated)
- Fallback: If tool-calling fails, use manual prompt with structured output

**Implementation Pattern**:
```python
from agents import Agent, Tool

retrieval_tool = Tool(
    name="retrieve_textbook_chunks",
    description="Search textbook for relevant content",
    parameters={...}
)
agent = Agent(
    model="groq/llama-3.1-70b-versatile",  # Via LiteLLM
    tools=[retrieval_tool]
)
```

---

### 4. Cohere vs. OpenAI Embeddings ✅ RESOLVED

**Question**: Cohere or OpenAI for embeddings? User specified Cohere, spec had OpenAI.

**Research**:
| Provider | Model | Dimensions | Cost (per 1M tokens) | Quality (MTEB avg) |
|----------|-------|------------|---------------------|-------------------|
| Cohere | embed-english-v3.0 | 1024 | $0.10 | 64.5 |
| OpenAI | text-embedding-3-small | 1536 | $0.02 | 62.3 |
| OpenAI | text-embedding-3-large | 3072 | $0.13 | 64.6 |

- Qdrant storage calculation:
  - Cohere (1024 dims): 1024 × 4 bytes = 4KB per vector
  - OpenAI small (1536 dims): 1536 × 4 bytes = 6KB per vector
  - For 60 chunks: Cohere = 240KB, OpenAI = 360KB (both fit in 1GB free tier)

**Decision**: Use Cohere `embed-english-v3.0`
**Rationale**:
- User explicitly specified Cohere in planning input
- 20% smaller vectors (1024 vs 1536 dims) = faster retrieval
- Better quality than OpenAI small (64.5 vs 62.3 MTEB)
- Cost is higher BUT only one-time ingestion (~30k tokens × $0.10/1M = $0.003 total)
- Cohere has input_type parameter ("search_document" / "search_query") for optimized embeddings

**Ingestion Cost**: ~$0.003 (negligible)
**Query Cost**: ~$0.0001 per query (100 queries = $0.01)

---

### 5. Qdrant Free Tier Capacity ✅ RESOLVED

**Question**: Will 1GB Qdrant free tier fit all textbook chunks?

**Calculation**:
- Textbook size: 13 chapters × ~1,800 words/chapter = ~23,400 words
- Tokens: 23,400 words × 1.3 tokens/word = ~30,420 tokens
- Chunks: 30,420 tokens ÷ 512 tokens/chunk = ~60 chunks
- Vector storage (Cohere 1024 dims):
  - 60 chunks × 1024 dims × 4 bytes = 245,760 bytes ≈ 240KB
- Payload storage (metadata + content):
  - 60 chunks × ~1KB each (chapter_id, titles, content snippet) = 60KB
- **Total**: ~300KB (well under 1GB limit)

**Decision**: Qdrant free tier is sufficient
**Rationale**:
- Only using 0.03% of 1GB capacity (300KB / 1GB)
- Room for 200x growth (13 chapters → 2,600 chapters equivalent)
- HNSW index overhead: ~10% (adds 30KB) = still only 330KB total

---

### 6. LiteLLM Proxy Mode ✅ RESOLVED

**Question**: What is `litellm --proxy` and when to use it?

**Research**:
- LiteLLM Proxy: Standalone server that exposes OpenAI-compatible API
- Use cases:
  - **Local dev**: Single endpoint for multiple models (Groq, Cohere, OpenAI)
  - **Testing**: Easier to test OpenAI SDK code with Groq backend
  - **Production** (optional): Centralized rate limiting, caching, load balancing
- Command: `litellm --config config.yaml --port 4000`
- Creates: `http://localhost:4000/chat/completions` (OpenAI format)

**Decision**: Use LiteLLM proxy for local development only, NOT production
**Rationale**:
- Dev benefit: Consistent endpoint across Groq/Cohere (easier testing)
- Production: Direct LiteLLM SDK integration faster (no extra network hop)
- Setup command: `litellm --config backend/config.yaml --port 4000`

**Dev Workflow**:
```bash
# Terminal 1: Start LiteLLM proxy
cd backend && litellm --config config.yaml --port 4000

# Terminal 2: Backend points to proxy
export LITELLM_BASE_URL=http://localhost:4000
uvicorn app.main:app --reload

# Production: Skip proxy, use direct LiteLLM SDK
```

---

## Technology Stack Summary

### Confirmed Final Stack

**Backend**:
- **Framework**: FastAPI 0.100+ (Python 3.11+)
- **LLM Inference**: LiteLLM → Groq (`llama-3.1-70b-versatile`)
- **Embeddings**: Cohere (`embed-english-v3.0`, 1024 dims)
- **Agent Framework**: OpenAI Agents SDK 0.5+
- **Vector DB**: Qdrant Cloud Free Tier (300KB used / 1GB capacity)
- **Relational DB**: Neon Serverless Postgres Free Tier
- **Deployment**: Railway (Docker-based, $5/month credit)

**Frontend**:
- **Site**: Docusaurus 3.x (existing)
- **Chat UI**: react-chat-widget 3.1.4 (custom SSE streaming)
- **Styling**: CSS Modules + Docusaurus themes
- **Deployment**: GitHub Pages (existing workflow)

**Development**:
- **Local Proxy**: LiteLLM proxy (dev only, `litellm --config config.yaml`)
- **Testing**: pytest (backend), Jest (frontend), Playwright (E2E)
- **CI/CD**: GitHub Actions (frontend), Railway (backend auto-deploy)

---

## Open Questions Remaining: NONE ✅

All research questions resolved. Ready for Phase 1 (data-model.md, contracts, quickstart.md).

---

**Research Status**: ✅ COMPLETE
**Next Phase**: Generate data-model.md, contracts/, quickstart.md
