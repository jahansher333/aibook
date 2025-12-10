# Quickstart Guide: RAG Chatbot Development

**Feature**: 002-rag-chatbot-groq
**Date**: 2025-12-09

## Prerequisites

**Software**:
- Python 3.11+ (`python --version`)
- Node.js 18+ (`node --version`)
- Git (`git --version`)

**API Keys** (sign up for free tiers):
1. **Groq API**: https://console.groq.com → Create API key
2. **Cohere API**: https://dashboard.cohere.com → API Keys
3. **Qdrant Cloud**: https://cloud.qdrant.io → Create cluster → Get API key
4. **Neon Postgres**: https://console.neon.tech → Create project → Copy connection string

---

## Backend Setup (FastAPI)

### 1. Clone and Navigate
```bash
git clone <repo-url>
cd backend
```

### 2. Create Virtual Environment
```bash
# Create venv
python3.11 -m venv venv

# Activate (Linux/Mac)
source venv/bin/activate

# Activate (Windows)
venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

**Expected packages**:
- FastAPI 0.100+
- LiteLLM 1.x
- OpenAI SDK 1.x (for Agents SDK)
- Qdrant Client 1.7+
- Psycopg3 (Neon)
- Uvicorn (ASGI server)

### 4. Configure Environment Variables
Create `.env` file in `backend/` directory:

```bash
# .env
GROQ_API_KEY=gsk_xxxxxxxxxxxxxxxxxxxx
COHERE_API_KEY=xxxxxxxxxxxxxxxxxxxxxxxx
QDRANT_URL=https://xxxxxxxx-xxxxx-xxxxx-xxxxx-xxxxxxxxxxxx.us-east.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=xxxxxxxxxxxxxxxxxxxxxxxx
NEON_DATABASE_URL=postgresql://user:password@ep-xxxxxxxx-pooler.us-east-2.aws.neon.tech/neondb?sslmode=require
ENVIRONMENT=development
LOG_LEVEL=INFO
```

**Validation**:
```bash
python -c "from app.config import settings; print(settings.GROQ_API_KEY[:10])"
# Should print: gsk_xxxxxx
```

### 5. Setup Database
```bash
python scripts/setup_db.py
```

**Output**:
```
✓ Connected to Neon Postgres
✓ Created table: sessions
✓ Created table: messages
✓ Created table: queries
✓ Created table: feedback
Database setup complete!
```

### 6. Ingest Textbook
```bash
python scripts/ingest_textbook.py --chapters ../docusaurus/docs/ch*/*.md
```

**Output**:
```
Reading 13 chapter files...
Parsing and chunking...
Generating Cohere embeddings...
Upserting to Qdrant...
[████████████████████████████████████████] 60/60 chunks
✓ Ingestion complete: 60 chunks, 30,420 tokens
✓ Qdrant collection size: 320KB
✓ Logged to Neon: ingestion_metadata table
```

**Verify in Qdrant Console**:
- Go to https://cloud.qdrant.io
- Select collection: `textbook_chunks`
- Scroll through 60 points
- Check payload for `chapter_id`, `content`

### 7. Start Backend Server
```bash
uvicorn app.main:app --reload --port 8000
```

**Output**:
```
INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345]
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

**Test**:
- Open browser: http://localhost:8000/docs (Swagger UI)
- Try POST /api/session → Get session_id
- Try POST /api/search with query "ROS 2" → See chunks returned

---

## Frontend Setup (Docusaurus)

### 1. Navigate to Docusaurus
```bash
cd ../docusaurus
```

### 2. Install Dependencies
```bash
npm install
```

**New packages added**:
- `react-chat-widget` (chatbot UI)
- TypeScript types for SSE

### 3. Configure API Endpoint
Create `.env.local` file:

```bash
# .env.local
REACT_APP_API_URL=http://localhost:8000
```

### 4. Start Frontend Server
```bash
npm run start
```

**Output**:
```
[SUCCESS] Docusaurus website is running at: http://localhost:3000/
```

---

## End-to-End Test

### Terminal Setup
```bash
# Terminal 1: Backend
cd backend && uvicorn app.main:app --reload

# Terminal 2: Frontend
cd docusaurus && npm run start
```

### Manual Test Flow

1. **Open Browser**: http://localhost:3000/docs/ch01-physical-ai-intro/ch01
2. **Locate Chatbot Widget**: Bottom-right floating button
3. **Click to Open**: Chatbot window expands
4. **Type Query**: "What is Physical AI?"
5. **Observe**:
   - Streaming response appears word-by-word
   - Citation link shows: [Chapter 1: Physical AI Intro]
   - Click citation → navigates to /docs/ch01-physical-ai-intro/ch01
6. **Test Context-Aware Mode**:
   - Highlight paragraph about "Vision-Language-Action models"
   - Click "Ask about this" tooltip
   - Type: "Explain this in simpler terms"
   - Response should reference highlighted text

### Automated Test
```bash
# Backend tests
cd backend
pytest tests/ -v

# Frontend tests
cd docusaurus
npm run test
```

---

## Optional: LiteLLM Proxy (Dev Only)

For easier local testing with multiple models:

```bash
# Install proxy
pip install 'litellm[proxy]'

# Create config
cat > backend/config.yaml << 'EOF'
model_list:
  - model_name: groq-llama
    litellm_params:
      model: groq/llama-3.1-70b-versatile
      api_key: ${GROQ_API_KEY}
  - model_name: cohere-embed
    litellm_params:
      model: cohere/embed-english-v3.0
      api_key: ${COHERE_API_KEY}
EOF

# Start proxy
litellm --config backend/config.yaml --port 4000

# Update .env
LITELLM_BASE_URL=http://localhost:4000
```

**Test Proxy**:
```bash
curl -X POST http://localhost:4000/chat/completions \
  -H "Content-Type: application/json" \
  -d '{"model":"groq-llama","messages":[{"role":"user","content":"Hello"}]}'
```

---

## Troubleshooting

### Error: `ModuleNotFoundError: No module named 'litellm'`
**Solution**: Activate venv: `source venv/bin/activate`

### Error: `Connection refused` to Qdrant
**Solution**: Check Qdrant URL and API key in `.env`. Test with:
```bash
curl -X GET "$QDRANT_URL/collections" \
  -H "api-key: $QDRANT_API_KEY"
```

### Error: `Groq API rate limit exceeded`
**Solution**: Wait 60 seconds. Free tier: 30 req/min.

### Error: `ChatbotWidget not rendering`
**Solution**:
1. Check browser console for errors
2. Verify REACT_APP_API_URL in `.env.local`
3. Restart frontend: `npm run start`

### Error: `No chunks returned from search`
**Solution**: Re-run ingestion:
```bash
python scripts/ingest_textbook.py --chapters ../docusaurus/docs/ch*/*.md --force
```

---

## Next Steps

1. **Run Tests**: `pytest backend/tests/ && npm run test`
2. **Check Performance**: Open http://localhost:8000/metrics (if implemented)
3. **Load Test**: `k6 run backend/tests/load-test.js`
4. **Deploy**: Follow deployment guide for Railway (backend) + GitHub Pages (frontend)

---

**Quickstart Status**: ✅ COMPLETE
**Estimated Setup Time**: 20 minutes
