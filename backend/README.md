# RAG Chatbot Backend

FastAPI backend for Physical AI textbook chatbot with Groq + LiteLLM + Qdrant.

## Quick Start

### 1. Install Dependencies

```bash
cd backend
python3.11 -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 2. Configure Environment

Copy `.env.example` to `.env` and fill in your API keys:

```bash
cp .env.example .env
# Edit .env with your actual keys
```

**Get API Keys**:
- **Groq**: https://console.groq.com → API Keys
- **Cohere**: https://dashboard.cohere.com → API Keys
- **Qdrant**: https://cloud.qdrant.io → Create cluster → API Keys
- **Neon**: https://console.neon.tech → Create project → Connection string

### 3. Setup Database

```bash
# Connect to Neon and run schema
psql $NEON_DATABASE_URL -f schema.sql
```

### 4. Ingest Textbook

```bash
# Ingest all 13 chapters into Qdrant
python scripts/ingest.py --chapters ../docusaurus/docs/ch*/*.md
```

Expected output:
```
Found 13 chapter files
Processing chapters: 100%|██████████| 13/13
Generating embeddings for 60 chunks...
Upserting 60 chunks to Qdrant...
✓ Ingestion complete: 60 chunks
✓ Total tokens: ~30420
✓ Qdrant collection size: 60 points
```

### 5. Run Server

```bash
uvicorn app.main:app --reload --port 8000
```

Server running at: http://localhost:8000
API docs: http://localhost:8000/docs

## API Endpoints

### POST /query

Query the textbook with RAG retrieval and streaming response.

**Request**:
```json
{
  "message": "What is Physical AI?",
  "selected_text": "optional context from highlighted text",
  "session_id": "optional-session-uuid"
}
```

**Response**: Server-Sent Events (SSE) stream

```
data: {"type": "chunk", "content": "Physical"}
data: {"type": "chunk", "content": " AI"}
data: {"type": "citations", "data": [{"chapter_id": "ch01", "chapter_title": "Introduction to Physical AI", "similarity": 0.89}]}
data: {"type": "done"}
```

### GET /health

Health check endpoint.

**Response**:
```json
{
  "status": "healthy",
  "model": "groq/llama-3.1-70b-instant"
}
```

## Deployment

### Vercel

```bash
# Install Vercel CLI
npm install -g vercel

# Add environment variables
vercel env add GROQ_API_KEY
vercel env add COHERE_API_KEY
vercel env add QDRANT_URL
vercel env add QDRANT_API_KEY
vercel env add NEON_DATABASE_URL

# Deploy
vercel --prod
```

Your API will be available at: `https://your-project.vercel.app`

### Railway

```bash
# Install Railway CLI
npm install -g @railway/cli

# Login
railway login

# Initialize project
railway init

# Add environment variables
railway variables set GROQ_API_KEY=gsk_...
railway variables set COHERE_API_KEY=...
railway variables set QDRANT_URL=https://...
railway variables set QDRANT_API_KEY=...
railway variables set NEON_DATABASE_URL=postgresql://...

# Deploy
railway up
```

## Testing

```bash
# Test health endpoint
curl http://localhost:8000/health

# Test query endpoint
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS 2?"}'
```

## Architecture

```
app/
├── main.py           # FastAPI app, /query endpoint, SSE streaming
├── config.py         # Environment configuration
└── __init__.py

scripts/
└── ingest.py         # MDX parsing, chunking, Cohere embedding, Qdrant upsert

schema.sql            # Neon Postgres tables
requirements.txt      # Python dependencies
vercel.json           # Vercel deployment config
```

## Performance

- **Latency**: <200ms time-to-first-token (Groq LLM)
- **Retrieval**: <50ms (Qdrant semantic search)
- **Embedding**: <100ms (Cohere API)
- **Total**: <300ms end-to-end query

## Troubleshooting

**Error: "GROQ_API_KEY not found"**
- Ensure `.env` file exists and contains all required keys
- Activate virtual environment: `source venv/bin/activate`

**Error: "Collection 'physical_ai_book' not found"**
- Run ingestion script: `python scripts/ingest.py --chapters ../docusaurus/docs/ch*/*.md`

**Error: "litellm.exceptions.RateLimitError"**
- Groq free tier: 30 requests/min, 6000 tokens/min
- Wait 60 seconds before retrying

**Error: "Failed to connect to Qdrant"**
- Verify `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
- Test connection: `curl -H "api-key: $QDRANT_API_KEY" $QDRANT_URL/collections`

## License

MIT
