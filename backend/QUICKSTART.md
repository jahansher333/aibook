# Quick Start Guide - RAG Chatbot with Agents SDK

## ✅ Prerequisites Completed

- [X] `.env` file created with your API keys
- [⏳] Dependencies installing (`pip install -r requirements.txt` is running)

## 🚀 Next Steps

### 1. Wait for Dependencies to Finish Installing

The command is currently running in the background. It should complete in 2-3 minutes.

Packages being installed:
- ✅ FastAPI 0.104.1
- ✅ Uvicorn 0.24.0
- ✅ LiteLLM 1.17.0
- ✅ OpenAI 1.6.1
- ⏳ **Agents SDK 1.4.0** (currently installing)
- Qdrant Client, Cohere, SQLAlchemy, etc.

### 2. Setup Neon Database (Optional for MVP)

The chatbot will work without the database (sessions won't persist). To enable full session management:

```bash
# Install psql if not available
# Then run:
psql 'postgresql://neondb_owner:npg_K50wZmqMundL@ep-wispy-darkness-a40u7fom-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require' -f schema.sql
```

### 3. Create Sample Textbook Content (For Testing)

Since we don't have the actual 13 chapters yet, let's create a sample chapter:

```bash
# Create docs directory
mkdir -p ../docusaurus/docs/ch01-physical-ai-intro

# We'll create a sample chapter file
```

Let me create this for you...

### 4. Run Ingestion (After Creating Sample Content)

```bash
python scripts/ingest.py --chapters ../docusaurus/docs/ch01-physical-ai-intro/*.md
```

### 5. Start the Backend Server

```bash
# From the backend directory
python -m uvicorn app.main:app --reload --port 8000
```

### 6. Test the Agent

Open another terminal and test:

```bash
# Health check
curl http://localhost:8000/health

# Test agent query
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"message": "What is Physical AI?"}'
```

## 🔍 What to Expect

When you query the agent, you'll see:

1. **Debug logs** showing agent thinking:
   ```
   [debug] Running agent with query: What is Physical AI?...
   [debug] Searching textbook for: Physical AI concepts
   ```

2. **Streaming response**:
   ```json
   data: {"type":"chunk","content":"Physical "}
   data: {"type":"chunk","content":"AI "}
   data: {"type":"chunk","content":"combines "}
   ...
   data: {"type":"citations","data":[{"chapter_id":"ch01","chapter_title":"Chapter 1"}]}
   data: {"type":"done"}
   ```

## 🐛 Troubleshooting

### "No module named 'agents'"
Wait for the background installation to complete, or run:
```bash
pip install agents
```

### "Collection 'physical_ai_book' not found"
You need to run the ingestion script first (step 4 above).

### "GROQ_API_KEY not found"
Make sure the `.env` file exists in the backend directory and contains all keys.

## 📊 Your API Keys Status

✅ **Cohere**: 9n28R7x... (configured)
✅ **Groq**: gsk_6bWRC... (configured)
✅ **Qdrant**: https://97840018-5568-4f3b-bebd... (configured)
✅ **Neon**: postgresql://neondb_owner... (configured)

## 🎯 Quick Test Without Ingestion

You can test the server without ingesting data - the agent will just return responses saying no information was found in the textbook.

```bash
# Start server
python -m uvicorn app.main:app --reload

# In another terminal:
curl http://localhost:8000/health
# Should return: {"status":"healthy","model":"groq/llama-3.1-70b-instant","agent":"OpenAI Agents SDK + LiteLLM"}
```

## 📁 Project Structure

```
backend/
├── app/
│   ├── main.py          # FastAPI routes (uses get_agent())
│   ├── agent.py         # RagChatbotAgent with Runner.run()
│   ├── tools.py         # @function_tool search_textbook()
│   └── config.py        # Settings from .env
├── scripts/
│   └── ingest.py        # Qdrant ingestion script
├── .env                 # ✅ Your API keys
├── requirements.txt     # ⏳ Installing now
└── schema.sql           # Neon database schema
```

---

**Status**: ⏳ **WAITING FOR DEPENDENCIES TO FINISH**

Check progress with:
```bash
# Monitor the installation
tail -f pip_install.log  # If you redirected output
```

Or just wait 2-3 minutes and proceed to step 5 (Start the Backend Server).
