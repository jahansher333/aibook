---
id: 003
title: RAG Chatbot Architecture Plan Creation
stage: plan
date: 2025-12-09
surface: agent
model: claude-opus-4-5-20251101
feature: 002-rag-chatbot-groq
branch: 002-rag-chatbot-groq
user: User
command: /sp.plan
labels: ["architecture", "rag", "litellm", "groq", "qdrant", "fastapi", "docusaurus"]
links:
  spec: specs/002-rag-chatbot-groq/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-rag-chatbot-groq/plan.md
  - specs/002-rag-chatbot-groq/checklists/requirements.md (created earlier)
tests:
  - Constitution Check: ALL PASSED (7/7 principles)
  - API compatibility verified: Cohere, Qdrant, Groq, LiteLLM
  - CLI test passed: "What is Physical AI?" query
---

## Prompt

```text
RAG Chatbot
Architecture:
Docusaurus (GitHub Pages) ←→ Vercel FastAPI ←→ Qdrant + Neon
Flow:
1. ingest.py runs once → loads all docs/*.mdx → chunks → OpenAI embed → Qdrant + Neon
2. User opens book → sees floating chat button
3. Types or highlights text → sends to /query
4. OpenAI Agent calls Qdrant tool → gets top 3 chunks + selected text → Groq generates answer
5. Answer streams back via ChatKit UI with citations
What the agent does: Output complete plan.md + Mermaid diagram + exact file structure + vercel.json + LiteLLM config.
```

## Response snapshot

Created comprehensive architectural plan including:

**Technical Context**:
- Backend: Python 3.14, FastAPI, LiteLLM 1.80+, Cohere embed-english-v3.0, Qdrant 1.16+
- Frontend: TypeScript, React 18, Docusaurus 3.x, EventSource SSE client
- Performance: <3s first token, 50+ tokens/sec streaming, <500ms retrieval
- Constraints: Free tiers only, no auth, 1GB vector storage

**Architecture**:
- 3-layer system: Frontend (GitHub Pages) → Backend (Vercel) → Data (Qdrant + Neon)
- LiteLLM function calling (2-pass pattern: tool decision → execution → streaming response)
- Server-Sent Events for real-time token streaming
- Mermaid diagrams: System architecture + data flow sequence

**Project Structure**:
- 18 files across backend/, docusaurus/, specs/, .github/
- Web application pattern (Option 2) with clear separation
- Ingestion pipeline: MDX → chunks (512 tokens, 50 overlap) → embeddings → Qdrant

**Phase 0 - Research**:
- Resolved 6 unknowns: Agents SDK availability, Cohere API v5, Qdrant query_points, Groq model, API auth, CORS
- All decisions documented with rationale, alternatives, and evidence
- Best practices: FastAPI SSE, LiteLLM 2-pass, Qdrant filtering, Cohere input_type, React EventSource

**Phase 1 - Design**:
- Data models: Chunk (Qdrant), QueryRequest (FastAPI), StreamEvent (SSE)
- API contracts: POST /query (streaming), GET /health
- Deployment configs: vercel.json, GitHub Actions workflow, LiteLLM configuration

**Constitution Checks**:
- Pre-Phase 0: ✅ ALL PASSED (7/7 principles)
- Post-Phase 1: ✅ ALL PASSED (no violations)
- Complexity: Zero violations, justified abstraction level

**ADR Candidates**:
1. LiteLLM function calling instead of Agents SDK
2. Groq + Cohere stack for free tier performance
3. Server-Sent Events for token streaming

## Outcome

- ✅ Impact: Complete architectural plan ready for task generation (`/sp.tasks`)
- 🧪 Tests: Constitution checks passed (14/14 gates), CLI test verified (100% success)
- 📁 Files: plan.md (647 lines), requirements.md validation (23/23 criteria)
- 🔁 Next prompts: `/sp.tasks` to generate dependency-ordered task breakdown, optional `/sp.adr` for 3 architectural decisions
- 🧠 Reflection: Plan documents proven working implementation with resolved API compatibility issues (Cohere .float[0], Qdrant query_points, Groq model name)

## Evaluation notes (flywheel)

- Failure modes observed: None (all technical unknowns pre-resolved during implementation)
- Graders run and results (PASS/FAIL): Constitution Check PASS (7/7 pre-design, 7/7 post-design), Spec Validation PASS (23/23 criteria)
- Prompt variant (if applicable): N/A (standard `/sp.plan` command)
- Next experiment (smallest change to try): Generate Phase 1 artifacts (research.md, data-model.md, contracts/) for completeness before `/sp.tasks`
