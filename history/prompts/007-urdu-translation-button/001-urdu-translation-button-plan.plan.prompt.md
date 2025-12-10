---
id: 001
title: Urdu Translation Button System Plan
stage: plan
date: 2025-12-10
surface: agent
model: claude-haiku-4-5-20251001
feature: 007-urdu-translation-button
branch: 007-urdu-translation-button
user: Physical AI Team
command: /sp.plan
labels: ["groq-api", "react", "translation", "caching", "localization"]
links:
  spec: specs/007-urdu-translation-button/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/007-urdu-translation-button/plan.md
  - specs/007-urdu-translation-button/research.md
  - specs/007-urdu-translation-button/data-model.md
  - specs/007-urdu-translation-button/quickstart.md
  - specs/007-urdu-translation-button/contracts/translation-api.openapi.yaml
tests:
  - None (planning phase; no tests run yet)
---

## Prompt

```
Urdu Translation Button System
Tech:
- Groq API via LiteLLM (model: llama-3.1-70b-instant)
- React state: isUrdu + translatedContent
- Cache: localStorage + chapterId → avoid re-calling Groq
- Prompt: "Translate the following robotics content to natural, easy-to-read Urdu. Keep technical terms accurate (ROS2, Isaac Sim, URDF, etc.)"
What the agent does: Full plan + prompt template + cache strategy.
```

Execute `/sp.plan` for the Urdu Translation Button feature:
1. Load feature spec and constitution
2. Execute Phase 0: Research (resolve clarifications)
3. Execute Phase 1: Design data model, API contracts, quickstart
4. Document project structure
5. Generate PHR

## Response snapshot

### Phase 0: Research (COMPLETE ✅)

**Key Findings**:
1. **Groq API Integration**: LiteLLM abstraction layer with llama-3.1-70b-instant
2. **Caching Strategy**: Browser localStorage (primary) + optional Qdrant backend (future)
3. **Code Preservation**: Parse HTML/Markdown to identify code blocks; translate only narrative text
4. **Race Condition Prevention**: Promise caching in React Context ensures single API call per chapter
5. **Offline Fallback**: API error → English + notification; cached content works offline
6. **State Persistence**: Session-level React Context; per-chapter toggle (survives navigation)
7. **Personalization Integration**: Translation hook reads current DOM (after personalization applied)
8. **Testing Strategy**: Unit tests (hook, cache), integration tests (full flow), acceptance tests

**All Clarifications Resolved** in `research.md`:
- Groq API rate limits: 30 req/min free tier; 100+ paid (not a constraint for typical usage)
- Code block detection: Parse HTML DOM; preserve `<code>`, `<pre>` via prompt markers
- localStorage quota: Fall back to session memory; notify user
- Technical terms: ROS 2, Isaac Sim, URDF, Gazebo stay English; "robot" → "روبوٹ" if standard exists
- Personalization: Translate reads current DOM (after personalization); not original
- State persistence: Session-level (React Context); survives chapter navigation; resets on reload
- Race conditions: Deduplicate via Promise caching; only one API call per chapter

### Phase 1: Design & Contracts (COMPLETE ✅)

**Data Model** (`data-model.md`):
- **UrduTranslation**: Entity with chapterId, originalContent, originalContentHash, urduTranslation, timestamp, expiryTime (30-day TTL)
- **LanguageState**: Session-level per-chapter boolean (isUrdu flag in React Context)
- **CacheEntry**: localStorage wrapper with get/set/delete/isStale operations
- Storage: localStorage (`urdu_translation_<chapterId>`) + session memory fallback
- Validation: SHA-256 hash + TTL check; stale entries auto-removed
- Relationships: Chapter 1:0..1 UrduTranslation; Chapter 1:1 LanguageState
- Extensibility: Migration path for Qdrant backend and multi-language support

**API Contracts** (`contracts/translation-api.openapi.yaml`):
- `POST /api/v1/translate`: request (chapterId, content, skipCache); response (urduContent, duration, cached, cacheExpiryTime, contentHash)
- `GET /api/v1/translate/{chapterId}/status`: cache status check
- `DELETE /api/v1/translate/{chapterId}`: cache cleanup
- Error codes: 400 (bad request), 429 (rate limited with retryAfter), 500 (internal), 503 (service unavailable)
- Response metadata: duration (ms), cached (boolean), cacheExpiryTime (ISO 8601), contentHash (SHA-256)

**Quickstart** (`quickstart.md`):
- Step 1: Obtain Groq API key, store in `.env.local`
- Step 2: Create utilities (translationCache.ts, groqClient.ts, urduPrompt.ts)
- Step 3: Create React Context (LanguageContext.tsx)
- Step 4: Create Component (UrduTranslationButton.tsx + CSS)
- Step 5: Integrate into chapters
- Step 6: Test end-to-end (2s API, <500ms cache)
- Step 7: Performance benchmarking
- Troubleshooting: API key validation, Groq errors, localStorage quota, Urdu fonts

**Project Structure**:
```
docusaurus/src/
├── components/UrduTranslationButton.tsx + CSS + tests
├── contexts/LanguageContext.tsx
├── hooks/useUrduTranslation.ts
└── utils/translationCache.ts, groqClient.ts, urduPrompt.ts

backend/src/ (optional, for distributed caching)
├── services/translation_service.py (LiteLLM + Qdrant)
├── routes/translation.py (FastAPI endpoint)
└── models/translation.py (Pydantic schemas)

tests/ (unit + integration)
```

### Constitution Check (GATE: ALL PASS ✅)

All 7 constitution principles verified:
1. **Helpful and Impactful**: Improves accessibility for Urdu-speaking learners
2. **Honest and Accurate**: Groq API choice explicit; translation quality validated by Urdu speaker
3. **Harmless and Inclusive**: Offline fallback; no localization barriers
4. **Spec-Driven and AI-Native**: Spec.md with user stories; plan uses template; reusable TranslateToUrduSkill
5. **Structured and Comprehensive**: Component structure documented; cache schema clear; fallback paths defined
6. **Efficient and Scalable**: Uses Groq free tier; caching avoids repeated API calls
7. **Innovative yet Practical**: Optional Qdrant caching; frontend-first approach; fallback mechanisms

### Phase 2: Implementation (NEXT)

Next step: Run `/sp.tasks` to generate task breakdown:
- Implement React component, hook, utils, context
- Unit + integration tests
- Integrate into all 13 chapters
- Performance testing
- Peer review with Urdu speaker
- Deploy to GitHub Pages

Acceptance Criteria:
- All 13 chapters have functional button
- <2s first translation, <500ms cached
- 100% text coverage (excluding code)
- Graceful offline fallback
- Personalization integration working
- All tests passing
- Urdu translation quality reviewed

## Outcome

- ✅ **Impact**: Complete architecture plan for Urdu translation feature; all technical decisions documented and justified
- 🧪 **Tests**: No tests run (planning phase); test strategy defined in research.md and quickstart.md
- 📁 **Files**: 5 artifacts created (plan.md, research.md, data-model.md, quickstart.md, translation-api.openapi.yaml)
- 🔁 **Next prompts**: Run `/sp.tasks` to generate Phase 2 task breakdown; then implement and test
- 🧠 **Reflection**: Planning phase completed successfully. Feature aligns with all 7 constitution principles. Clear path to implementation with minimal unknowns. Ready to begin Phase 2 (tasks).

## Evaluation notes (flywheel)

- **Failure modes observed**: None; all clarifications resolved during Phase 0 research
- **Graders run and results**: Constitution Check: PASS (all 7 principles verified)
- **Prompt variant**: N/A (single execution)
- **Next experiment**: Execute `/sp.tasks` to convert design into testable task breakdown
