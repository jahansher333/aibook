# Phase 0 Research: Urdu Translation Button System

**Date**: 2025-12-10 | **Feature**: 007-urdu-translation-button | **Status**: COMPLETE

## Research Findings

### 1. Groq API Integration (LiteLLM)

**Decision**: Use LiteLLM as the abstraction layer to call Groq API with llama-3.1-70b-instant model.

**Rationale**:
- LiteLLM provides unified interface across multiple LLM providers (Groq, OpenAI, Anthropic, etc.)
- Reduces vendor lock-in; easy to swap providers if needed
- Built-in retry logic, rate limit handling, and cost tracking
- Works seamlessly in both browser (via backend) and Node.js environments
- Groq's llama-3.1-70b-instant is fast (~100ms response time) and cheap (minimal token costs)

**Alternatives Considered**:
- Direct Groq SDK: Lower-level control, but duplicates retry/error handling logic
- OpenAI API: Rejected per spec requirement (FR-014); Groq is 3-5x faster for same task
- Local LLM (Ollama): No internet latency benefits for translation task

**Implementation**:
- Frontend: Call `/api/translate` backend endpoint (not direct API key in browser)
- Backend: Use `litellm.completion()` with model="groq/llama-3.1-70b-instant"
- Environment: Store GROQ_API_KEY in `.env` (never in frontend code)

**Production Rate Limits**: Groq free tier ~30 requests/minute; paid tier 100+ req/min. For 13 chapters × ~2 translations/user/month = ~26 API calls/user/month. Not a constraint for typical usage.

---

### 2. Caching Strategy (localStorage + Qdrant)

**Decision**: Two-tier caching: Browser localStorage (primary) + optional Qdrant backend cache (future).

**Rationale**:
- **localStorage** (primary): Instant offline access, no server dependency, <500ms retrieval
- **Qdrant** (optional): Distribute cache across users, reduce API calls, support multi-device sync
- Browser storage key format: `urdu_translation_ch{chapterID}` + `{timestamp}` + `{hash of original content}`
- TTL: 30 days per spec (FR-010)

**Cache Entry Schema**:
```json
{
  "chapterId": "ch01",
  "originalHash": "sha256(originalContent)",
  "urduContent": "اردو میں ترجمہ شدہ متن",
  "timestamp": 1702209600000,
  "expiryTime": 1704801600000,
  "contentVersion": 1
}
```

**Fallback Path**: If localStorage quota exceeded, fall back to session memory (non-persistent).

---

### 3. Handling Code Blocks and Special Content (FR-006, FR-007)

**Decision**: Parse HTML/Markdown to identify code blocks, inline code, and preserve them; translate only narrative text.

**Rationale**:
- ROS 2, Isaac Sim, URDF, Gazebo should remain in English (standard technical terms)
- Code examples must never be translated (syntax breaking risk)
- Headings, links, images preserved; only text content translates

**Prompt Engineering**:
```
Translate the following robotics content to natural, easy-to-read Urdu.
CRITICAL RULES:
- Keep technical terms in English: ROS 2, Isaac Sim, URDF, Gazebo, Jetson, RTX, Unreal, Unity, etc.
- Preserve all code blocks (they are marked with [CODE_BLOCK] markers)—do NOT translate code
- Keep all links and markdown formatting intact
- Translate ONLY narrative text to natural, fluent Urdu
- Maintain educational tone (formal, accurate, clear)
```

---

### 4. Race Condition Prevention (Edge Case)

**Decision**: Deduplicate simultaneous translation requests using Promise caching in React Context.

**Rationale**:
- User clicks button twice rapidly → only one API call made
- In-flight promise stored in state; subsequent requests await the same promise
- Prevents duplicate API calls; saves costs; faster UX

---

### 5. Offline Fallback & Error Handling (FR-011)

**Decision**: If API unreachable → show English + notification; cached content works offline.

**Rationale**:
- User experience never breaks; graceful degradation
- Notification informs user why Urdu isn't available
- If cached Urdu exists → display it even offline

---

### 6. Translation State Persistence (FR-013)

**Decision**: Session-level state; translation choice persists during user's current session but resets on page refresh.

**Rationale**:
- React Context stores `isUrdu` flag per chapter
- localStorage stores the translated content (retrieval is fast)
- User navigates chapter-to-chapter; language preference follows

---

### 7. Personalization Integration (FR-012)

**Decision**: Personalize button (from 006-personalize-button) modifies chapter DOM; translation button translates the personalized DOM (not original).

**Rationale**:
- Personalize button rewrites DOM content based on user hardware profile
- Translation hook reads the current DOM state (after personalization)
- User flow: (1) Personalize → (2) Translate → gets personalized content in Urdu

---

### 8. Testing Strategy

**Unit Tests**:
- `useUrduTranslation` hook: mock API, test cache hit/miss, localStorage operations
- Error scenarios: API failure, quota exceeded, network timeout
- Edge case: rapid successive clicks (deduplication)

**Integration Tests**:
- Full flow: Click button → API call → cache → toggle → retrieve from cache
- Offline behavior: disable network → verify fallback
- Personalization interaction: personalize content, then translate

---

## Clarifications Resolved

| Item | Resolution |
|------|-----------|
| Groq API rate limits | Free: ~30 req/min; Paid: 100+ req/min; Not a constraint for typical textbook usage |
| Code block detection | Parse HTML DOM; mark `<code>`, `<pre>` as preserve in prompt; reconstruct after translation |
| localStorage quota | Fall back to session memory (non-persistent); notify user; graceful degradation |
| Technical terms in Urdu | ROS 2, Isaac Sim, URDF, Gazebo, Jetson, RTX stay English; "robot" → "روبوٹ" if standard exists |
| Personalization integration | Translate reads current DOM (after personalization); translates personalized content, not original |
| State persistence | Session-level (React Context); survives chapter navigation; resets on page refresh |
| Race conditions | Deduplicate via Promise caching; only one API call per chapter |

---

## Next Steps (Phase 1)

- [ ] Create `data-model.md` with cache schema, state shapes
- [ ] Create API contract: `contracts/translation-api.openapi.yaml`
- [ ] Create `quickstart.md` with setup instructions
- [ ] Run `/sp.tasks` to generate task breakdown
