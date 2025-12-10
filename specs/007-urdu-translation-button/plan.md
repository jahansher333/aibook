# Implementation Plan: [FEATURE]

**Branch**: `[###-feature-name]` | **Date**: [DATE] | **Spec**: [link]
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a per-chapter Urdu translation button for the Physical AI Textbook using Groq API (llama-3.1-70b-instant via LiteLLM). The system translates entire chapter content to natural Urdu while preserving technical terms and formatting, caches translations in browser localStorage keyed by chapter ID, and gracefully falls back to English if the API is unavailable. The button toggles between "اردو میں دیکھیں" (View in Urdu) and "Show in English" states.

## Technical Context

**Language/Version**: TypeScript/React 18+ (frontend component); Python 3.11+ (optional backend cache layer)
**Primary Dependencies**: LiteLLM (API abstraction), Groq API (llama-3.1-70b-instant model), React Context (state management)
**Storage**: Browser localStorage (primary cache), optional: Qdrant (backend cache per constitution principle VI)
**Testing**: React Testing Library (component), integration tests for API fallback
**Target Platform**: Web browsers (Chrome, Safari, Firefox, Edge) with localStorage support
**Project Type**: Web/React component (frontend-first; optional FastAPI backend for distributed caching)
**Performance Goals**: First translation <2 seconds (API), cached translation <500ms (localStorage), 100% text coverage (excluding code)
**Constraints**: Groq API rate limits (NEEDS CLARIFICATION: production rate limits), localStorage quota handling (fallback to session memory)
**Scale/Scope**: 13 chapters, ~2,000-5,000 words per chapter, per-chapter caching, deduplication of simultaneous requests

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Gate | Status | Notes |
|-----------|------|--------|-------|
| **I. Helpful and Impactful** | Urdu translation button accessible to all students, improves accessibility for Urdu-speaking learners | ✅ PASS | Feature directly supports Principle I goal of linguistic inclusivity |
| **II. Honest and Accurate** | Groq API choice is verified; translation quality validated by Urdu-speaking reviewer; no hallucinations in tech terms | ✅ PASS | User provided explicit API choice (Groq); requirements specify technical term accuracy |
| **III. Harmless and Inclusive** | Translation supports offline fallback; localStorage caching avoids excessive API calls; no localization barriers | ✅ PASS | Feature explicitly includes offline fallback (FR-011); aligns with Principle III |
| **IV. Spec-Driven and AI-Native** | Feature defined via spec.md with user stories; plan uses template; can leverage TranslateToUrduSkill reuse | ✅ PASS | Spec-driven structure present; reusable skill pattern follows constitution |
| **V. Structured and Comprehensive** | React component structure documented; localStorage schema clear; fallback paths defined | ✅ PASS | Plan documents component structure and error paths |
| **VI. Efficient and Scalable** | Uses Groq free tier (fast, cheap); localStorage avoids repeated API calls; optional Qdrant caching (backend) | ✅ PASS | Efficient use of APIs; caching strategy aligns with free tier optimization |
| **VII. Innovative yet Practical** | Optional backend cache (Qdrant) supports future scaling; frontend-first approach ensures immediate value | ✅ PASS | Fallback mechanisms present; cross-browser compatible design |

**GATE RESULT**: ✅ ALL GATES PASS - Feature aligns with all 7 constitution principles

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Frontend: React component embedded in Docusaurus
docusaurus/src/components/
├── UrduTranslationButton.tsx        # Main button component
├── TranslationContext.tsx           # React Context for language state (isUrdu, translatedContent)
└── hooks/
    └── useUrduTranslation.ts        # Custom hook for localStorage, API calls, caching

# Backend (optional, for distributed caching)
backend/src/
├── services/
│   └── translation_service.py       # LiteLLM wrapper + Qdrant caching
├── routes/
│   └── translation.py               # FastAPI endpoint: POST /translate
└── models/
    └── translation.py               # Pydantic models for request/response

# Tests
docusaurus/src/components/__tests__/
├── UrduTranslationButton.test.tsx   # Component rendering, button toggle
├── useUrduTranslation.test.ts       # Hook: localStorage, API, fallback
└── integration/
    └── translation-flow.test.tsx    # Full flow: click → API call → cache

backend/tests/
├── unit/
│   └── translation_service_test.py  # Service: LiteLLM call, cache hit/miss
└── integration/
    └── translation_api_test.py      # API endpoint: request validation, error handling
```

**Structure Decision**: Frontend-first React component (UrduTranslationButton.tsx) embedded in Docusaurus chapters; optional FastAPI backend for distributed caching and batch translation. Primary caching via browser localStorage; Qdrant backend cache for future multi-user optimization.

## Complexity Tracking

> **No violations detected** — Constitution gates all pass; complexity is within scope.

---

## Phase 0: Research (COMPLETE ✅)

**Output**: `research.md` with all clarifications resolved

**Key Findings**:
1. **Groq API Integration**: Use LiteLLM abstraction with llama-3.1-70b-instant
2. **Caching Strategy**: Two-tier (browser localStorage + optional Qdrant backend)
3. **Code Preservation**: Parse HTML/Markdown to identify code blocks; translate only narrative text
4. **Race Condition Prevention**: Promise caching in React Context (single API call per chapter)
5. **Offline Fallback**: API error → English + notification; cached content works offline
6. **State Persistence**: Session-level React Context; per-chapter toggle
7. **Personalization Integration**: Translate reads current DOM (after personalization)
8. **Testing Strategy**: Unit, integration, and acceptance tests defined

**Gates Passed**: ✅ ALL 7 Constitution principles verified

---

## Phase 1: Design & Contracts (COMPLETE ✅)

### 1.1 Data Model (`data-model.md`)

**Entities**:
- **UrduTranslation**: Cached translation with SHA-256 hash, TTL (30 days), metadata
- **LanguageState**: Session-level per-chapter boolean (isUrdu flag)
- **CacheEntry**: localStorage wrapper with get/set/delete operations

**Storage**:
- Browser localStorage (primary): `urdu_translation_<chapterId>` key format
- Session memory (fallback): if localStorage quota exceeded
- Future: Qdrant backend cache for distributed caching

**Relationships**:
- Chapter 1:0..1 UrduTranslation (every chapter may have 0 or 1 cached translation)
- Chapter 1:1 LanguageState (every chapter has exactly one state during session)

**Validation**: Content hash + TTL check; stale entries auto-removed; QuotaExceededError handling

**Extensibility**: Migration path for server-side caching (Qdrant) and multi-language support

### 1.2 API Contracts (`contracts/translation-api.openapi.yaml`)

**Endpoints**:
- `POST /api/v1/translate`: Translate chapter (request: chapterId + content; response: urduContent + duration + cacheExpiry)
- `GET /api/v1/translate/{chapterId}/status`: Check cache status
- `DELETE /api/v1/translate/{chapterId}`: Clear cache

**Error Handling**:
- 400: Bad request (invalid chapterId, content too long)
- 429: Rate limited (Groq API limit exceeded; includes retryAfter)
- 500: Internal error
- 503: Service unavailable (Groq API down)

**Response Metadata**: duration (ms), cached (boolean), cacheExpiryTime (ISO 8601), contentHash (SHA-256)

### 1.3 Quickstart (`quickstart.md`)

**Setup Steps**:
1. Obtain Groq API key from console.groq.com
2. Store in `.env.local`: `REACT_APP_GROQ_API_KEY=gsk_xxx`
3. Create utilities: `translationCache.ts`, `groqClient.ts`, `urduPrompt.ts`
4. Create React Context: `LanguageContext.tsx`
5. Create Component: `UrduTranslationButton.tsx` + CSS
6. Integrate into chapters
7. Test end-to-end + offline fallback
8. Performance benchmark (1200-1700ms API, 150-350ms cache)

**Troubleshooting**: API key validation, Groq errors, localStorage quota, Urdu font rendering

### 1.4 Project Structure (Updated)

```
docusaurus/src/
├── components/
│   ├── UrduTranslationButton.tsx        # Main button component
│   ├── UrduTranslationButton.module.css # Styling
│   └── __tests__/
│       └── UrduTranslationButton.test.tsx
├── contexts/
│   └── LanguageContext.tsx              # Language state provider
├── hooks/
│   └── useUrduTranslation.ts            # Custom hook (cache + API)
└── utils/
    ├── translationCache.ts              # localStorage operations
    ├── groqClient.ts                    # Groq API wrapper
    └── urduPrompt.ts                    # Prompt templates

backend/src/ (optional, for distributed caching)
├── services/
│   └── translation_service.py           # LiteLLM wrapper + Qdrant
├── routes/
│   └── translation.py                   # FastAPI endpoint
└── models/
    └── translation.py                   # Pydantic schemas

tests/
├── unit/
│   ├── useUrduTranslation.test.ts       # Hook tests
│   └── translationCache.test.ts         # Cache operations
└── integration/
    └── translation-flow.test.tsx        # Full flow tests
```

**Gate Result (Phase 1)**: ✅ Data model complete, API contract defined, quickstart provided, structure documented

---

## Phase 2: Implementation Tasks (NEXT)

Tasks generated via `/sp.tasks` command:
- [ ] Implement React component (UrduTranslationButton.tsx)
- [ ] Implement custom hook (useUrduTranslation.ts)
- [ ] Implement utils (translationCache.ts, groqClient.ts)
- [ ] Implement React Context (LanguageContext.tsx)
- [ ] Create unit tests (cache, hook, error scenarios)
- [ ] Create integration tests (full flow, offline, personalization)
- [ ] Integrate into all 13 chapters
- [ ] Performance testing & optimization
- [ ] Peer review with Urdu speaker
- [ ] Deploy to GitHub Pages

**Acceptance Criteria**:
- ✅ All 13 chapters have functional button
- ✅ <2s first translation, <500ms cached
- ✅ 100% text coverage (excluding code)
- ✅ Graceful offline fallback
- ✅ Personalization integration working
- ✅ All tests passing
- ✅ Urdu translation quality reviewed
