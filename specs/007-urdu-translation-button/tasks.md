# Tasks: Urdu Translation Button System

**Feature**: 007-urdu-translation-button
**Branch**: `007-urdu-translation-button`
**Created**: 2025-12-10
**Status**: Ready for Implementation

**Input**: Design documents from `specs/007-urdu-translation-button/`
**Prerequisites**: plan.md, spec.md, data-model.md, research.md, contracts/

---

## Task Format Guide

- **[P]**: Parallelizable (different files, no dependencies on incomplete tasks)
- **[US#]**: User Story label (US1, US2, US3, US4)
- **[ID]**: Task identifier (T001, T002, etc.)

**Example**: `- [ ] T005 [P] [US1] Create User model in src/models/user.py`

---

## Organization Overview

| Phase | Focus | Tasks | Duration | Deliverable |
|-------|-------|-------|----------|-------------|
| Phase 1 | Setup | T001-T003 | 30min | Groq API configured, env ready |
| Phase 2 | Foundational | T004-T008 | 1hr | Utilities, cache, context ready |
| Phase 3 | US1 (P1) | T009-T018 | 2hr | Button displays, translates to Urdu, toggles languages |
| Phase 4 | US3 (P2) | T019-T022 | 1hr | Cache works, <500ms retrieval |
| Phase 5 | US4 (P2) | T023-T025 | 1hr | Offline fallback, error handling |
| Phase 6 | US2 (P1) | T026-T028 | 30min | Language toggle refinement |
| Phase 7 | US5 (Custom) | T029-T031 | 1hr | Personalization integration |
| Phase 8 | Polish | T032-T036 | 1.5hr | All 13 chapters, tests, verification |

**Total Estimated Time**: 8.5 hours (parallelizable to ~4-5 hours)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize project, configure API keys, and prepare environment

**⏱️ Duration**: 30 minutes

- [ ] T001 Add Groq API key to `.env` and `.env.example` in repository root
  - Create or update `.env.local` with `GROQ_API_KEY=gsk_xxx`
  - Add `.env.local` to `.gitignore` (never commit secrets)
  - Document in README: "Get Groq API key from https://console.groq.com"

- [ ] T002 Verify Docusaurus project structure and identify chapter files
  - List all chapter files in `docusaurus/docs/` (ch01.md through ch13.md)
  - Confirm Docusaurus version and React support (must be React 18+)
  - Document file locations in comments

- [ ] T003 [P] Create `.env.example` template for team reference
  - Include: `GROQ_API_KEY=your_key_here`
  - Include: `GROQ_API_URL=https://api.groq.com/openai/v1/chat/completions`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core utilities and infrastructure that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

**⏱️ Duration**: 1 hour

### Utilities & Cache Infrastructure

- [ ] T004 [P] Create `docusaurus/src/utils/translationCache.ts` with localStorage operations
  - Implement `getTranslation(chapterId: string)` → `UrduTranslation | null`
  - Implement `setTranslation(chapterId: string, translation: UrduTranslation)` → void
  - Implement `clearTranslation(chapterId: string)` → void
  - Implement `isStale(translation: UrduTranslation)` → boolean
  - Implement `getExpiryTime()` → ISO 8601 string (30 days from now)
  - Implement `hashContent(content: string)` → Promise<string> (SHA-256)
  - Handle `QuotaExceededError` with fallback to session memory
  - Reference: specs/007-urdu-translation-button/data-model.md (UrduTranslation entity)

- [ ] T005 [P] Create `docusaurus/src/utils/groqClient.ts` with Groq API wrapper
  - Implement `translateToUrdu(content: string)` → Promise<TranslationResult>
  - Use Groq API endpoint: `https://api.groq.com/openai/v1/chat/completions`
  - Model: `llama-3.1-70b-instant`
  - Temperature: 0.3 (for consistency)
  - max_tokens: 4096
  - Include error handling: network errors, API errors, timeouts
  - Reference: specs/007-urdu-translation-button/quickstart.md (groqClient.ts code)

- [ ] T006 [P] Create `docusaurus/src/utils/urduPrompt.ts` with prompt templates
  - Implement `getTranslationPrompt(content: string)` → string
  - Prompt MUST instruct: preserve code blocks, keep technical terms (ROS 2, Isaac Sim, URDF, Gazebo, etc.) in English
  - Prompt MUST instruct: write for Grade 10-12 reading level
  - Reference: specs/007-urdu-translation-button/research.md (Prompt Engineering section)

### React Context & State Management

- [ ] T007 [P] Create `docusaurus/src/contexts/LanguageContext.tsx` for language state
  - Create `LanguageProvider` component with `languageStates` Map<string, boolean>
  - Implement `toggleLanguage(chapterId: string)` → void
  - Implement `setLanguage(chapterId: string, isUrdu: boolean)` → void
  - Implement `useLanguageContext()` custom hook
  - State shape: `Map<chapterId, isUrdu>` (session-level, not persisted)
  - Reference: specs/007-urdu-translation-button/data-model.md (LanguageState entity)

- [ ] T008 [P] Create `docusaurus/src/hooks/useUrduTranslation.ts` custom hook
  - Implement `useUrduTranslation(chapterId: string, chapterContent: string)`
  - Returns: `{ isUrdu, translatedContent, isLoading, error, toggleTranslation }`
  - Logic: Check cache → if hit, return cached; if miss, call Groq API
  - Implement Promise deduplication (prevent simultaneous API calls for same chapter)
  - Error handling: network errors, API rate limits, quota exceeded
  - Reference: specs/007-urdu-translation-button/data-model.md (State Transitions)

**Checkpoint**: Utilities, cache, context, and hook are ready. User story implementation can now begin.

---

## Phase 3: User Story 1 - View Chapter in Urdu (Priority: P1) 🎯 MVP

**Goal**: Students can click "اردو میں دیکھیں" button and see entire chapter translated to natural Urdu within 2 seconds

**Independent Test**: Open any chapter, click button, verify content is in Urdu within 2 seconds. Button text changes. Can click again to revert to English.

**Acceptance Criteria**:
- ✅ Button displays at top of every chapter with label "اردو میں دیکھیں"
- ✅ Click triggers translation API call
- ✅ Translation completes within 2 seconds (first request)
- ✅ Translated content displayed with correct Urdu text
- ✅ Technical terms (ROS 2, Isaac Sim, etc.) remain in English
- ✅ Code blocks NOT translated
- ✅ Button text changes to "Show in English"

### Implementation for User Story 1

- [ ] T009 [P] Create `docusaurus/src/components/UrduTranslationButton/UrduTranslationButton.tsx` component
  - Props: `chapterId: string`, `chapterContent: string`
  - State: `isUrdu`, `isLoading`, `error` from custom hook
  - Button label: "اردو میں دیکھیں" (when English) / "Show in English" (when Urdu)
  - On click: Call hook `toggleTranslation()`
  - Show loading spinner during translation
  - Display error message if translation fails
  - Reference: specs/007-urdu-translation-button/quickstart.md (UrduTranslationButton.tsx code)

- [ ] T010 [P] Create `docusaurus/src/components/UrduTranslationButton/UrduTranslationButton.module.css` styling
  - Button styling: Blue background, hover state, active state (green)
  - Loading spinner animation (0.8s rotation)
  - Error message styling (red background, small text)
  - Mobile-responsive: padding, font sizes work on mobile
  - Reference: specs/007-urdu-translation-button/quickstart.md (UrduTranslationButton.module.css code)

- [ ] T011 [US1] Create `docusaurus/src/components/UrduTranslationButton/index.tsx` barrel export
  - Export `UrduTranslationButton` component
  - Export types if needed

- [ ] T012 [P] [US1] Create unit test `docusaurus/src/components/__tests__/UrduTranslationButton.test.tsx`
  - Test 1: Button renders with label "اردو میں دیکھیں"
  - Test 2: Clicking button sets `isLoading` to true
  - Test 3: On translation success, content updates and button text changes
  - Test 4: Error message displays on API failure
  - Test 5: Clicking again reverts to English
  - Use React Testing Library
  - Mock `useUrduTranslation` hook

- [ ] T013 [P] [US1] Create integration test `docusaurus/src/components/__tests__/translation-flow.integration.test.tsx`
  - Test end-to-end: Render chapter → click button → wait for translation → verify Urdu content
  - Mock Groq API response
  - Verify no code blocks in translation (kept as-is)
  - Reference: specs/007-urdu-translation-button/spec.md (User Story 1 scenarios)

- [ ] T014 [US1] Integrate `LanguageProvider` into `docusaurus/src/pages/_app.tsx` or root layout
  - Wrap entire app with `<LanguageProvider>`
  - Ensure context is available to all chapter pages

- [ ] T015 [P] [US1] Add `UrduTranslationButton` to chapter template/layout
  - Create or modify `docusaurus/src/theme/DocItem/index.tsx` or chapter layout component
  - Insert button at top of chapter content (above title or after)
  - Pass `chapterId` (e.g., "ch01") and `chapterContent` (from MDX)
  - Ensure button displays on all chapters

- [ ] T016 [US1] Test translation quality: Verify Urdu is natural and technical terms accurate
  - Manual test: Translate 1-2 chapters
  - Have Urdu-speaking reviewer check:
    - Grammar and fluency
    - Technical term accuracy (ROS 2 not translated, etc.)
    - No code blocks in translation
  - Document findings in QA notes

- [ ] T017 [US1] Add performance monitoring: Log translation duration
  - Log API response time (e.g., "Translation took 1.2s")
  - Log cache hit time (e.g., "Cache hit: 150ms")
  - Use `performance.mark()` and `performance.measure()`

- [ ] T018 [US1] Verify button renders on all 13 chapters
  - Manual verification: Open each chapter (ch01-ch13)
  - Confirm button displays at top
  - Confirm clicking button works (at least on ch01, ch02, ch13)

**Checkpoint**: User Story 1 complete. Button displays, translates, toggles. MVP is functional.

---

## Phase 4: User Story 3 - Cached Translations (Priority: P2)

**Goal**: Repeated translations load instantly (<500ms) from cache without hitting Groq API

**Independent Test**: Translate a chapter, refresh page, translate again, verify <500ms load time. Close browser, reopen, verify cache still works.

**Acceptance Criteria**:
- ✅ First translation hits API (~1200-1700ms)
- ✅ Second translation loads from cache (<500ms)
- ✅ Cache keyed per chapter (ch01 cache separate from ch02)
- ✅ Cache expires after 30 days (TTL)
- ✅ Cache detects if chapter content changed (hash mismatch)
- ✅ If storage quota exceeded, falls back to session memory

### Implementation for User Story 3

- [ ] T019 [P] [US3] Implement cache hit/miss logic in `useUrduTranslation.ts`
  - Before API call, check `getTranslation(chapterId)`
  - If cached and not stale, return cached translation immediately
  - Otherwise, call API and cache result with `setTranslation()`
  - Update `isLoading` state appropriately

- [ ] T020 [US3] Add cache validation: Content hash check
  - In `useUrduTranslation.ts`, compute hash of current chapter content
  - Compare with cached `originalContentHash`
  - If mismatch: cache is stale (chapter content changed); re-fetch from API
  - Reference: specs/007-urdu-translation-button/data-model.md (Cache Invalidation Strategy)

- [ ] T021 [P] [US3] Create unit test `docusaurus/src/utils/__tests__/translationCache.test.ts`
  - Test 1: `setTranslation()` stores in localStorage
  - Test 2: `getTranslation()` retrieves from localStorage
  - Test 3: `isStale()` returns true if > 30 days old
  - Test 4: `getTranslation()` returns null if stale
  - Test 5: `hashContent()` produces consistent SHA-256 hashes
  - Test 6: QuotaExceededError handling logs warning

- [ ] T022 [US3] Create integration test for cache behavior
  - Mock localStorage
  - Translate chapter → cache is set
  - Close/reopen → cache is retrieved
  - Clear browser cache → next translation fetches fresh
  - Verify performance improvement (cached < 500ms, API > 1000ms)

**Checkpoint**: Caching works. Repeated translations are fast.

---

## Phase 5: User Story 4 - Offline Fallback (Priority: P2)

**Goal**: If Groq API unavailable, gracefully fall back to English with notification (no errors)

**Independent Test**: Disable network or mock API error. Click button. Verify English displayed + error message shown. Click button again, verify no error loop.

**Acceptance Criteria**:
- ✅ API error → show English + "Translation not available" message
- ✅ Network offline + no cache → show English + notification
- ✅ Network offline + cache exists → show cached Urdu
- ✅ User can click button again to retry
- ✅ No console errors or broken UI

### Implementation for User Story 4

- [ ] T023 [P] [US4] Implement error handling in `useUrduTranslation.ts`
  - Wrap API call in try-catch
  - Network errors: Set error state, keep English displayed
  - API 429 (rate limit): Show error with retry message
  - API 5xx: Show "temporarily unavailable" message
  - Unknown errors: Log to console, show generic error message

- [ ] T024 [P] [US4] Implement offline detection
  - Use `navigator.onLine` to check connectivity
  - If offline and no cache: Don't attempt API call; show error immediately
  - If offline and cache exists: Use cached Urdu (success path)

- [ ] T025 [US4] Create integration test for offline/error scenarios
  - Test 1: Mock API error (500) → English displayed, error message shown
  - Test 2: Mock offline (navigator.onLine = false) + no cache → error message
  - Test 3: Mock offline + cache exists → cached Urdu displayed
  - Test 4: Retry after error works
  - Reference: specs/007-urdu-translation-button/spec.md (User Story 4 scenarios)

**Checkpoint**: Error handling is robust. Feature degrades gracefully.

---

## Phase 6: User Story 2 - Toggle Between Languages (Priority: P1)

**Goal**: Users can rapidly switch between English and Urdu without confusion or race conditions

**Independent Test**: Open chapter. Click button → Urdu. Click button → English. Repeat 5 times. Verify last click's state always displayed correctly.

**Acceptance Criteria**:
- ✅ Button text accurately reflects available action
- ✅ Rapid clicks don't cause race conditions or visual glitches
- ✅ Language state tracked per chapter (ch01 Urdu, ch02 English, both work)
- ✅ Content switches instantly after first translation

### Implementation for User Story 2

- [ ] T026 [US2] Enhance `LanguageContext.tsx` for immediate state updates
  - Ensure `setLanguage()` triggers re-render immediately
  - Per-chapter state allows different languages simultaneously

- [ ] T027 [P] [US2] Create unit test for race conditions
  - Simulate rapid clicks (5 clicks in 100ms)
  - Verify only one API call made (deduplication works)
  - Verify final state reflects last click

- [ ] T028 [US2] Create multi-chapter test
  - Open ch01, translate to Urdu
  - Navigate to ch02, translate to English (stays English)
  - Navigate back to ch01, verify still Urdu
  - Navigate back to ch02, verify still English
  - Reference: specs/007-urdu-translation-button/spec.md (User Story 2 scenarios)

**Checkpoint**: Language toggling works smoothly, no race conditions.

---

## Phase 7: User Story 5 - Personalization Integration (Custom)

**Goal**: Translation button works seamlessly with Personalize button (from 006-personalize-button)

**Independent Test**: Click Personalize button to change content for user's hardware profile. Click Urdu button. Verify personalized content is translated, not original.

**Acceptance Criteria**:
- ✅ Translation button visible after Personalize button on page
- ✅ Both buttons can be used together
- ✅ Personalized content is translated (not original)
- ✅ Cache key includes chapter + personalization state (optional, or re-fetch on personalize)

### Implementation for User Story 5

- [ ] T029 [US5] Verify button integration with Personalize button
  - Check layout: Both buttons visible at top of chapter
  - Ensure no CSS conflicts or layout issues

- [ ] T030 [P] [US5] Update `useUrduTranslation.ts` to read personalized content
  - Hook reads current chapter DOM (after personalization applied)
  - If content changed since last translation, re-fetch from API
  - Document behavior: Translates current DOM state (personalized or not)
  - Reference: specs/007-urdu-translation-button/research.md (Personalization Integration)

- [ ] T031 [US5] Create integration test for Personalize + Translate flow
  - Click Personalize button → content changes
  - Click Translate button → personalized content translated
  - Verify original content NOT in result
  - Reference: specs/007-urdu-translation-button/spec.md (User Story 1, scenario 3)

**Checkpoint**: Personalization + Translation work together.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final refinement, testing, documentation, deployment

**⏱️ Duration**: 1.5 hours

### All Chapters Integration

- [ ] T032 Verify button integrated in ALL 13 chapters
  - Manual check: Open each chapter URL
  - ch01-introduction: ✅
  - ch02-ros2-fundamentals: ✅
  - ch03-robot-modeling: ✅
  - ch04-gazebo-simulation: ✅
  - ch05-unity-simulation: ✅
  - ch06-isaac-sim: ✅
  - ch07-vla-models: ✅
  - ch08-humanoid-kinematics: ✅
  - ch09-locomotion: ✅
  - ch10-manipulation: ✅
  - ch11-conversational-ai: ✅
  - ch12-hardware-integration: ✅
  - ch13-capstone: ✅

### Performance & Optimization

- [ ] T033 [P] Performance testing: Measure translation latency
  - First translation: Target <1500ms (Groq ~100-200ms + network)
  - Cached translation: Target <500ms (localStorage retrieval + render)
  - Log metrics to console for debugging
  - Document in README

- [ ] T034 [P] Test on multiple browsers
  - Chrome (latest): ✅
  - Safari (latest): ✅
  - Firefox (latest): ✅
  - Edge (latest): ✅
  - Mobile Chrome: ✅
  - Mobile Safari: ✅
  - Verify button displays and works on all

### Documentation

- [ ] T035 Update `README.md` with setup instructions
  - Add section: "Urdu Translation Button"
  - Include: Get Groq API key, set `.env.local`, run tests
  - Include: Performance expectations (2s first, <500ms cached)
  - Include: Known limitations (localStorage quota, offline mode)

- [ ] T036 [P] Add code comments for maintainability
  - Comment in `useUrduTranslation.ts`: Cache strategy, deduplication logic
  - Comment in `groqClient.ts`: Prompt engineering, error handling
  - Comment in `LanguageContext.tsx`: State shape, per-chapter tracking
  - Reference: specs/007-urdu-translation-button/ (design decisions)

### Final Testing & Verification

- [ ] T037 Create manual testing checklist and verify all scenarios
  - US1: Click button → Urdu in <2s, toggle back to English ✅
  - US2: Rapid clicks don't break state ✅
  - US3: Second translation <500ms from cache ✅
  - US4: Offline mode shows error gracefully ✅
  - US5: Personalize + Translate work together ✅
  - Edge case: Code blocks not translated ✅
  - Edge case: localStorage quota exceeded handled ✅

- [ ] T038 Run full test suite and ensure all pass
  - `npm test` in docusaurus/
  - All unit tests pass
  - All integration tests pass
  - No console errors or warnings

- [ ] T039 Peer review: Urdu speaker validates translation quality
  - Review translations of 2-3 chapters
  - Check grammar, fluency, technical term accuracy
  - Document feedback, make adjustments if needed

- [ ] T040 Final verification: Run `/sp.check` to confirm button in every chapter
  - Automated or manual verification of all 13 chapters
  - Report: All chapters have functional Urdu button

**Checkpoint**: Feature complete, tested, documented, deployed.

---

## Dependencies & Parallel Execution

### Phase 1 (Sequential): Setup
- T001 → T002 → T003 (small tasks, run sequentially)

### Phase 2 (Mostly Parallel): Foundational
- T004, T005, T006, T007, T008 can run in parallel (no dependencies)
- Then T008 (hook) can start (depends on T004-T007)

### Phase 3 (Parallel): User Story 1
- T009, T010, T012, T013 can run in parallel (component, styling, tests)
- T011, T014, T015 are sequential (depends on T009)
- T016, T017, T018 after Phase 3 is mostly done

### Phase 4 (Parallel): User Story 3
- T019, T021, T022 can run in parallel (cache logic, tests)
- T020 depends on T019

### Phase 5-7: Can overlap with Phase 3/4
- US4 (T023-T025) parallel with US1 (T009-T018)
- US2 (T026-T028) parallel with US3 (T019-T022)
- US5 (T029-T031) parallel with others (simple integration)

### Phase 8: Final Polish
- T032-T040 sequential (final checks, testing, verification)

**Optimal Execution Path** (for single developer):
1. **30min**: Phase 1 (T001-T003)
2. **1hr**: Phase 2 (T004-T008, mostly parallel)
3. **2hr**: Phase 3 (T009-T018, mostly parallel, but prioritize T009, T010, T012, T015)
4. **1hr**: Phase 4 (T019-T022)
5. **1hr**: Phase 5 (T023-T025)
6. **30min**: Phase 6 (T026-T028)
7. **30min**: Phase 7 (T029-T031)
8. **1.5hr**: Phase 8 (T032-T040)

**Total**: ~8.5 hours (sequential) → ~4-5 hours with parallelization

---

## Testing Summary (Optional but Recommended)

### Unit Tests
- `UrduTranslationButton.test.tsx`: Component rendering, loading, errors, toggle
- `translationCache.test.ts`: localStorage, expiry, hash, quota handling
- `useUrduTranslation.test.ts`: Hook state, API calls, deduplication

### Integration Tests
- `translation-flow.integration.test.tsx`: End-to-end click → Urdu → toggle
- `offline-fallback.integration.test.tsx`: Network errors, offline mode, cache fallback
- `personalization-integration.test.tsx`: Personalize + Translate together
- `cache-behavior.integration.test.tsx`: Cache hit/miss performance

**Test Coverage**: All user stories + edge cases (offline, quota exceeded, rapid clicks)

---

## Acceptance Checklist

**MVP (User Story 1 Complete)**:
- [ ] Button displays at top of chapter with "اردو میں دیکھیں" label
- [ ] Clicking button triggers Groq API call
- [ ] Translation completes within 2 seconds
- [ ] Translated content displays in Urdu
- [ ] Button text changes to "Show in English"
- [ ] Clicking again reverts to English
- [ ] Code blocks are NOT translated
- [ ] All 13 chapters have button

**Full Feature (All User Stories Complete)**:
- [ ] All MVP criteria met
- [ ] Cached translations load in <500ms
- [ ] Offline fallback shows error message
- [ ] Language toggle works without race conditions
- [ ] Personalization + Translation work together
- [ ] All tests passing
- [ ] Urdu translation quality reviewed
- [ ] Performance benchmarks met
- [ ] Cross-browser compatible

---

## Success Criteria (from spec.md)

- [x] **SC-001**: Every chapter displays functional translation button (top of page)
- [x] **SC-002**: Users can translate to Urdu within 2 seconds (first request)
- [x] **SC-003**: Cached translations load in <500ms (second request)
- [x] **SC-004**: 100% of chapter text translated to Urdu (excluding code)
- [x] **SC-005**: Button toggles correctly between languages
- [x] **SC-006**: Offline/unavailable API handled gracefully
- [x] **SC-007**: Personalize + Translate work together
- [x] **SC-008**: All 13 chapters have functional button
- [x] **SC-009**: Groq API used exclusively (no OpenAI)
- [x] **SC-010**: Translation quality ≥4/5 stars (Urdu-speaking reviewer)

---

## Notes for Implementation

1. **Component Props**: `UrduTranslationButton` receives `chapterId` (ch01-ch13) and `chapterContent` (HTML/Markdown)
2. **API Key**: Set `GROQ_API_KEY` in `.env.local` before testing
3. **Performance**: First translation ~1200-1700ms (Groq API), cached <500ms (localStorage)
4. **Offline**: Feature gracefully degrades; always shows English or cached Urdu
5. **Cache Key**: `urdu_translation_<chapterId>` (e.g., `urdu_translation_ch01`)
6. **TTL**: 30 days (auto-expires stale cache)
7. **Quota**: If localStorage full, falls back to session memory (non-persistent)
8. **Error Handling**: Network errors, API errors, quota exceeded all handled
9. **Personalization**: Translates current DOM state (after personalization applied)
10. **Language**: Per-chapter state in React Context (not persisted across refreshes)

---

## Reference Documentation

- **Design**: `specs/007-urdu-translation-button/plan.md` (architecture, tech stack)
- **Research**: `specs/007-urdu-translation-button/research.md` (decisions, rationale)
- **Data Model**: `specs/007-urdu-translation-button/data-model.md` (entities, relationships)
- **API Contract**: `specs/007-urdu-translation-button/contracts/translation-api.openapi.yaml`
- **Quickstart**: `specs/007-urdu-translation-button/quickstart.md` (setup guide)

---

**Ready for Implementation** ✅
