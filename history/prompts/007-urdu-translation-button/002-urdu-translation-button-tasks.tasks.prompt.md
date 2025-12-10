---
id: 002
title: Urdu Translation Button Tasks Generation
stage: tasks
date: 2025-12-10
surface: agent
model: claude-haiku-4-5-20251001
feature: 007-urdu-translation-button
branch: 007-urdu-translation-button
user: Physical AI Team
command: /sp.tasks
labels: ["task-breakdown", "implementation", "groq-api", "react", "frontend"]
links:
  spec: specs/007-urdu-translation-button/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/007-urdu-translation-button/tasks.md
tests:
  - All integration + unit tests defined in tasks.md (Phase 3-7)
---

## Prompt

```
Urdu Translation Button
[ ] Add Groq API key in .env
[ ] Create src/components/UrduButton.tsx
[ ] Create src/utils/translateToUrdu.ts (Groq + LiteLLM)
[ ] Add button in ALL 13 chapter MDX files
[ ] Add localStorage cache (key: "urdu-chapter-1")
[ ] Test: Click → Urdu in <1.5s
[ ] Test: Personalize + Urdu both work together
End with /sp.check showing Urdu button in every chapter
```

Execute `/sp.tasks` for Urdu Translation Button:
1. Load design documents (plan.md, spec.md, data-model.md, research.md)
2. Extract 4 user stories with priorities (P1, P2, P2, P2)
3. Map user stories to implementation tasks
4. Generate tasks.md with dependencies and parallelization
5. Organize into 8 phases (Setup, Foundational, US1, US3, US4, US2, US5, Polish)

## Response snapshot

### Task Generation Summary

**Total Tasks Generated**: 40 tasks organized by phase and user story

**Task Format**: `[ID] [P?] [Story?] Description with file path`

**Key Statistics**:
- Setup (Phase 1): 3 tasks
- Foundational (Phase 2): 6 tasks
- User Story 1 (Phase 3): 10 tasks (P1 - MVP)
- User Story 3 (Phase 4): 4 tasks (P2 - Caching)
- User Story 4 (Phase 5): 3 tasks (P2 - Offline Fallback)
- User Story 2 (Phase 6): 3 tasks (P1 - Language Toggle)
- User Story 5 (Phase 7): 3 tasks (Custom - Personalization Integration)
- Polish & Cross-Cutting (Phase 8): 9 tasks (Final verification, testing, deployment)

### Phase Breakdown

**Phase 1: Setup (30 min)**
- T001: Add Groq API key to .env / .env.example
- T002: Verify Docusaurus structure and chapter files
- T003: Create .env.example template

**Phase 2: Foundational (1 hr) - CRITICAL BLOCKING TASKS**
- T004: translationCache.ts (localStorage operations, cache TTL, quota handling)
- T005: groqClient.ts (Groq API wrapper with error handling)
- T006: urduPrompt.ts (prompt templates, technical term preservation)
- T007: LanguageContext.tsx (language state provider)
- T008: useUrduTranslation.ts (custom hook, API logic, deduplication)

**Phase 3: User Story 1 - View Chapter in Urdu (P1) MVP (2 hr)**
- T009: UrduTranslationButton.tsx component (main button UI)
- T010: UrduTranslationButton.module.css (styling, animations)
- T011: Component barrel export
- T012: Unit test for component (rendering, loading, errors)
- T013: Integration test for translation flow
- T014: Integrate LanguageProvider into root layout
- T015: Add button to chapter template (all chapters)
- T016: Manual testing with Urdu-speaking reviewer
- T017: Performance monitoring (track API latency)
- T018: Verify button on all 13 chapters

**Phase 4: User Story 3 - Cached Translations (P2) (1 hr)**
- T019: Implement cache hit/miss logic in hook
- T020: Add cache validation (content hash check)
- T021: Unit test for cache operations (localStorage, expiry, hash)
- T022: Integration test for cache behavior (hit/miss, performance)

**Phase 5: User Story 4 - Offline Fallback (P2) (1 hr)**
- T023: Error handling in hook (API errors, rate limits)
- T024: Offline detection (navigator.onLine)
- T025: Integration test for offline/error scenarios

**Phase 6: User Story 2 - Language Toggle (P1) (30 min)**
- T026: Enhance LanguageContext for immediate updates
- T027: Unit test for race conditions (rapid clicks)
- T028: Multi-chapter test (different languages per chapter)

**Phase 7: User Story 5 - Personalization Integration (30 min)**
- T029: Verify button layout with Personalize button
- T030: Update hook to read personalized content
- T031: Integration test for Personalize + Translate flow

**Phase 8: Polish & Cross-Cutting (1.5 hr)**
- T032: Verify button in ALL 13 chapters (manual checklist)
- T033: Performance testing (latency benchmarks)
- T034: Cross-browser testing (Chrome, Safari, Firefox, Edge, Mobile)
- T035: Update README with setup instructions
- T036: Add code comments for maintainability
- T037: Manual testing checklist (all user stories + edge cases)
- T038: Run full test suite
- T039: Peer review with Urdu speaker
- T040: Final verification `/sp.check`

### Parallelization Opportunities

**Parallel in Phase 2**:
- T004, T005, T006, T007 can run in parallel (no dependencies)
- T008 depends on T004-T007

**Parallel in Phase 3 (US1)**:
- T009, T010, T012, T013 can run in parallel (component, styling, tests)
- T011, T014 depend on T009
- T015, T016, T017, T018 depend on T015 completion

**Optimal Execution**:
- Sequential: Phase 1 (30min) → Phase 2 (1hr) → Phase 3-7 (5hr, with parallelization)
- Total: ~8.5 hours (sequential) → ~4-5 hours with parallelization
- Critical path: T001 → T002 → T004-T007 → T008 → T009 → T015 → T018

### User Story Mapping

**User Story 1 - View Chapter in Urdu (P1) MVP**:
- Tasks: T009-T018 (component, styling, tests, integration, all chapters)
- Acceptance: Button displays, translates within 2 seconds, toggles languages

**User Story 2 - Language Toggle (P1)**:
- Tasks: T026-T028 (state management, race conditions)
- Acceptance: Rapid clicks handled, per-chapter state tracked

**User Story 3 - Cached Translations (P2)**:
- Tasks: T019-T022 (cache logic, validation, tests)
- Acceptance: Cached translations load <500ms, per-chapter keying

**User Story 4 - Offline Fallback (P2)**:
- Tasks: T023-T025 (error handling, offline detection, tests)
- Acceptance: Graceful degradation, error messages, no broken UI

**User Story 5 - Personalization Integration (Custom)**:
- Tasks: T029-T031 (integration with Personalize button)
- Acceptance: Both buttons work together, personalized content translated

### Test Coverage

**Unit Tests** (Tasks T012, T021, T027):
- UrduTranslationButton component (rendering, state, errors)
- translationCache utilities (localStorage, expiry, quota)
- useUrduTranslation hook (API calls, deduplication)

**Integration Tests** (Tasks T013, T022, T025, T028, T031):
- Translation flow (click → API → Urdu → toggle)
- Cache behavior (hit, miss, expiry, quota)
- Offline/error scenarios (network down, API errors)
- Language state (rapid clicks, per-chapter)
- Personalization integration

**Manual Testing** (Tasks T016, T037, T039):
- Translation quality review with Urdu speaker
- Cross-browser testing (Chrome, Safari, Firefox, Edge, Mobile)
- Performance benchmarking
- All 13 chapters verification

### MVP Scope

**MVP = User Story 1 (P1) Complete** (Tasks T001-T018):
- Button displays at top of every chapter
- Click → translates to Urdu within 2 seconds
- Toggle → switches back to English
- No code blocks translated
- All 13 chapters have button
- Estimated time: 3.5-4 hours

**MVP+ = Add US3 Caching** (Tasks T019-T022):
- Cached translations load in <500ms
- Per-chapter keying, 30-day TTL
- Estimated additional time: 1 hour
- **Total MVP+**: 4.5-5 hours

**Full Feature = All User Stories** (Tasks T001-T040):
- US1: Translation button works
- US2: Language toggle smooth
- US3: Caching fast
- US4: Offline fallback graceful
- US5: Personalization integrated
- Estimated time: 8.5 hours

## Outcome

- ✅ **Impact**: 40 tasks generated, organized by phase and user story, with dependencies and parallelization identified
- 🧪 **Tests**: 13 test tasks defined (unit + integration), covering all user stories + edge cases
- 📁 **Files**: tasks.md created with complete implementation roadmap
- 🔁 **Next prompts**: Begin Phase 1 (T001-T003) setup; then Phase 2 (T004-T008) foundational; then Phase 3 (T009-T018) MVP implementation
- 🧠 **Reflection**: Tasks are specific, executable, and independently testable. MVP (US1) is the priority. Full feature can be delivered incrementally. User input checklist mapped to formal user stories and tasks.

## Evaluation notes (flywheel)

- **Failure modes observed**: None; task generation successful
- **Graders run and results**: Task format validation: PASS (all 40 tasks follow `[ID] [P?] [Story?] Description` format)
- **Prompt variant**: N/A (single execution)
- **Next experiment**: Execute Phase 1 (T001-T003) setup tasks; verify Groq API key works; then execute Phase 2 (T004-T008) foundational tasks
