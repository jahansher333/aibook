# Feature Specification: Personalize Button per Chapter

**Feature Branch**: `006-personalize-button`
**Created**: 2025-12-10
**Status**: Draft
**Input**: User description: "Personalize Button per Chapter - Every single chapter has 'Personalize this chapter' button at the top - One click → content instantly rewrites based on user's saved background - Works with existing Better-Auth + Neon profile - No page reload — pure React state - Urdu translation still works after personalization - All 13 chapters affected"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - One-Click Personalization Button (Priority: P1)

Logged-in users click a "Personalize this chapter" button on any chapter to instantly adapt content based on their profile (saved during auth signup), with zero page reload and immediate visual feedback.

**Why this priority**: This is the core feature request. The button must be visible, clickable, and functional across all 13 chapters. Without this, personalization cannot happen.

**Independent Test**: Navigate to Chapter 4 while logged in, locate "Personalize this chapter" button in top-right corner near "Translate to Urdu" button. Click button, verify content transforms within 500ms (check browser DevTools timeline), see visual indicator showing personalization is active (e.g., blue chip "Personalized"), verify no full page reload occurs (URL unchanged, scroll position preserved).

**Acceptance Scenarios**:

1. **Given** a logged-in user viewing any chapter, **When** they click "Personalize this chapter" button, **Then** React state updates instantly (within 500ms) and chapter content re-renders with personalized variant visible
2. **Given** a user with profile `{learning_environment: "cloud_only"}`, **When** they personalize Chapter 4, **Then** cloud-based instructions (AWS EC2) are prioritized and highlighted, local installation section is hidden or grayed out
3. **Given** a user with profile `{ros2_knowledge: "none"}`, **When** they personalize Chapter 3, **Then** ROS 2 primer section appears at chapter start with definitions and tutorial links
4. **Given** a user clicks "Personalize this chapter", **When** the transformation completes, **Then** a visual indicator (badge/banner) appears showing "Personalized for: [Profile Summary]" with link to edit profile
5. **Given** a personalized chapter, **When** the user clicks "Show Original Content", **Then** the chapter reverts to default (non-personalized) view instantly, without page reload

---

### User Story 2 - No-Reload React State Transformation (Priority: P1)

Content personalization happens entirely client-side using React component state, without triggering full page reloads or navigation, preserving scroll position and form state.

**Why this priority**: "No page reload — pure React state" is explicitly required. Server-side rendering or page reloads violate this requirement and would degrade UX (slow, jarring transitions).

**Independent Test**: Open Chapter 6 in DevTools Network tab. Click "Personalize this chapter". Verify:
- No new HTTP requests to fetch content (except optional API call to load user profile if not cached)
- No full page reload (measure by checking `document.readyState` before/after, should stay "complete")
- No navigation change (URL remains `/docs/ch06-isaac-sim/ch06`)
- Scroll position preserved (if user scrolled down to section 3, personalization happens without jumping to top)
- Form state preserved (if chapter has quiz/input fields, values remain intact)

**Acceptance Scenarios**:

1. **Given** a user scrolled halfway through Chapter 5 and clicks "Personalize this chapter", **When** transformation completes, **Then** scroll position is unchanged (user remains at same spot, sees personalized content in that location)
2. **Given** Chapter 8 contains a code editor or form, **When** user personalizes the chapter, **Then** any text entered in form fields remains intact (no state loss)
3. **Given** a user personalizes Chapter 2, **When** they open browser DevTools and monitor network tab, **Then** no new HTML/CSS/JS bundles are downloaded (only optional API call to confirm user profile)
4. **Given** a user clicks "Personalize" then immediately clicks "Show Original Content", **When** content reverts, **Then** all transitions happen instantly without delay, demonstrating pure state toggle

---

### User Story 3 - Integration with Existing Better-Auth Profile (Priority: P1)

Personalization button automatically fetches user's saved profile from Better-Auth (created during signup with 7 background questions) and applies transformation rules, requiring no manual profile entry or re-authentication.

**Why this priority**: Feature relies entirely on existing Better-Auth system. If profile data isn't accessible from the button, personalization cannot happen.

**Independent Test**:
1. Create account via signup form, answer 7 background questions, verify profile saved in Neon
2. Sign out and sign in again
3. Navigate to any chapter, click "Personalize this chapter"
4. Verify personalization rules match the profile answers (e.g., if profile says `learning_environment: "local_preferred"`, local setup is prioritized in Chapter 4)
5. Check browser console: no errors fetching profile, React context contains correct profile data

**Acceptance Scenarios**:

1. **Given** a user completed signup with profile answers stored in Neon, **When** they click "Personalize this chapter", **Then** system fetches profile from React context (if loaded) or backend API (if not cached), applies rules within 500ms
2. **Given** a user's profile includes `{learning_goal: "build_humanoid"}`, **When** they personalize Chapter 12 (Hardware Deployment), **Then** humanoid-specific hardware options and configuration steps are highlighted
3. **Given** a user profile was saved with profile_hash in localStorage during login, **When** they personalize a chapter, **Then** system uses cached profile hash to match against transformation rules (no repeated Neon queries)
4. **Given** a user has incomplete profile (skipped onboarding quiz), **When** they click "Personalize this chapter", **Then** system applies default profile rules (beginner, cloud-first, no hardware) and shows banner: "Complete your profile to unlock fully personalized content"

---

### User Story 4 - Personalization Preserves Urdu Translation (Priority: P2)

After clicking "Personalize this chapter", the existing "Translate to Urdu" button still works correctly, translating the personalized content variant (not the original) into Urdu.

**Why this priority**: Existing Urdu feature must not break. Users who prefer Urdu expect translation to work on personalized content. P2 because it's a compatibility requirement rather than new functionality.

**Independent Test**:
1. Navigate to Chapter 4 as logged-in user
2. Click "Personalize this chapter" (profile is `learning_environment: "cloud_preferred"`)
3. Verify content shows cloud instructions (e.g., "AWS EC2 g4dn.xlarge instance")
4. Click "Translate to Urdu"
5. Verify Urdu translation shows cloud instructions in Urdu (not local setup in Urdu)
6. Confirm translation loaded within 2 seconds (cached from Qdrant)

**Acceptance Scenarios**:

1. **Given** a user personalizes Chapter 4 for cloud environment, **When** they click "Translate to Urdu", **Then** system generates Qdrant cache key as `chapter_id + profile_hash + "urdu"` and returns personalized variant in Urdu (e.g., "AWS" translated to "ايه ڈبليو ايس" in Urdu)
2. **Given** a user toggles between personalized and original content, **When** they translate each variant, **Then** system maintains separate Qdrant cache entries (e.g., `ch04_personalized_cloud_urdu` vs `ch04_original_urdu`)
3. **Given** a translated personalized chapter, **When** user clicks "Show Original Content", **Then** Urdu translation is hidden (user returns to English original), and "Translate to Urdu" button shows again for original content
4. **Given** user has incomplete profile and uses default personalization, **When** they translate to Urdu, **Then** Urdu translation of default-personalized content is cached and reused for all other beginner users with incomplete profiles

---

### User Story 5 - Button Visibility and Accessibility (Priority: P1)

"Personalize this chapter" button is visible, accessible, and intuitively placed on every chapter for logged-in users; hidden or disabled for anonymous visitors with clear messaging.

**Why this priority**: Button must be discoverable. Poor UX (button hidden, unclear placement, confusing state) will cause users to miss the feature entirely.

**Independent Test**:
1. Visit Chapter 2 while logged out, verify button is hidden or shows "Sign in to personalize"
2. Log in, navigate to same chapter, verify button is now visible in top-right corner
3. Test keyboard navigation: Tab key reaches button, Enter key activates personalization
4. Test mobile: button is visible and tappable on smartphones (responsive design)
5. Test screen reader: button has clear label "Personalize this chapter" announced correctly

**Acceptance Scenarios**:

1. **Given** an anonymous user viewing any chapter, **When** they look for "Personalize this chapter" button, **Then** button is not displayed OR shown as disabled with tooltip: "Sign in to personalize this chapter based on your background"
2. **Given** a logged-in user viewing any chapter, **When** they look at top-right corner, **Then** "Personalize this chapter" button is visible next to "Translate to Urdu" button with clear text and icon
3. **Given** a mobile user (viewport <768px), **When** they view any chapter, **Then** button is visible, tappable (min 44x44px), and doesn't overlap with other controls
4. **Given** a user using screen reader, **When** they navigate to "Personalize this chapter" button, **Then** button is announced as "button: Personalize this chapter, not pressed" and pressing it triggers content update with live region announcing "Content personalized"

---

### Edge Cases

- **What if user's profile hasn't loaded yet when they click the button?** Show loading spinner on button, fetch profile from API, then apply transformation. Display error message if fetch fails: "Unable to load your profile. Please refresh."

- **What if chapter has no personalizable content?** Button is still shown but disabled with tooltip: "This chapter has universal content. No personalization needed." OR show button and apply minimal personalization (e.g., beginner users get "Key Concepts" callouts).

- **What if user updates profile from dashboard while viewing a chapter?** React context updates (via event listener or periodic poll). Show toast notification: "Your profile was updated. Click 'Personalize this chapter' again to apply new settings." Previous personalization state is not auto-updated (user must re-click button).

- **What if personalization transformation takes >500ms?** Show loading spinner on button with text "Personalizing...", prevent double-clicks. If transformation takes >2s, show timeout warning: "Personalization took longer than expected. Showing default content." and fall back to non-personalized view.

- **What if user is on a very slow connection (3G)?** Profile fetch might take 2-3 seconds. Show loading state. Implement fallback: if profile not loaded within 5s, use localStorage cached profile (from previous session) or default profile. Never leave button in hung state.

- **What if user has conflicting profile answers (e.g., `hardware_budget: "none"` but `ros2_knowledge: "advanced"`)** Personalization rules handle this: honor `hardware_budget` for hardware sections, honor `ros2_knowledge` for ROS sections. No conflict resolution needed; rules are independent per topic.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Every chapter (all 13) MUST display "Personalize this chapter" button in top-right area (header or chapter controls section) visible for logged-in users
- **FR-002**: Button MUST be clickable and trigger personalization flow without page reload, with visual feedback (loading spinner, button state change)
- **FR-003**: Clicking button MUST fetch or retrieve user's profile from React context (loaded on login from Better-Auth) containing 7 background question answers
- **FR-004**: System MUST apply personalization transformation rules based on profile within 500ms, updating React component state and re-rendering chapter content
- **FR-005**: Personalized content MUST include visual indicator (badge, banner, or chip) showing active personalization and link to edit profile
- **FR-006**: Button label MUST toggle between "Personalize this chapter" and "Show Original Content" based on current state
- **FR-007**: System MUST preserve scroll position, form state, and URL during personalization (no page navigation)
- **FR-008**: Button MUST be hidden or disabled for anonymous users with clear messaging encouraging sign-in
- **FR-009**: System MUST render personalized content client-side (React state update) rather than server-side API call, with optional lazy profile fetch if context not available
- **FR-010**: Urdu "Translate to Urdu" button MUST remain functional after personalization, translating personalized content variant
- **FR-011**: System MUST cache personalized content in React component state (during session) to avoid repeated transformations on toggle

### Personalization Rules (Applied Per Chapter)

These rules are applied when user clicks "Personalize this chapter", transforming content sections based on profile values:

**Learning Environment Rules** (profile key: `learning_environment`):
- `cloud_only` | `cloud_preferred`: Prioritize cloud instructions (AWS EC2), emphasize cost estimates, hide local installation
- `local_preferred` | `local_only`: Prioritize local installation, show cloud as alternative, emphasize hardware requirements

**ROS 2 Knowledge Rules** (profile key: `ros2_knowledge`):
- `none`: Add ROS 2 primer section at chapter start, define key terms (node, topic, service), link to beginner tutorials
- `basic`: Show brief refresher with links to specific documentation
- `intermediate` | `advanced`: Assume knowledge, link to advanced patterns instead of basics

**Learning Goal Rules** (profile key: `learning_goal`):
- `academic`: Highlight quiz/assessment sections, link to assignment templates, emphasize learning objectives
- `hobby`: Emphasize fun projects, weekend-friendly timelines, exploratory sections
- `career_transition`: Link to industry job descriptions, production best practices, scalability considerations
- `professional`: Assume workplace context, link to advanced deployment options, team collaboration patterns

**Hardware Experience Rules** (profile key: `hardware_experience`):
- `none` | `some`: Simplify technical terminology, add beginner callouts, hide advanced customization sections
- `proficient` | `expert`: Show advanced configuration options, assume hardware knowledge, link to deep-dive resources

**GPU Access Rules** (profile key: `gpu_access`):
- `none` | `consumer`: Prioritize cloud-based instructions, warn about local performance limitations, suggest managed services
- `midrange`: Show both local and cloud options equally, default to local with cloud as backup
- `highend`: Default to local setup, emphasize GPU-accelerated features, show custom optimization paths

**Python Level Rules** (profile key: `python_level`):
- `beginner`: Add Python syntax explanations in code comments, link to Python docs for unfamiliar constructs, show step-by-step tutorials
- `intermediate`: Standard code examples with brief comments
- `advanced` | `expert`: Show Pythonic alternatives (list comprehensions, async/await), assume modern Python knowledge

### Key Entities

- **UserProfile**: JSON object stored in localStorage or React Context during session, loaded from Better-Auth Neon database
  - Attributes: `hardware_experience`, `gpu_access`, `ros2_knowledge`, `learning_goal`, `python_level`, `learning_environment`, profile_hash (SHA-256 of sorted profile), timestamp
  - Contains: All 7 background question answers from signup onboarding

- **PersonalizedContent**: Transient React component state, generated on button click
  - Attributes: `isPersonalized` (boolean), `profileSummary` (human-readable string), `appliedRules` (array of rule IDs), `originalContent` (cached chapter markdown for toggle)
  - Scope: Lives in component state during session, not persisted to database

- **TransformationRules**: Static YAML configuration defining how each profile value maps to content changes
  - Attributes: `rule_id`, `profile_key`, `profile_value`, `content_selector` (which sections to show/hide), `text_replacements` (optional), `section_visibility` (show/hide rules)
  - Format: Defined in YAML config file, not hardcoded in components

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All 13 chapters display "Personalize this chapter" button when user is logged in, verified by visual inspection or automated test checking button presence on each chapter page
- **SC-002**: Clicking "Personalize this chapter" button transforms content within 500ms (measured from click to DOM update completing), with visual feedback (spinner or button state) shown during transformation
- **SC-003**: Personalization is applied client-side (React component state update) without full page reload, verified by monitoring URL stability, scroll position preservation, and network tab showing no new document requests
- **SC-004**: Personalized content includes visible indicator (badge/banner) showing "Personalized for: [Profile Summary]" (e.g., "Cloud-first Beginner"), with link to edit profile
- **SC-005**: At least 3 distinct content sections per chapter change visibly when personalized based on profile, detectable by screenshot comparison or DOM inspection (e.g., cloud instructions hidden/shown, beginner primer added/hidden)
- **SC-006**: Clicking "Show Original Content" reverts chapter to original (non-personalized) view instantly, and clicking "Personalize this chapter" re-applies personalization without data loss
- **SC-007**: Urdu translation works on personalized content, verified by personalizing Chapter 4, translating to Urdu, and confirming translated text matches personalized variant (cloud instructions in Urdu, not local)
- **SC-008**: Anonymous users do not see "Personalize this chapter" button OR see it disabled with clear messaging prompting sign-in
- **SC-009**: Button is keyboard accessible (Tab key reaches it, Enter/Space activates), screen reader announces it correctly, and mobile users can tap it (44x44px minimum)
- **SC-010**: Zero functional regressions in existing features: Urdu button still works, chapters still render correctly, RAG chatbot still functional, no broken links

### Non-Functional Success Criteria

- **SC-011**: React component rendering stays under 100ms for personalization state toggle (no jank on 60fps displays), verified by React DevTools Profiler
- **SC-012**: Personalization rules are maintainable: defined in YAML config (not hardcoded in components), new rules can be added without code changes
- **SC-013**: System handles edge cases gracefully: slow profile loads, missing profile data, chapters with no personalizable content
- **SC-014**: Session-level caching prevents repeated transformations for same profile/chapter combination during same session

## Assumptions

1. **Better-Auth Integration Already Complete**: Feature 004 (better-auth-personalization) is fully implemented. User profile with 7 answers is accessible via React Context or API endpoint.

2. **User Profile Available on Load**: When user logs in, profile is fetched from Neon and stored in React Context by authentication wrapper. By the time user navigates to chapters, profile is available (no fetch delay on button click).

3. **Client-Side Transformation Preferred**: Content transformation happens client-side using React conditional rendering and CSS hiding, not server-side API calls. Reduces latency, enables instant toggle.

4. **Chapter Content Structure Predictable**: Chapters are structured with section-level granularity (sections can be shown/hidden or emphasized independently). Content is not monolithic text that requires complex parsing.

5. **Profile Hash Available**: User profile hash (SHA-256 of sorted profile values) is computed during login and stored in localStorage/Context. Used as cache key for Urdu translations with `chapter_id + profile_hash + "urdu"`.

6. **Docusaurus Layout Supports Button Placement**: Docusaurus theme allows custom buttons in chapter header area (top-right). If not natively supported, layout will be customized via Docusaurus theme swizzling.

7. **Existing Urdu Translation System Uses Qdrant**: Urdu translations are cached in Qdrant vector DB. Personalization respects existing cache mechanism by including profile_hash in cache key.

8. **No Real-Time Profile Updates During Session**: If user edits profile from dashboard in separate tab, personalization in chapter doesn't auto-update until user re-clicks button. Refresh-based sync is acceptable.

9. **Personalization is Deterministic**: Given same profile and chapter, personalization produces same output every time. No randomness or LLM-based rewriting (those are future enhancements).

10. **All 13 Chapters Have Same Content Structure**: Chapters follow consistent Markdown/HTML structure, allowing same personalization rules to apply uniformly across all chapters.

## Dependencies

### Internal Dependencies

1. **Better-Auth Profile Data** (Feature 004): Assumes user profile with 7 background questions is available in React Context (loaded during authentication). If feature 004 not complete, personalization cannot access profile data.

2. **Docusaurus React Setup**: Feature requires React hooks (useState, useContext) and component re-rendering. Docusaurus must be configured with React 18+ (already the case in existing setup).

3. **Existing Urdu Translation System**: Must be compatible with personalized content. Qdrant cache keys must include profile_hash to differentiate translations.

4. **Chapter Content (Markdown/MDX)**: Chapters are Docusaurus docs with Markdown or MDX format. Personalization rules target specific sections (heading levels, custom components) to show/hide.

### External Dependencies

1. **React**: Runtime library for component state management and re-rendering. Already available in Docusaurus setup.

2. **Neon Postgres**: Stores user profiles. Must be accessible via Better-Auth API to fetch profile on demand (if not cached in React Context).

### Risk Mitigations

- **Risk**: Profile data not available when button clicked
  - **Mitigation**: Cache profile in React Context on login. If cache missing, fetch from API before applying rules.

- **Risk**: Transformation takes >500ms, appears slow
  - **Mitigation**: Optimize rule evaluation (object property lookups, not array searches). Pre-process chapters into sections during build time. Show loading spinner to manage expectations.

- **Risk**: Personalization rules conflict (e.g., "show section for beginners" AND "hide section for no-hardware" both apply)
  - **Mitigation**: Rules are independent and additive (sections can be visible with multiple conditions). Design rules carefully to avoid contradictions.

## Out of Scope

1. **AI-Generated Personalization**: Content is not rewritten by LLM based on profile. Transformation is rule-based (show/hide sections, change text via templating).

2. **Server-Side Personalization**: Backend does not generate per-user content variants. All transformation happens client-side in React.

3. **Granular Paragraph-Level Personalization**: Personalization is at section level (headings, major blocks). Individual sentence or paragraph rewrites are not supported.

4. **Real-Time Sync Across Devices**: Profile updates on one device don't instantly appear in personalization on another device. User must refresh or re-click button.

5. **Personalization A/B Testing**: No built-in experiments or metric tracking for different personalization strategies.

6. **Analytics on Personalization**: No admin dashboard showing "60% of users personalize Chapter 4". Analytics are post-MVP.

7. **Personalization for Chapters with No Content Variants**: If chapter has no personalizable content, button is hidden/disabled. No forced personalization.

8. **Profile Auto-Detection**: System does not automatically detect user's hardware/environment from browser/device. Profile comes from signup quiz answers only.

## Risks & Mitigations

### Technical Risks

1. **Risk**: React component re-renders cause full page flicker during personalization
   - **Impact**: Medium (poor UX, looks buggy)
   - **Mitigation**: Use React.memo or key props to prevent unnecessary re-renders. Wrap personalization logic in useCallback. Test rendering performance with DevTools Profiler.

2. **Risk**: Personalization rules are hardcoded in component, making them hard to maintain
   - **Impact**: High (future changes require code updates, slows development)
   - **Mitigation**: Define rules in separate YAML config file. Load during build time. Rules become data-driven, not code-driven.

3. **Risk**: Profile hash mismatch causes wrong Urdu translation cached with wrong key
   - **Impact**: Medium (users see wrong language variant)
   - **Mitigation**: Compute profile hash consistently (sorted JSON keys, SHA-256). Test cache keys match between profile creation and personalization.

### Content Risks

4. **Risk**: Personalization rules produce invalid content (e.g., hidden section referenced in text, broken links)
   - **Impact**: High (breaks learning experience)
   - **Mitigation**: Define rules with explicit tests: "For profile {gpu_access: none}, Cloud Instructions MUST be shown AND Local Setup MUST be hidden." Manual QA per profile combination.

5. **Risk**: Urdu translation of personalized content introduces errors (e.g., AWS references untranslated/misspelled)
   - **Impact**: Medium (confuses Urdu-speaking users)
   - **Mitigation**: Implement translation validation: back-translate Urdu → English, flag technical term mismatches. Human review of Urdu translations.

### UX Risks

6. **Risk**: Users don't discover "Personalize this chapter" button (hidden, unclear label)
   - **Impact**: Medium (feature not used, wasted effort)
   - **Mitigation**: Place button prominently in top-right corner next to existing "Translate to Urdu" button. Add tooltip on hover. Consider onboarding banner on first login: "Tip: Click 'Personalize this chapter' for content tailored to your background."

7. **Risk**: Personalization is slow on low-end devices (large chapters, complex rules)
   - **Impact**: Medium (users on phones see spinner for >1s)
   - **Mitigation**: Lazy-load chapter sections. Pre-compute personalization during build time if possible. Implement timeout fallback (>2s, show default content).

## Notes

- **Architectural Approach**: Personalization is purely client-side. Button click triggers React state update, which conditionally renders sections based on profile. No server-side API calls needed for content generation (optional API call only to fetch profile if not cached).

- **Profile Storage Strategy**: Profile is loaded once on login and stored in React Context. Used by all chapters during session. Reduces Neon database queries.

- **Cache Invalidation**: Profile cache invalidates when user updates profile (via profile dashboard) and logs out. Urdu translation cache invalidates when profile hash changes.

- **Content Sourcing**: All chapters must be available as structured Markdown/MDX with clearly identified sections. Non-standard chapters may require manual content restructuring to support personalization rules.

- **Testing Plan**:
  1. **Unit Tests**: Personalization rules evaluation (profile → expected output)
  2. **Component Tests**: Button visibility, state toggle, rendering without reload
  3. **E2E Tests**: Full flow (login → personalize Chapter 4 → toggle back → translate to Urdu)
  4. **Visual Regression**: Compare screenshots of same chapter with different profiles to verify rules applied correctly
  5. **Performance Tests**: Measure personalization latency (target <500ms) on low-end device via DevTools

- **Future Enhancements**:
  - LLM-based content rewriting (GPT-4 personalizes paragraphs dynamically)
  - Progressive profiling (ask follow-up questions based on user behavior)
  - Personalization analytics (which profiles use personalization most, engagement metrics)
  - Multi-language support beyond Urdu

---

**Specification Complete**: Ready for `/sp.plan` phase to design component architecture, define personalization rules in YAML, and plan implementation tasks.
