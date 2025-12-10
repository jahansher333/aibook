# Implementation Tasks: Personalize Button per Chapter

**Feature**: Personalize Button per Chapter
**Branch**: 006-personalize-button
**Created**: 2025-12-10
**Spec**: [specs/006-personalize-button/spec.md](specs/006-personalize-button/spec.md)
**Plan**: [specs/006-personalize-button/plan.md](specs/006-personalize-button/plan.md)

## Implementation Strategy

The personalization system will be implemented using a client-side approach with React components that transform content based on user profiles retrieved from Better-Auth. The system will include a PersonalizeButton component, profile context management, and rule-based content transformation.

## Dependencies

- Better-Auth must be implemented (Feature 004) to provide user profiles
- Docusaurus setup must be complete with 13 chapters
- React 18+ must be available in the Docusaurus environment

## Parallel Execution Examples

- ProfileProvider and PersonalizeButton components can be developed in parallel [P]
- Personalization rules configuration can be developed while components are being built [P]
- CSS styling can be developed in parallel with component logic [P]

## Phase 1: Setup

### Goal
Initialize the personalization system structure and dependencies

- [ ] T001 Create directory structure for personalization components in docusaurus/src/components/PersonalizeButton
- [ ] T002 Create directory structure for profile provider in docusaurus/src/components/ProfileProvider
- [ ] T003 Create directory structure for personalized content in docusaurus/src/components/PersonalizedContent
- [ ] T004 Create utilities directory in docusaurus/src/utils for personalization logic
- [ ] T005 Create styles directory in docusaurus/src/styles for personalization CSS
- [ ] T006 Install any required dependencies for personalization system (if needed)

## Phase 2: Foundational Components

### Goal
Build the core infrastructure needed by all user stories

- [ ] T007 [P] Create ProfileProvider context component in docusaurus/src/components/ProfileProvider/ProfileProvider.tsx
- [ ] T008 [P] Create useProfile hook in docusaurus/src/components/ProfileProvider/useProfile.ts
- [ ] T009 [P] Create TypeScript types for UserProfile in docusaurus/src/components/ProfileProvider/types.ts
- [ ] T010 [P] Create personalization rules configuration in docusaurus/src/utils/personalization-rules.ts
- [ ] T011 [P] Create content transformation utilities in docusaurus/src/utils/content-transformer.ts
- [ ] T012 [P] Create profile helper utilities in docusaurus/src/utils/profile-helpers.ts
- [ ] T013 Implement profile caching logic with localStorage in ProfileProvider
- [ ] T014 Create basic CSS styles for personalization in docusaurus/src/styles/personalization.css

## Phase 3: User Story 1 - One-Click Personalization Button (P1)

### Goal
Implement the core "Personalize this chapter" button functionality with instant content transformation

**Independent Test**: Navigate to Chapter 4 while logged in, locate "Personalize this chapter" button in top-right corner near "Translate to Urdu" button. Click button, verify content transforms within 500ms (check browser DevTools timeline), see visual indicator showing personalization is active (e.g., blue chip "Personalized"), verify no full page reload occurs (URL unchanged, scroll position preserved).

- [ ] T015 [P] [US1] Create PersonalizeButton component in docusaurus/src/components/PersonalizeButton/PersonalizeButton.tsx
- [ ] T016 [P] [US1] Create usePersonalization hook in docusaurus/src/components/PersonalizeButton/usePersonalization.ts
- [ ] T017 [P] [US1] Create PersonalizeButton CSS module in docusaurus/src/components/PersonalizeButton/PersonalizeButton.module.css
- [ ] T018 [US1] Implement button click handler to trigger personalization
- [ ] T019 [US1] Implement content transformation logic in usePersonalization hook
- [ ] T020 [US1] Add visual feedback (loading spinner) during personalization
- [ ] T021 [US1] Implement button state toggle between "Personalize" and "Show Original"
- [ ] T022 [US1] Add visual indicator showing "Personalized for: [Profile Summary]"
- [ ] T023 [US1] Implement content toggle functionality to revert to original content
- [ ] T024 [US1] Add performance timing to ensure <500ms transformation
- [ ] T025 [US1] Test that no page reload occurs during personalization
- [ ] T026 [US1] Position button in top-right corner of chapter header area, adjacent to Urdu translation button

## Phase 4: User Story 2 - No-Reload React State Transformation (P1)

### Goal
Ensure content personalization happens entirely client-side without page reloads, preserving scroll position and form state

**Independent Test**: Open Chapter 6 in DevTools Network tab. Click "Personalize this chapter". Verify: No new HTTP requests to fetch content (except optional API call to load user profile if not cached), No full page reload (measure by checking `document.readyState` before/after, should stay "complete"), No navigation change (URL remains `/docs/ch06-isaac-sim/ch06`), Scroll position preserved (if user scrolled down to section 3, personalization happens without jumping to top), Form state preserved (if chapter has quiz/input fields, values remain intact).

- [ ] T027 [P] [US2] Implement scroll position preservation during personalization
- [ ] T028 [P] [US2] Implement form state preservation during personalization
- [ ] T029 [US2] Ensure URL remains unchanged during personalization
- [ ] T030 [US2] Verify no page navigation occurs during personalization
- [ ] T031 [US2] Minimize network requests during personalization (use cached profile)
- [ ] T032 [US2] Test document.readyState remains "complete" during transformation
- [ ] T033 [US2] Implement React state management for content transformation
- [ ] T034 [US2] Add performance optimization to prevent unnecessary re-renders

## Phase 5: User Story 3 - Integration with Existing Better-Auth Profile (P1)

### Goal
Personalization button automatically fetches user's saved profile from Better-Auth and applies transformation rules

**Independent Test**: Create account via signup form, answer 7 background questions, verify profile saved in Neon. Sign out and sign in again. Navigate to any chapter, click "Personalize this chapter". Verify personalization rules match the profile answers (e.g., if profile says `learning_environment: "local_preferred"`, local setup is prioritized in Chapter 4). Check browser console: no errors fetching profile, React context contains correct profile data.

- [ ] T034 [P] [US3] Implement Better-Auth integration in ProfileProvider
- [ ] T035 [P] [US3] Create API call to fetch user profile from Better-Auth in docusaurus/src/components/ProfileProvider/ProfileProvider.tsx
- [ ] T036 [US3] Implement profile validation and error handling
- [ ] T037 [US3] Ensure profile is cached after initial fetch
- [ ] T038 [US3] Implement fallback to default profile if user has incomplete profile
- [ ] T039 [US3] Test profile availability when personalization button is clicked
- [ ] T040 [US3] Verify personalization rules match profile values correctly
- [ ] T041 [US3] Add console logging for debugging profile access

## Phase 6: User Story 4 - Personalization Preserves Urdu Translation (P2)

### Goal
After clicking "Personalize this chapter", the existing "Translate to Urdu" button still works correctly, translating the personalized content variant

**Independent Test**: Navigate to Chapter 4 as logged-in user. Click "Personalize this chapter" (profile is `learning_environment: "cloud_preferred"`). Verify content shows cloud instructions (e.g., "AWS EC2 g4dn.xlarge instance"). Click "Translate to Urdu". Verify Urdu translation shows cloud instructions in Urdu (not local setup in Urdu). Confirm translation loaded within 2 seconds (cached from Qdrant).

- [ ] T042 [P] [US4] Analyze existing Urdu translation system integration points
- [ ] T043 [P] [US4] Implement profile_hash generation for translation cache keys
- [ ] T044 [US4] Ensure personalized content can be translated by Urdu system
- [ ] T045 [US4] Test that translation cache keys include profile_hash
- [ ] T046 [US4] Verify translation works on both personalized and original content
- [ ] T047 [US4] Implement proper cache key management for personalized content
- [ ] T048 [US4] Test translation toggle after content personalization

## Phase 7: User Story 5 - Button Visibility and Accessibility (P1)

### Goal
"Personalize this chapter" button is visible, accessible, and intuitively placed on every chapter for logged-in users; hidden or disabled for anonymous visitors

**Independent Test**: Visit Chapter 2 while logged out, verify button is hidden or shows "Sign in to personalize". Log in, navigate to same chapter, verify button is now visible in top-right corner. Test keyboard navigation: Tab key reaches button, Enter key activates personalization. Test mobile: button is visible and tappable on smartphones (responsive design). Test screen reader: button has clear label "Personalize this chapter" announced correctly.

- [ ] T049 [P] [US5] Implement button visibility based on authentication status
- [ ] T050 [P] [US5] Add keyboard accessibility (Tab navigation, Enter/Space activation)
- [ ] T051 [US5] Implement proper ARIA attributes for screen readers
- [ ] T052 [US5] Ensure button is responsive and visible on mobile devices
- [ ] T053 [US5] Add proper tooltip and visual feedback for button states
- [ ] T054 [US5] Implement "Sign in to personalize" state for anonymous users
- [ ] T055 [US5] Test button placement in top-right corner of chapter header
- [ ] T056 [US5] Verify button meets accessibility standards (44x44px minimum touch target)

## Phase 8: Integration with All 13 Chapters

### Goal
Add personalization button to all 13 chapters as specified

- [ ] T057 [P] Add PersonalizeButton to Chapter 1 (docusaurus/docs/ch01-intro/ch01.mdx)
- [ ] T058 [P] Add PersonalizeButton to Chapter 2 (docusaurus/docs/ch02-ros2/ch02.mdx)
- [ ] T059 [P] Add PersonalizeButton to Chapter 3 (docusaurus/docs/ch03-modeling/ch03.mdx)
- [ ] T060 [P] Add PersonalizeButton to Chapter 4 (docusaurus/docs/ch04-gazebo/ch04.mdx)
- [ ] T061 [P] Add PersonalizeButton to Chapter 5 (docusaurus/docs/ch05-unity/ch05.mdx)
- [ ] T062 [P] Add PersonalizeButton to Chapter 6 (docusaurus/docs/ch06-isaac-sim/ch06.mdx)
- [ ] T063 [P] Add PersonalizeButton to Chapter 7 (docusaurus/docs/ch07-vision/ch07.mdx)
- [ ] T064 [P] Add PersonalizeButton to Chapter 8 (docusaurus/docs/ch08-kinematics/ch08.mdx)
- [ ] T065 [P] Add PersonalizeButton to Chapter 9 (docusaurus/docs/ch09-locomotion/ch09.mdx)
- [ ] T066 [P] Add PersonalizeButton to Chapter 10 (docusaurus/docs/ch10-manipulation/ch10.mdx)
- [ ] T067 [P] Add PersonalizeButton to Chapter 11 (docusaurus/docs/ch11-conversational/ch11.mdx)
- [ ] T068 [P] Add PersonalizeButton to Chapter 12 (docusaurus/docs/ch12-hardware/ch12.mdx)
- [ ] T069 [P] Add PersonalizeButton to Chapter 13 (docusaurus/docs/ch13-capstone/ch13.mdx)
- [ ] T070 Wrap all chapters with ProfileProvider context

## Phase 9: Edge Cases and Error Handling

### Goal
Handle all specified edge cases gracefully

- [ ] T071 [P] Implement loading state for when profile hasn't loaded yet
- [ ] T072 [P] Add error handling for profile fetch failures
- [ ] T073 Handle case where chapter has no personalizable content
- [ ] T074 Implement timeout for personalization transformation (>500ms)
- [ ] T075 Handle slow connection scenarios with appropriate loading states
- [ ] T076 Address conflicting profile answers without conflict resolution
- [ ] T077 Implement toast notification when profile is updated in another tab
- [ ] T078 Add timeout fallback (>2s) to show default content if transformation fails

## Phase 10: Polish & Cross-Cutting Concerns

### Goal
Complete the implementation with final touches and quality improvements

- [ ] T079 [P] Add comprehensive TypeScript types for all personalization components
- [ ] T080 [P] Implement performance monitoring and timing metrics
- [ ] T081 Add unit tests for personalization logic (if requested)
- [ ] T082 Add integration tests for personalization flow (if requested)
- [ ] T083 Optimize personalization rules evaluation performance
- [ ] T084 Add visual regression test scenarios for different profile types
- [ ] T085 Document personalization rules configuration format
- [ ] T086 Create troubleshooting guide for personalization issues
- [ ] T087 Verify all success criteria from spec are met
- [ ] T088 Performance test: ensure <500ms transformation time
- [ ] T089 Accessibility test: verify all WCAG requirements are met
- [ ] T090 Cross-browser compatibility test: Chrome, Firefox, Safari, Edge
- [ ] T091 Mobile responsiveness test: ensure button works on all screen sizes
- [ ] T092 Integration test: verify compatibility with existing features (Urdu translation, RAG chatbot)