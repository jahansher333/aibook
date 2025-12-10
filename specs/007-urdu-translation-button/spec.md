# Feature Specification: Urdu Translation Button per Chapter

**Feature Branch**: `007-urdu-translation-button`
**Created**: 2025-12-10
**Status**: Draft
**Input**: User description: "Urdu Translation Button per Chapter - Success criteria: Every chapter has "اردو میں دیکھیں" button at top, One click → entire chapter instantly translates to natural Urdu, Uses Groq + Llama-3.1-70B (not OpenAI) → fast + beautiful Urdu, Works after Personalize button too, Translation cached per chapter (no re-translate), Falls back to English if offline, Button toggles between English/Urdu"

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.
  
  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - View Chapter in Urdu (Priority: P1)

A student reading the Physical AI textbook wants to understand content in their native language. They see a "اردو میں دیکھیں" (View in Urdu) button at the top of each chapter and click it to translate the entire chapter to natural, fluent Urdu while maintaining technical accuracy.

**Why this priority**: This is the core value proposition. Without this, the feature provides no benefit to Urdu-speaking users. It enables non-English speakers to access the full curriculum.

**Independent Test**: Can be fully tested by opening any chapter, clicking the "اردو میں دیکھیں" button, and verifying that all visible content is translated to Urdu. Delivers immediate value to Urdu readers.

**Acceptance Scenarios**:

1. **Given** a user is on any chapter page, **When** they click the "اردو میں دیکھیں" button, **Then** the entire visible chapter content is translated to Urdu within 2 seconds
2. **Given** a chapter is displayed in Urdu, **When** the user reads the content, **Then** the translation uses natural, fluent Urdu with correct technical terminology (e.g., "روبوٹ" for robot, "ROS 2" remains as-is)
3. **Given** a user has personalized content, **When** they translate to Urdu, **Then** the personalized version is also translated (not the original)

---

### User Story 2 - Toggle Between Languages (Priority: P1)

A bilingual user wants to switch back and forth between English and Urdu to compare translations or clarify technical terms. The button text changes to reflect the current state.

**Why this priority**: Essential for usability. Users should be able to quickly switch languages without confusion about which language they're currently viewing.

**Independent Test**: Can be fully tested by clicking the language toggle button repeatedly and verifying the chapter alternates between English and Urdu. The button text accurately reflects the available action (show English or show Urdu).

**Acceptance Scenarios**:

1. **Given** chapter content is in English, **When** the user clicks the button, **Then** content switches to Urdu and button text changes to "Show in English"
2. **Given** chapter content is in Urdu, **When** the user clicks the button, **Then** content switches back to English and button text changes to "اردو میں دیکھیں"
3. **Given** a user toggles languages rapidly, **When** they click multiple times, **Then** the last click's state is what's displayed (no race conditions)

---

### User Story 3 - Cached Translations (Priority: P2)

A student reads Chapter 1 in Urdu, closes the browser, and returns the next day. When they open Chapter 1 again and click the Urdu button, the translation loads instantly without re-translating, saving API costs and providing fast user experience.

**Why this priority**: Significantly improves performance and reduces backend costs. Users expect snappy, instant translation on repeat visits. Without caching, every translation request hits the API.

**Independent Test**: Can be fully tested by translating a chapter, refreshing the page, translating again, and verifying the second translation loads in under 500ms (cached) versus 2+ seconds (fresh API call). Cache key is per chapter.

**Acceptance Scenarios**:

1. **Given** a chapter has been translated to Urdu before, **When** the user clicks the Urdu button again, **Then** the cached translation loads instantly (under 500ms)
2. **Given** translations are cached per chapter, **When** the user views Chapter 1 in Urdu then switches to Chapter 2, **Then** Chapter 2 is translated fresh and cached separately
3. **Given** a cache exists, **When** the user clears their browser cache, **Then** the next translation request fetches from API and re-caches

---

### User Story 4 - Offline Fallback (Priority: P2)

A user is offline or the translation service is unavailable. When they click the Urdu button, the system gracefully falls back to English and shows a notification that Urdu translation is temporarily unavailable.

**Why this priority**: Ensures feature degrades gracefully. Users should never see a broken experience or error page; they should fall back to English with a clear message.

**Independent Test**: Can be fully tested by simulating offline conditions (network disabled, API returns error) and verifying the button click either shows English with a fallback message or retains current content with an informational notification.

**Acceptance Scenarios**:

1. **Given** the translation API is unreachable, **When** the user clicks the Urdu button, **Then** the page displays English content with a message "Urdu translation is not available. Please try again later"
2. **Given** the user is offline, **When** no cached translation exists and they click Urdu, **Then** the system falls back to English gracefully without errors
3. **Given** offline mode, **When** a cached Urdu translation exists locally, **Then** the cached version is displayed (works offline for cached content)

### Edge Cases

- What happens when a chapter contains code blocks, formulas, or special formatting? (Technical content should remain in English, surrounding text translates to Urdu)
- What happens when a user has JavaScript disabled? (Button is disabled with tooltip explaining why)
- What happens when a chapter is extremely long (10,000+ words)? (Translation still completes within 2 seconds; chunked if necessary)
- What happens when the translation is requested multiple times simultaneously for the same chapter? (Deduplicate requests; return the same in-flight promise)
- What happens when the user's localStorage quota is exceeded? (Fall back to session memory; notify user that caching is temporarily unavailable)

## Requirements *(mandatory)*

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

- **FR-001**: Every chapter MUST display a translation button with label "اردو میں دیکھیں" at the top, positioned next to or below the Personalize button
- **FR-002**: Clicking the button MUST trigger translation of the entire visible chapter content to Urdu using the Groq API with Llama-3.1-70B model
- **FR-003**: Translation MUST complete within 2 seconds for typical chapter lengths (2,000-5,000 words)
- **FR-004**: The button MUST toggle between "اردو میں دیکھیں" (View in Urdu) and "Show in English" based on current chapter language
- **FR-005**: Translated content MUST preserve formatting, links, images, code blocks, and headings from the original
- **FR-006**: Code blocks and inline code MUST remain in English; only narrative text translates to Urdu
- **FR-007**: Technical terms (ROS 2, Isaac Sim, Gazebo, etc.) MUST appear in English within Urdu text unless a standard Urdu equivalent exists
- **FR-008**: Translated chapters MUST be cached per chapter ID to prevent re-translation on subsequent views
- **FR-009**: Cache MUST use browser localStorage with chapter ID as key (e.g., `urdu_translation_ch01`)
- **FR-010**: Cache entries MUST include timestamp and TTL; stale entries (>30 days) are re-fetched
- **FR-011**: If the translation API is unreachable or returns an error, the system MUST fall back to English and display an informational message
- **FR-012**: The button and translation feature MUST work correctly even after the Personalize button is used; personalized content is translated, not original
- **FR-013**: Translation state (English/Urdu) MUST persist across page navigation within the textbook (session state)
- **FR-014**: The translation feature MUST use Groq API (not OpenAI); no fallback to OpenAI

### Key Entities *(include if feature involves data)*

- **Chapter**: A document with ID (ch01, ch02, etc.), title, content (HTML/Markdown), and optional cached Urdu translation
- **Translation Cache**: Stored entry with chapter_id, original_content_hash, urdu_translation, timestamp, and expiry_time
- **Language State**: Session-level boolean indicating current display language (English or Urdu) for each chapter

## Success Criteria *(mandatory)*

<!--
  ACTION REQUIRED: Define measurable success criteria.
  These must be technology-agnostic and measurable.
-->

### Measurable Outcomes

- **SC-001**: Every chapter displays a functional translation button at the top of the page, visible and clickable
- **SC-002**: Users can translate any chapter to Urdu and see the complete translated content within 2 seconds on the first request
- **SC-003**: Subsequent translations of the same chapter load from cache and display within 500ms
- **SC-004**: 100% of chapter text (excluding code and special formatting) is translated to Urdu with natural, fluent phrasing (verified by Urdu-speaking reviewer)
- **SC-005**: The translation button correctly toggles between English and Urdu labels, and chapter content reflects the selected language
- **SC-006**: When the Urdu translation API is unavailable, the system gracefully falls back to English with a user-friendly notification (no errors or broken UI)
- **SC-007**: Translation feature works seamlessly after Personalize button is used; personalized content is translated accurately
- **SC-008**: All 13 chapters in the textbook have the translation button integrated and functional
- **SC-009**: API calls use Groq's Llama-3.1-70B model exclusively; no requests are sent to OpenAI
- **SC-010**: User satisfaction with Urdu translation quality is ≥4/5 stars (measured via post-translation feedback)

## Assumptions

- **A-001**: Groq API access is available and stable; pricing/rate limits are acceptable for production use
- **A-002**: Chapter content is well-structured HTML/Markdown without extremely complex layouts that would break during translation
- **A-003**: Browser supports localStorage for caching (or gracefully degrades without caching)
- **A-004**: Urdu text display is supported on all target browsers (modern Chrome, Safari, Firefox, Edge)
- **A-005**: Users have sufficient internet connectivity for initial translation API calls; offline fallback is for temporary disruptions
- **A-006**: 30-day cache TTL is acceptable; translations don't need frequent updates unless chapter content changes
- **A-007**: Code blocks and inline code are reliably identified in the source content and excluded from translation

## Out of Scope

- Translating the UI itself (buttons, navigation, sidebar labels) to Urdu—only chapter content translates
- Supporting languages other than Urdu (but architecture should be extensible for future languages)
- Real-time collaborative translation or user feedback on translation quality
- Automatic invalidation of cache when chapter content is updated (manual invalidation or TTL-based expiry only)
