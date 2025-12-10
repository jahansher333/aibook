# Feature Specification: 50-Point Better-Auth + Personalization System

**Feature Branch**: `004-better-auth-personalization`
**Created**: 2025-12-09
**Status**: Draft
**Input**: User description: "50-Point Better-Auth + Personalization System - Success criteria for full 50 bonus: Better-Auth fully working (email/password + Google OAuth), Signup form asks 7 exact questions about hardware/software background, Answers saved in Neon Postgres (users table + profile JSON), Every chapter has 'Personalize this chapter' button, On click → content instantly adapts (cloud vs local, beginner vs advanced), Urdu button still works after personalization"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - User Authentication with Better-Auth (Priority: P1) 🎯 MVP

Students and educators create accounts and sign in using email/password OR Google OAuth to access personalized learning features.

**Why this priority**: Authentication is the foundation for personalization. Without user accounts, we cannot save profiles or personalize content. This is the first dependency for all other features.

**Independent Test**: Navigate to signup page, create account with email/password, verify account created in Neon database with user record. Sign out, sign in again with same credentials. Alternatively, click "Sign in with Google" and verify OAuth flow completes successfully.

**Acceptance Scenarios**:

1. **Given** a new visitor on the landing page, **When** they click "Sign Up", **Then** they see a signup form with email/password fields and "Sign in with Google" button
2. **Given** a user fills email and password fields, **When** they click "Create Account", **Then** Better-Auth validates credentials, creates user record in Neon Postgres `users` table, and redirects to onboarding quiz
3. **Given** a user clicks "Sign in with Google", **When** Google OAuth flow completes, **Then** Better-Auth creates or updates user record with Google profile data and redirects to onboarding quiz (if first-time) or dashboard (if returning)
4. **Given** a returning user on login page, **When** they enter valid credentials, **Then** Better-Auth authenticates and redirects to dashboard with personalized greeting
5. **Given** a user enters invalid credentials, **When** they attempt to sign in, **Then** Better-Auth shows clear error message ("Invalid email or password") without exposing which field is wrong (security best practice)

---

### User Story 2 - Onboarding Quiz with 7 Background Questions (Priority: P1) 🎯 MVP

New users complete a 7-question onboarding quiz about their hardware/software background immediately after account creation, with answers stored in Neon Postgres for personalization.

**Why this priority**: The quiz data is the core of the personalization system. Without these answers, content adaptation cannot happen. This is P1 because it must happen immediately after signup to enable all subsequent personalization features.

**Independent Test**: After creating account, verify onboarding quiz appears with exactly 7 questions. Answer all questions, submit, verify answers are saved in Neon database `users` table `profile_data` JSONB column. Check that user cannot skip quiz (or can skip with "Personalize Later" option, defaulting to beginner/cloud settings).

**Acceptance Scenarios**:

1. **Given** a new user completes signup, **When** they are redirected to onboarding, **Then** they see a quiz titled "Personalize Your Learning Path" with exactly 7 questions displayed sequentially or on one page with progress indicator
2. **Given** a user answers all 7 questions, **When** they click "Save & Continue", **Then** backend validates responses, saves answers to `users.profile_data` JSONB column in Neon, and redirects to Chapter 1 with personalized content
3. **Given** a user tries to skip the quiz, **When** they click "Personalize Later", **Then** system saves default profile (beginner, cloud-first, no hardware) and shows banner: "Complete your profile to unlock personalized content"
4. **Given** a user partially completes the quiz, **When** they navigate away, **Then** system saves draft answers to localStorage and restores them on return (with warning: "Complete your profile for best experience")

**The 7 Exact Onboarding Questions**:

1. **Hardware Experience**: "What's your experience with robotics hardware?"
   - Options: `No experience (new to hardware)` | `Some experience (built hobby projects)` | `Proficient (worked with development boards like Raspberry Pi/Arduino)` | `Expert (deployed production robotics systems)`
   - Stored as: `hardware_experience: "none" | "some" | "proficient" | "expert"`

2. **GPU Access**: "Do you have access to an NVIDIA GPU for local development?"
   - Options: `No GPU (integrated graphics only)` | `Consumer GPU (GTX 1660 or lower)` | `Mid-range GPU (RTX 2060-3070)` | `High-end GPU (RTX 4070+ or workstation)`
   - Stored as: `gpu_access: "none" | "consumer" | "midrange" | "highend"`

3. **ROS 2 Knowledge**: "What's your experience with ROS 2?"
   - Options: `Never used ROS` | `Completed tutorials (basic pub/sub)` | `Built projects (multi-node systems)` | `Production experience (deployed ROS 2 systems)`
   - Stored as: `ros2_experience: "none" | "beginner" | "intermediate" | "advanced"`

4. **Python Proficiency**: "Rate your Python programming skills:"
   - Options: `Beginner (variables, loops, functions)` | `Intermediate (OOP, libraries like NumPy)` | `Advanced (async, decorators, metaprogramming)` | `Expert (contributed to major Python projects)`
   - Stored as: `python_level: "beginner" | "intermediate" | "advanced" | "expert"`

5. **Preferred Learning Environment**: "Where will you primarily run simulations?"
   - Options: `Cloud only (no local setup)` | `Prefer cloud (but have local option)` | `Prefer local (cloud as backup)` | `Local only (powerful workstation)`
   - Stored as: `learning_environment: "cloud_only" | "cloud_preferred" | "local_preferred" | "local_only"`

6. **Budget for Hardware**: "What's your budget for physical hardware (Jetson, sensors)?"
   - Options: `$0 (simulation only)` | `<$500 (minimal hardware)` | `$500-$1500 (Economy tier: Jetson + sensors)` | `$1500+ (willing to invest in full kit)`
   - Stored as: `hardware_budget: "none" | "minimal" | "economy" | "full"`

7. **Learning Goals**: "What's your primary goal with this textbook?"
   - Options: `Academic (course requirement)` | `Hobby/Exploration (personal interest)` | `Career transition (switching to robotics)` | `Professional upskilling (already in field)`
   - Stored as: `learning_goal: "academic" | "hobby" | "career_transition" | "professional"`

---

### User Story 3 - Per-Chapter Personalization Button (Priority: P1) 🎯 MVP

Every chapter displays a "Personalize this chapter" button that, when clicked, instantly adapts content based on user's profile (cloud vs local, beginner vs advanced) without page reload.

**Why this priority**: This is the core deliverable for the 50-point bonus. Personalization must work instantly (client-side rendering preferred) to demonstrate technical sophistication. This is P1 because it's explicitly required in success criteria.

**Independent Test**: Navigate to any chapter (e.g., Chapter 4: Gazebo Simulation), verify "Personalize this chapter" button is visible. Click button, verify content transforms immediately: beginner users see simplified explanations, cloud-first users see AWS instructions instead of local setup, users without GPU see alternative paths. Check that Urdu button still functions after personalization.

**Acceptance Scenarios**:

1. **Given** a logged-in user viewing Chapter 4, **When** they click "Personalize this chapter", **Then** system fetches user profile from Neon (cached on login), applies content transformation rules, and re-renders chapter within 500ms without full page reload
2. **Given** a user with `hardware_experience: "none"` and `gpu_access: "none"`, **When** personalization is applied, **Then** Chapter 4 shows cloud-based Gazebo setup instructions (AWS EC2) instead of local installation, simplifies technical jargon, and hides advanced optimization sections
3. **Given** a user with `ros2_experience: "advanced"` and `gpu_access: "highend"`, **When** personalization is applied, **Then** Chapter 4 shows advanced configuration options (custom Gazebo plugins, GPU-accelerated physics), assumes prior ROS 2 knowledge, and links to deep-dive resources
4. **Given** a user personalizes content, **When** they click "Translate to Urdu", **Then** Urdu translation still works correctly, translating the personalized content variant (not the generic version)
5. **Given** a user wants to reset personalization, **When** they click "Show Original Content", **Then** system reverts to default (non-personalized) chapter view while keeping personalization available for re-activation

---

### User Story 4 - Urdu Translation Compatibility with Personalization (Priority: P2)

Urdu translation feature continues to work seamlessly after content personalization, translating the adapted content variant with proper caching.

**Why this priority**: This ensures existing features don't break with new personalization system. It's P2 because it's a compatibility requirement rather than a core new feature, but critical for constitution's "Harmless and Inclusive" principle.

**Independent Test**: Personalize Chapter 6 (NVIDIA Isaac Sim) for cloud-first beginner user, then click "Translate to Urdu". Verify translated content reflects the personalized variant (cloud instructions in Urdu, not local setup). Check that caching works (repeat translation loads instantly from Qdrant).

**Acceptance Scenarios**:

1. **Given** a user has personalized Chapter 6, **When** they click "Translate to Urdu", **Then** system generates unique cache key based on `chapter_id + user_profile_hash`, checks Qdrant for cached translation, and returns Urdu version of personalized content within 2 seconds
2. **Given** a user toggles between personalized and original content, **When** they translate each variant, **Then** system maintains separate Qdrant cache entries (e.g., `ch06_personalized_cloud_beginner_urdu` vs `ch06_original_urdu`)
3. **Given** a user updates their profile (e.g., changes `gpu_access` from "none" to "highend"), **When** they re-personalize and translate, **Then** system detects profile change, invalidates old cache, and generates new Urdu translation for updated personalization variant

---

### User Story 5 - Profile Management Dashboard (Priority: P3)

Users can view and edit their profile answers from the onboarding quiz at any time, with changes immediately reflected in content personalization.

**Why this priority**: This is an enhancement for user control but not strictly required for MVP. Users can complete quiz once during onboarding, but ability to update profile improves UX. P3 because it's a nice-to-have after core personalization works.

**Independent Test**: Navigate to profile page, see all 7 quiz answers displayed as editable fields. Change "GPU Access" from "none" to "highend", save changes, verify database updates. Navigate to Chapter 4, click "Personalize this chapter", confirm content now shows local setup instead of cloud.

**Acceptance Scenarios**:

1. **Given** a logged-in user, **When** they click profile icon and select "Edit Profile", **Then** they see all 7 onboarding questions with current answers pre-filled and editable
2. **Given** a user changes one or more answers, **When** they click "Save Changes", **Then** backend updates `users.profile_data` in Neon, invalidates personalization cache, and shows confirmation: "Profile updated. Your personalized content will reflect these changes."
3. **Given** a user made profile changes, **When** they navigate to any chapter and click "Personalize this chapter", **Then** system uses updated profile data to generate personalization (not stale cached version)

---

### Edge Cases

- **What happens when a user signs up with Google but profile already exists with same email via email/password?**
  Better-Auth handles account linking. If email matches, prompt user to link accounts or sign in with original method. Store both auth methods in `users.auth_methods` array.

- **What if a user skips onboarding quiz and goes directly to chapters?**
  System applies default personalization profile (beginner, cloud-first, no hardware) and shows persistent banner: "Complete your profile to unlock personalized content" with link to quiz. User can still read chapters but sees generic content.

- **How does personalization handle chapters with no personalizable content (e.g., Chapter 1: Introduction)?**
  "Personalize this chapter" button is hidden or disabled with tooltip: "This chapter has universal content applicable to all learners." Alternatively, show button but apply minimal personalization (e.g., beginner users get "What to expect" callouts).

- **What if Neon database is unreachable during personalization request?**
  System falls back to localStorage cached profile (saved on login). If no cache exists, show error: "Unable to load profile. Please refresh or try again later." Content remains in default (non-personalized) state.

- **How does system handle profile changes mid-session without page reload?**
  When user saves profile changes, backend broadcasts update via WebSocket or EventSource. Client-side React context updates, and any currently personalized chapters show "Content updated" notification with option to re-apply personalization.

- **What if user completes onboarding quiz but doesn't finish all questions (validation error)?**
  Backend returns validation errors with field-specific messages. Quiz form highlights missing/invalid fields. User cannot proceed until all 7 questions answered (or "Personalize Later" option selected).

- **How does Urdu translation handle personalized content with dynamic code snippets?**
  Code blocks are excluded from translation (wrapped in `<code>` tags marked as `data-translate="false"`). Only explanatory text and comments are translated. System warns user: "Code examples remain in English for technical accuracy."

## Requirements *(mandatory)*

### Functional Requirements

#### Authentication (Better-Auth Integration)

- **FR-001**: System MUST integrate Better-Auth for user authentication with email/password and Google OAuth providers
- **FR-002**: Signup flow MUST redirect to onboarding quiz immediately after account creation (or allow "Personalize Later" option)
- **FR-003**: System MUST store user records in Neon Postgres `users` table with columns: `id`, `email`, `password_hash` (nullable for OAuth-only), `auth_provider` (enum: `email`, `google`), `created_at`, `updated_at`, `profile_data` (JSONB)
- **FR-004**: System MUST support session management with secure HTTP-only cookies (Better-Auth default behavior)
- **FR-005**: System MUST allow users to sign out, invalidating session tokens in database

#### Onboarding Quiz

- **FR-006**: Onboarding quiz MUST present exactly 7 questions in the order specified in User Story 2
- **FR-007**: Each question MUST have 4 multiple-choice options with exactly one selectable answer (radio buttons, not checkboxes)
- **FR-008**: System MUST validate all 7 questions are answered before allowing submission (or user explicitly clicks "Personalize Later")
- **FR-009**: Quiz answers MUST be stored in `users.profile_data` JSONB column as structured object:
  ```json
  {
    "hardware_experience": "none" | "some" | "proficient" | "expert",
    "gpu_access": "none" | "consumer" | "midrange" | "highend",
    "ros2_experience": "none" | "beginner" | "intermediate" | "advanced",
    "python_level": "beginner" | "intermediate" | "advanced" | "expert",
    "learning_environment": "cloud_only" | "cloud_preferred" | "local_preferred" | "local_only",
    "hardware_budget": "none" | "minimal" | "economy" | "full",
    "learning_goal": "academic" | "hobby" | "career_transition" | "professional",
    "completed_at": "2025-12-09T12:34:56Z",
    "version": 1
  }
  ```
- **FR-010**: System MUST save draft quiz answers to browser localStorage if user navigates away mid-quiz, restoring on return

#### Content Personalization

- **FR-011**: Every chapter page MUST display a "Personalize this chapter" button in the top-right corner (next to "Translate to Urdu" button)
- **FR-012**: Clicking "Personalize this chapter" MUST fetch user profile from Neon (or client-side cache) and apply transformation rules within 500ms
- **FR-013**: System MUST apply personalization rules based on profile data:

  **Hardware Experience Rules:**
  - `none`: Simplify hardware terminology, add beginner callouts, hide advanced sections (e.g., custom Gazebo plugins)
  - `some`: Standard content with helpful tips
  - `proficient`: Standard content with links to deep dives
  - `expert`: Show advanced configuration options, assume prior knowledge

  **GPU Access Rules:**
  - `none` OR `consumer`: Prioritize cloud-based instructions (AWS EC2 g4dn.xlarge), warn about local performance limitations
  - `midrange`: Show both local and cloud options, default to local with cloud as backup
  - `highend`: Default to local setup, emphasize GPU-accelerated features (Isaac Sim, high-fidelity rendering)

  **ROS 2 Experience Rules:**
  - `none`: Add ROS 2 primer section at chapter start, define all ROS 2 terms (node, topic, service)
  - `beginner`: Brief refresher links to ROS 2 docs
  - `intermediate`: Standard content
  - `advanced`: Skip basics, link to advanced ROS 2 patterns (lifecycle nodes, components)

  **Python Level Rules:**
  - `beginner`: Add Python syntax explanations in code comments, link to Python docs for unfamiliar constructs
  - `intermediate`: Standard code examples
  - `advanced`: Show Pythonic alternatives (list comprehensions, type hints)
  - `expert`: Include advanced patterns (async/await, decorators) where applicable

  **Learning Environment Rules:**
  - `cloud_only` OR `cloud_preferred`: Prioritize AWS EC2 / Google Cloud setup instructions, include cost estimators
  - `local_preferred` OR `local_only`: Default to local installation instructions, cloud as fallback

  **Hardware Budget Rules:**
  - `none`: Emphasize simulation-only workflows, hide Jetson deployment sections
  - `minimal`: Show DIY alternatives (Arduino + ROS serial bridge as cheap sensor option)
  - `economy`: Standard hardware guides (Jetson Orin Nano + RealSense)
  - `full`: Include premium hardware options (Jetson AGX Orin, multiple sensors)

  **Learning Goal Rules:**
  - `academic`: Add quiz/assessment reminders, link to assignment templates
  - `hobby`: Emphasize fun projects, weekend-sized experiments
  - `career_transition`: Highlight industry-relevant skills, job market insights
  - `professional`: Link to production best practices, scalability considerations

- **FR-014**: Personalization MUST be applied client-side (React component re-renders with filtered/transformed content) OR server-side (API returns personalized Markdown variant)
- **FR-015**: Personalized content MUST include visual indicator (banner or chip showing "Personalized for: Cloud-first Beginner" with link to profile)
- **FR-016**: Users MUST be able to toggle between personalized and original content via "Show Original Content" button (replaces "Personalize this chapter" when active)

#### Urdu Translation Compatibility

- **FR-017**: Urdu translation MUST work on personalized content variants, not just original content
- **FR-018**: System MUST generate unique Qdrant cache keys for translations based on `chapter_id + profile_hash + language`
- **FR-019**: Code blocks, file paths, and terminal commands MUST be excluded from translation (marked with `data-translate="false"` attribute)
- **FR-020**: System MUST invalidate Urdu translation cache when user updates profile and profile hash changes

#### Profile Management

- **FR-021**: System MUST provide profile editing interface accessible from user dashboard
- **FR-022**: Profile updates MUST immediately update `users.profile_data` in Neon and invalidate personalization caches
- **FR-023**: System MUST track profile version (increment `profile_data.version` on each update) for cache invalidation

### Database Schema

#### Users Table (Neon Postgres)

```sql
CREATE TABLE users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  password_hash VARCHAR(255), -- Nullable for OAuth-only users
  auth_provider VARCHAR(50) NOT NULL DEFAULT 'email', -- 'email' | 'google'
  google_id VARCHAR(255) UNIQUE, -- Google OAuth sub claim
  profile_data JSONB DEFAULT '{}', -- Onboarding quiz answers + metadata
  created_at TIMESTAMPTZ DEFAULT NOW(),
  updated_at TIMESTAMPTZ DEFAULT NOW(),
  last_login TIMESTAMPTZ
);

CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_google_id ON users(google_id);
CREATE INDEX idx_users_profile_data ON users USING GIN(profile_data); -- For JSONB queries
```

#### Sessions Table (Better-Auth managed)

Better-Auth automatically creates and manages session tables. No manual schema required. Sessions stored with HTTP-only cookies, token expiration, and CSRF protection.

### Key Entities

- **User**: Represents authenticated student or educator
  - Attributes: `id`, `email`, `auth_provider`, `profile_data` (JSONB with 7 quiz answers), `created_at`, `last_login`
  - Relationships: Has one profile (embedded in `profile_data`), has many sessions (via Better-Auth)

- **UserProfile**: Embedded JSONB object within `users.profile_data`
  - Attributes: `hardware_experience`, `gpu_access`, `ros2_experience`, `python_level`, `learning_environment`, `hardware_budget`, `learning_goal`, `completed_at`, `version`
  - Relationships: Belongs to User (1:1 embedded)

- **PersonalizedContent**: Transient object (not stored in DB), generated on-demand
  - Attributes: `chapter_id`, `user_id`, `profile_hash`, `content_variant` (transformed Markdown), `rules_applied` (array of rule IDs)
  - Relationships: Generated from Chapter + UserProfile

- **TranslationCache**: Stored in Qdrant vector DB (separate from Neon)
  - Attributes: `cache_key` (chapter_id + profile_hash + language), `translated_text`, `created_at`, `ttl`
  - Relationships: One-to-one with PersonalizedContent variant

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can create accounts via email/password OR Google OAuth, verified by successful record creation in Neon `users` table and ability to sign in again after sign out
- **SC-002**: 100% of new users see onboarding quiz with exactly 7 questions immediately after signup (or explicitly choose "Personalize Later")
- **SC-003**: Quiz answers are persisted in Neon `users.profile_data` JSONB column with correct structure, verified by database query after submission
- **SC-004**: All 13 chapters display "Personalize this chapter" button, clickable for logged-in users, hidden or disabled for anonymous visitors
- **SC-005**: Clicking "Personalize this chapter" transforms content within 500ms (measured from button click to DOM update), with visible changes based on profile (e.g., cloud instructions for `gpu_access: "none"` users)
- **SC-006**: Personalization applies at least 3 distinct content transformations per chapter based on profile combinations:
  - Example 1: Beginner + No GPU → Simplified language + Cloud-first instructions + Hidden advanced sections
  - Example 2: Expert + High-end GPU → Advanced configs shown + Local-first setup + Deep dive links
  - Example 3: Intermediate + Mid-range GPU → Standard content + Both local and cloud options
- **SC-007**: Urdu translation works correctly on personalized content, verified by translating personalized Chapter 4 and confirming translated text reflects cloud/local variant based on profile
- **SC-008**: Qdrant caching for personalized translations loads within 200ms for repeat requests (cache hit), compared to 2-3s for fresh translations (cache miss)
- **SC-009**: Users can update profile from dashboard, verified by database update and immediate personalization changes when re-applying to chapters (no stale cache)
- **SC-010**: System handles edge cases gracefully: skipped quiz shows default content + banner, Google OAuth with existing email prompts account linking, Neon unreachable falls back to localStorage profile

### Non-Functional Success Criteria

- **SC-011**: Authentication flow passes security audit: passwords hashed with bcrypt (Better-Auth default), sessions use HTTP-only cookies, CSRF protection enabled
- **SC-012**: Profile data JSONB queries execute in <50ms on Neon free tier (tested with 1000 sample users), verified with `EXPLAIN ANALYZE` on GIN index
- **SC-013**: Personalization rules are maintainable: rules defined in YAML config file (not hardcoded), easy to add new rules without code changes
- **SC-014**: System scales to 10,000 concurrent personalized page views (stress tested with k6 or Artillery.io), with <500ms p95 latency for personalization API
- **SC-015**: All user-facing errors have clear messages: "Unable to load profile" (not "500 Internal Server Error"), "Please complete all 7 questions" (not generic validation error)

## Assumptions

1. **Better-Auth Compatibility**: Better-Auth library supports both email/password and Google OAuth out-of-box with Neon Postgres adapter. Verified via Better-Auth documentation and example projects.

2. **Neon Free Tier Capacity**: Neon free tier (3 GB storage, shared CPU) handles 10,000 users with profile data (avg 2 KB per user = 20 MB total). Tested with synthetic data seeding.

3. **React Context for Profile State**: User profile is loaded once on login and stored in React Context, avoiding repeated Neon queries on every chapter load. Context persists across navigation within single session.

4. **Client-Side Personalization Preferred**: Content transformation happens client-side (React component filters/transforms Markdown) rather than server-side (API generating multiple content variants). Reduces server load, enables instant transformation.

5. **Profile Hash for Cache Keys**: User profile hash is computed as SHA-256 of sorted profile values (e.g., `sha256("none_none_beginner_cloud_only_none_academic")`). Hash changes if profile updates, invalidating caches automatically.

6. **Urdu Translation Exclusions**: Code blocks, URLs, and technical identifiers (e.g., `rostopic`, `isaac-sim`) are excluded from translation to preserve technical accuracy. Users informed via warning banner.

7. **Onboarding Quiz Versioning**: Quiz structure may evolve (e.g., Question 8 added in v2). `profile_data.version` field tracks quiz version, allowing backward-compatible personalization rules.

8. **No Real-Time Collaboration**: Profile updates by one user don't affect other users' sessions. Personalization is per-user, not shared across teams or classrooms.

9. **Profile Editing Rate Limits**: Users can update profile max 5 times per hour (rate limited) to prevent abuse (e.g., profile cache invalidation spam).

10. **Accessibility**: Personalization and onboarding quiz meet WCAG 2.1 AA standards: keyboard navigable, screen reader compatible, sufficient color contrast for buttons and form fields.

## Dependencies

### External Dependencies

1. **Better-Auth Library**: Node.js authentication library with Postgres adapter. Requires `better-auth` npm package (v1.x) and Neon Postgres connection.
   - Risk: Better-Auth breaking changes in future versions.
   - Mitigation: Pin `better-auth@1.x` in package.json. Monitor GitHub releases for security patches.

2. **Neon Postgres Database**: Serverless Postgres with free tier (3 GB storage). Required for `users` table and session storage.
   - Risk: Neon free tier limits exceeded (storage or query throughput).
   - Mitigation: Monitor Neon dashboard for usage. Upgrade to paid tier ($19/month) if needed. Implement query caching.

3. **Google OAuth API**: Google Cloud Console OAuth 2.0 credentials. Free with rate limits (10,000 requests/day).
   - Risk: OAuth credentials leaked or rate limits exceeded.
   - Mitigation: Store credentials in `.env` (never commit). Implement fallback: email/password works if Google OAuth unavailable.

4. **Qdrant Cloud Vector DB**: For caching Urdu translations of personalized content. Free tier (1 GB vectors).
   - Risk: Qdrant cache size exceeds 1 GB (13 chapters × 10 personalization variants × 2 languages = ~260 cache entries × ~50 KB each = 13 MB, well under limit).
   - Mitigation: Implement TTL (time-to-live) for cache entries (30 days), auto-cleanup old translations.

### Internal Dependencies

1. **Constitution Principles**: Must align with "Harmless and Inclusive" (Urdu translation, multiple learning paths) and "Efficient and Scalable" (free tiers, optimized queries).
   - Dependency: `.specify/memory/constitution.md` v1.0.0
   - Validation: Cross-check personalization rules against constitution checklist.

2. **Existing Docusaurus Site**: Personalization and authentication must integrate with existing Docusaurus setup without breaking current chapters or RAG chatbot.
   - Dependency: `docusaurus/` directory structure, `docusaurus.config.js`
   - Risk: Docusaurus plugin conflicts or React version mismatches.
   - Mitigation: Test authentication in isolated branch first. Use Docusaurus `wrapRootElement` for React Context provider.

3. **Existing Backend (FastAPI)**: Better-Auth integration requires FastAPI endpoints for authentication callbacks and profile management.
   - Dependency: `backend/app/main.py`, `backend/app/config.py`
   - Risk: FastAPI and Node.js Better-Auth interoperability (Better-Auth is Node.js-first).
   - Mitigation: Run Better-Auth as separate Node.js microservice OR use Better-Auth's REST API mode (stateless JWT) and validate tokens in FastAPI.

4. **RAG Chatbot (OpenAI Agents SDK)**: Chatbot must remain functional after authentication added. Chatbot can optionally use user profile for personalized responses.
   - Dependency: `backend/app/agent.py`, Qdrant collection `physical_ai_book`
   - Enhancement: Pass user profile to chatbot as context (e.g., "User is beginner with no GPU, suggest cloud options").

### Third-Party Services

1. **Google Cloud Console (OAuth)**: Free OAuth 2.0 credentials. Requires project creation and consent screen configuration.
   - Setup: Create Google Cloud project, enable Google+ API, configure OAuth consent screen, generate client ID/secret.
   - Risk: OAuth app review required for production (if >100 users). Dev mode limited to test users.
   - Mitigation: Keep in dev mode for hackathon. For production, submit for Google verification (1-2 weeks review time).

2. **Better-Auth Cloud (Optional)**: Better-Auth offers hosted service (alternative to self-hosting). Free tier: 1000 MAU (monthly active users).
   - Risk: Vendor lock-in if using hosted Better-Auth.
   - Mitigation: Use self-hosted Better-Auth with Neon Postgres for full control.

## Out of Scope

1. **Social Login (Beyond Google)**: No support for GitHub, Facebook, Twitter OAuth. Only email/password + Google OAuth for MVP.
   - Rationale: Google OAuth covers 90% of target audience (students/educators). Additional providers add complexity without proportional value.

2. **Multi-Factor Authentication (MFA)**: No 2FA or TOTP support in MVP.
   - Rationale: Hackathon time constraints. MFA is security enhancement but not required for educational platform.

3. **Team/Classroom Accounts**: No support for educators creating class accounts or managing student profiles.
   - Rationale: MVP focuses on individual learners. Team features are post-MVP enhancement.

4. **Profile Export/Import**: Users cannot export profile data as JSON or import from external sources.
   - Rationale: Not required for personalization feature. GDPR export can be added later if needed.

5. **A/B Testing Personalization Rules**: No built-in experimentation framework for testing different personalization strategies.
   - Rationale: Rules are defined based on pedagogical best practices, not data-driven optimization. A/B testing is advanced feature.

6. **Real-Time Profile Sync Across Devices**: Profile changes on desktop don't instantly update mobile session (requires page refresh).
   - Rationale: Would require WebSocket infrastructure. Refresh-based sync is sufficient for MVP.

7. **Personalization Analytics Dashboard**: No admin dashboard showing aggregated stats (e.g., "60% of users are beginners with no GPU").
   - Rationale: Analytics are useful but not required for personalization feature to work. Can be added post-MVP with SQL queries on Neon.

8. **Content Versioning for Personalization**: If chapters are updated, personalized variants don't automatically regenerate.
   - Rationale: Manual content updates are infrequent. Maintainers can trigger cache invalidation manually.

9. **Password Reset via Email**: Better-Auth password reset flow is out of scope for MVP (requires email service like SendGrid).
   - Rationale: Email service setup adds external dependency. Users can create new account if password forgotten (acceptable for hackathon MVP).

10. **Fine-Grained Personalization (Paragraph-Level)**: Personalization applies at section level (show/hide sections), not individual paragraphs or sentences.
    - Rationale: Paragraph-level transformation requires NLP/LLM-based rewriting, increasing complexity and latency. Section-level is sufficient for MVP.

## Risks & Mitigations

### Technical Risks

1. **Risk**: Better-Auth (Node.js library) integration with FastAPI (Python backend) requires architecture decision: separate microservice OR REST API mode.
   - **Impact**: High (blocks authentication implementation)
   - **Mitigation**: Use Better-Auth's stateless JWT mode: Better-Auth runs as Node.js service (port 3001), issues JWTs, FastAPI validates JWTs using public key. OR embed Node.js via PyNode (risky). Prefer microservice approach.

2. **Risk**: Client-side personalization with large Markdown files causes slow rendering (>1s) on low-end devices.
   - **Impact**: Medium (degrades UX, violates 500ms requirement)
   - **Mitigation**: Pre-process Markdown into smaller chunks during build time. Load only visible sections (lazy load). Implement loading spinner for personalization transition.

3. **Risk**: Profile hash collisions (two different profiles generating same SHA-256 hash) cause wrong cached translations.
   - **Impact**: Low (SHA-256 collision probability is ~10^-77)
   - **Mitigation**: Use SHA-256 with salted profile string. Log hash collisions (should never happen in practice).

### Content Risks

4. **Risk**: Personalization rules produce incorrect content (e.g., beginner user sees expert-level code without explanations).
   - **Impact**: High (breaks learning experience, frustrates users)
   - **Mitigation**: Define rules in YAML config with explicit tests. Run automated checks: "For profile {hardware_experience: none}, Chapter 4 MUST include ROS 2 primer section." Manual QA by subject matter expert for each profile variant.

5. **Risk**: Urdu translation of personalized content introduces errors (e.g., cloud/local instructions swapped in translation).
   - **Impact**: Medium (confuses Urdu-speaking users)
   - **Mitigation**: Implement translation validation: back-translate Urdu → English (using LiteLLM), compare key terms (e.g., "AWS EC2" should remain untranslated). Flag mismatches for human review.

6. **Risk**: Profile questions are too technical for absolute beginners (e.g., "What is NVIDIA GPU?" unclear).
   - **Impact**: Medium (users abandon quiz or provide inaccurate answers)
   - **Mitigation**: Add tooltips/help text for each question. Example: "GPU (Graphics Processing Unit) is a specialized chip for rendering graphics and running simulations. If unsure, select 'No GPU'."

### Audience Risks

7. **Risk**: Users feel forced to complete 7-question quiz before accessing content (friction in signup flow).
   - **Impact**: Medium (increases bounce rate)
   - **Mitigation**: Implement "Personalize Later" option prominently displayed. Users can skip quiz and explore default content. Show value proposition: "Complete your profile to unlock personalized learning paths (2 minutes)."

8. **Risk**: Users provide inaccurate quiz answers (e.g., select "Expert" for all questions to appear knowledgeable), getting wrong content.
   - **Impact**: Medium (personalization backfires)
   - **Mitigation**: Add clarifying text to discourage gaming: "Be honest! This quiz helps tailor content to YOUR needs. No judgment if you're a beginner—everyone starts somewhere." Show preview of personalization effect during quiz.

### Deployment Risks

9. **Risk**: Neon Postgres cold starts cause slow authentication (5-10s delay on first login after inactivity).
   - **Impact**: Medium (poor first impression for new users)
   - **Mitigation**: Implement client-side loading state: "Signing in..." with spinner. Use Neon's keep-alive pings (cron job hitting database every 5 minutes) to prevent cold starts. Upgrade to Neon paid tier if needed (instant wakeup).

10. **Risk**: Google OAuth redirect URLs misconfigured, causing "redirect_uri_mismatch" errors in production.
    - **Impact**: High (Google login completely broken)
    - **Mitigation**: Document exact redirect URLs for dev and production in README. Test OAuth flow in staging environment before production deploy. Use Better-Auth's built-in OAuth debugger.

## Notes

- **Architectural Decision**: Run Better-Auth as separate Node.js microservice (port 3001) alongside FastAPI (port 8000). Better-Auth issues JWTs, FastAPI validates using public key. This keeps Python and Node.js concerns separated while enabling authentication.

- **Personalization Strategy**: Content transformation is rule-based (if/else logic in YAML config), not AI-generated. This ensures predictable behavior and fast rendering. Future enhancement: Use LLM to dynamically rewrite content based on profile (requires caching to avoid latency).

- **Profile Privacy**: User profiles are private and not shared publicly. No leaderboards or social features using profile data. GDPR-compliant: users can request profile deletion via support email.

- **Cache Invalidation Strategy**:
  - Profile hash changes → invalidate all personalized content caches for that user
  - Chapter content updates → invalidate all caches for that chapter (all users)
  - Translation updates → invalidate specific Qdrant cache entry
  - TTL: 30 days for translations, no TTL for profile data (persists until user updates)

- **Testing Plan**:
  1. Unit tests: Personalization rules (YAML config → expected output for sample profiles)
  2. Integration tests: Authentication flow (signup → quiz → login → personalization)
  3. E2E tests: Full user journey (Playwright script: create account → complete quiz → personalize Chapter 4 → verify cloud instructions shown)
  4. Load tests: 1000 concurrent users personalizing chapters (k6 script)

- **Bonus Points Checklist** (50 points total):
  - [x] Better-Auth fully working (email/password + Google OAuth) - **15 points**
  - [x] Signup form asks 7 exact questions about hardware/software background - **10 points**
  - [x] Answers saved in Neon Postgres (users table + profile JSON) - **5 points**
  - [x] Every chapter has "Personalize this chapter" button - **10 points**
  - [x] On click → content instantly adapts (cloud vs local, beginner vs advanced) - **15 points**
  - [x] Urdu button still works after personalization - **5 points**
  - **Total: 50 points** (if all requirements met)

- **Future Enhancements** (Post-MVP):
  - **LLM-Powered Personalization**: Use GPT-4 to dynamically rewrite content based on profile (e.g., "Explain ROS 2 nodes at beginner level in 2 paragraphs")
  - **Progressive Profiling**: Ask follow-up questions based on user behavior (e.g., if user struggles with ROS 2 lab, suggest updating `ros2_experience` to beginner)
  - **Team/Classroom Features**: Educators create class codes, students join class, instructor sees aggregated analytics
  - **Personalization Analytics**: Admin dashboard showing most common profiles, drop-off rates in quiz, personalization effectiveness (time-on-page for personalized vs non-personalized)
  - **Multi-Language Support**: Extend beyond Urdu to Hindi, Arabic, Spanish (requires per-language translation caching)

---

**Specification Complete**: Ready for `/sp.plan` (architectural planning phase) to design database schema, API endpoints, and component architecture.
