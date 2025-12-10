# Feature Specification: Better-Auth with FastAPI Backend Integration

**Feature Branch**: `005-better-auth-fastapi`
**Created**: 2025-12-09
**Status**: Draft
**Input**: User description: "Better-Auth with FastAPI Backend (50 Bonus) - Success criteria: Better-Auth React + TypeScript running in Docusaurus, Custom adapter points to FastAPI endpoints, FastAPI handles /api/auth/* routes (signin, signup, session), 7 background questions asked during signup, User profile + answers saved in Neon Postgres, Personalization works exactly as before"

## User Scenarios & Testing

### User Story 1 - User Registration with Background Questions (Priority: P1 🎯 MVP)

New users can create an account through a multi-step signup flow that collects authentication credentials (email/password) and captures 7 background questions about their hardware setup, software experience, and learning goals. All data is securely stored in the database and immediately available for content personalization.

**Why this priority**: Core authentication functionality that enables all other personalized features. Without user accounts and profiles, personalization cannot function. This story delivers immediate value by creating the foundation for the entire bonus feature set.

**Independent Test**: Navigate to signup page, complete email/password fields, answer all 7 background questions, submit form. Verify user account exists in database with profile data. Log out and log in again with same credentials to confirm authentication works end-to-end.

**Acceptance Scenarios**:

1. **Given** a new visitor on the signup page, **When** they enter valid email and password, **Then** they proceed to the 7-question onboarding quiz
2. **Given** a user completing the onboarding quiz, **When** they answer all 7 questions (RTX GPU, Jetson board, Ubuntu experience, ROS2 knowledge, simulation preference, learning goal, preferred language), **Then** their profile is saved and they are redirected to the first chapter
3. **Given** a user who completed signup, **When** they log out and log in again, **Then** their profile data is retrieved and personalization is immediately available
4. **Given** a user enters an email that already exists, **When** they attempt signup, **Then** they see a clear error message: "Email already registered. Please log in."

---

### User Story 2 - User Authentication and Session Management (Priority: P1 🎯 MVP)

Registered users can securely log into their accounts using email/password credentials. The system maintains their session across page navigations within the Docusaurus site and provides a seamless authentication experience with automatic session restoration on page reload.

**Why this priority**: Essential for returning users to access their personalized content. Authentication must be rock-solid to ensure users don't lose access to their profiles and personalization settings. This is a blocker for all subsequent personalization features.

**Independent Test**: After creating an account, log out completely. Return to login page, enter credentials, verify successful login and redirect to last visited chapter. Close browser, reopen site, verify session persists without requiring re-login (within session timeout period).

**Acceptance Scenarios**:

1. **Given** a registered user on the login page, **When** they enter correct email/password, **Then** they are logged in and redirected to their last visited chapter (or Chapter 1 if first login)
2. **Given** a logged-in user navigating between chapters, **When** they reload any page, **Then** their session persists and they remain authenticated
3. **Given** a user enters incorrect credentials, **When** they attempt login, **Then** they see error: "Invalid email or password"
4. **Given** a logged-in user, **When** they click logout, **Then** their session is terminated and they are redirected to the login page

---

### User Story 3 - Content Personalization Based on Profile (Priority: P1 🎯 MVP)

Users with completed profiles see personalized content recommendations and guidance throughout the textbook based on their hardware availability (RTX GPU, Jetson board), software experience (Ubuntu, ROS2), and learning preferences (cloud vs local simulation, learning goals). The personalization adapts instructions, hides/shows sections, and provides relevant callouts to match each user's specific setup and experience level.

**Why this priority**: This is the core value proposition of the authentication system—delivering tailored educational content. Without personalization, the profile data has no purpose. This story demonstrates the "50 bonus points" value by showing how FastAPI-backed authentication enables sophisticated content adaptation.

**Independent Test**: Log in as a user with no RTX GPU and cloud preference. Navigate to Chapter 4 (Gazebo). Click "Personalize this chapter". Verify cloud setup instructions are shown, local GPU instructions are hidden, and callout recommends AWS g5.2xlarge. Repeat with a different profile (RTX 4090 + local preference) and verify GPU instructions are shown, cloud alternatives are hidden.

**Acceptance Scenarios**:

1. **Given** a user with no RTX GPU, **When** they personalize Chapter 4, **Then** they see cloud setup instructions (AWS g5.2xlarge) and local GPU sections are hidden
2. **Given** a user with Jetson Nano, **When** they personalize deployment chapters, **Then** they see Jetson-specific deployment instructions with "Deploy to Jetson: flash weights locally" callouts
3. **Given** a beginner-level user, **When** they personalize any chapter, **Then** advanced sections (RL algorithms, custom URDF) are hidden and beginner callouts are added
4. **Given** a user with Urdu language preference, **When** they navigate any chapter, **Then** the translation button is active by default and translations use their profile-specific cache key

---

### User Story 4 - Profile Management and Updates (Priority: P2)

Users can view and modify their profile settings (7 background questions) from a dedicated profile dashboard. Changes to the profile immediately invalidate personalization cache and trigger re-personalization on the next chapter visit, ensuring content always reflects the user's current setup and preferences.

**Why this priority**: Users' hardware and experience evolve over time (e.g., purchasing a new GPU, upgrading from Jetson Nano to AGX). This story enables users to keep their personalization accurate without creating a new account, improving long-term engagement.

**Independent Test**: Log in, navigate to /profile page. Change "GPU Access" from "None" to "RTX 4090". Change "Simulation Preference" from "Cloud" to "Local". Save changes. Navigate to Chapter 4, personalize content. Verify local GPU instructions now appear (not cloud alternatives). Confirm profile hash changed in browser console/network tab.

**Acceptance Scenarios**:

1. **Given** a logged-in user on the profile page, **When** they view their profile, **Then** all 7 background answers are displayed as editable form fields
2. **Given** a user modifying their profile, **When** they change any answers and save, **Then** their profile version increments and profile hash is recomputed
3. **Given** a user who updated their profile, **When** they personalize a chapter, **Then** the new profile settings are applied (old personalization is discarded)
4. **Given** a user clicks "Save Changes" without modifying any fields, **When** they submit, **Then** no update occurs and they see message: "No changes detected"

---

### User Story 5 - Google OAuth Authentication (Priority: P3)

Users can sign up or log in using their Google account as an alternative to email/password authentication. Google OAuth users still complete the 7-question onboarding quiz after first authentication, and their profile data is stored identically to email/password users.

**Why this priority**: Nice-to-have convenience feature that reduces signup friction for users who prefer social login. Not critical for core functionality but improves user experience and reduces password management burden.

**Independent Test**: Click "Sign in with Google" on signup page. Complete Google OAuth consent flow in popup. After redirect, verify onboarding quiz appears. Complete quiz, verify profile saved. Log out, click "Sign in with Google" again, verify instant login without quiz (profile already exists).

**Acceptance Scenarios**:

1. **Given** a new user clicks "Sign in with Google", **When** they complete Google OAuth consent, **Then** they are redirected to onboarding quiz
2. **Given** a returning Google OAuth user, **When** they click "Sign in with Google", **Then** they bypass onboarding and go directly to Chapter 1
3. **Given** a user with existing email/password account, **When** they attempt Google OAuth with same email, **Then** accounts are not merged (treated as separate—user must use original login method)

---

### Edge Cases

- **What happens when a user abandons the quiz mid-way during signup?** Quiz answers are saved to browser localStorage as draft. If user returns to /onboarding page, draft answers are restored. User can also click "Personalize Later" to skip quiz and proceed with default profile (no GPU, beginner level, cloud-only, English).

- **What happens when database connection fails during login?** User sees error message: "Unable to connect to server. Please try again." Session is not created. After 3 consecutive failures, frontend displays "Service temporarily unavailable" with estimated recovery time (if available from health check endpoint).

- **What happens when a user's session expires mid-browsing?** Next API call to backend returns 401 Unauthorized. Frontend detects this, clears local session state, shows toast notification: "Session expired. Please log in again", and redirects to /login with `?redirect=/current/path` query parameter to return after re-authentication.

- **What happens when user enters invalid data in onboarding quiz?** Client-side validation prevents submission (e.g., RTX model field requires text if "Yes, I have RTX" is selected). Backend validation returns 400 Bad Request with specific field errors if client validation is bypassed. User sees inline error messages below each invalid field.

- **What happens when two users have identical profiles?** Profile hash will be identical, allowing them to share the same personalized translation cache entries in Qdrant. This is intended behavior—reduces storage and improves cache hit rate for common profiles.

- **What happens when user changes profile while viewing a personalized chapter?** Current page remains personalized with old profile until user manually clicks "Personalize this chapter" again or navigates to a different chapter. This prevents jarring mid-page content shifts.

- **What happens when authentication service is down but Docusaurus is up?** Unauthenticated users can still browse all content (personalization features are simply hidden). Authenticated users whose session is still valid can continue browsing with personalization, but cannot update profile or log out/log in until service recovers.

## Requirements

### Functional Requirements

- **FR-001**: System MUST allow users to create accounts using email and password with minimum 8 characters
- **FR-002**: System MUST validate email format (RFC 5322 compliant) before accepting registration
- **FR-003**: System MUST present 7 background questions immediately after successful signup: (1) RTX GPU ownership + model, (2) Jetson board type, (3) Ubuntu experience level, (4) ROS2 knowledge, (5) Simulation preference (cloud/local), (6) Learning goal, (7) Preferred language
- **FR-004**: System MUST save all 7 quiz answers plus metadata (completion timestamp, version number) to user's profile in database
- **FR-005**: System MUST allow users to skip onboarding quiz and assign default profile (no GPU, beginner, cloud-only, English)
- **FR-006**: System MUST allow registered users to log in with email/password credentials
- **FR-007**: System MUST maintain user sessions across page navigations within Docusaurus site
- **FR-008**: System MUST restore user session automatically on page reload (if session not expired)
- **FR-009**: System MUST terminate user session when logout action is triggered
- **FR-010**: System MUST return clear error messages for authentication failures (invalid credentials, email already exists, network errors)
- **FR-011**: System MUST compute deterministic profile hash (SHA-256) from sorted profile JSON for cache key generation
- **FR-012**: System MUST provide profile data to personalization engine whenever user clicks "Personalize this chapter"
- **FR-013**: System MUST hide/show content sections based on profile attributes (GPU access, Jetson ownership, experience level)
- **FR-014**: System MUST inject contextual callouts based on profile (cloud setup guides for non-GPU users, Jetson deployment for Jetson owners)
- **FR-015**: System MUST allow users to view their current profile settings on dedicated profile page
- **FR-016**: System MUST allow users to update any of the 7 profile answers after account creation
- **FR-017**: System MUST increment profile version number and recompute profile hash when profile is updated
- **FR-018**: System MUST invalidate personalization cache (force re-personalization) when profile hash changes
- **FR-019**: System MUST support Google OAuth as alternative authentication method
- **FR-020**: System MUST collect onboarding quiz answers from Google OAuth users on first login
- **FR-021**: System MUST bypass onboarding quiz for returning Google OAuth users (profile already exists)
- **FR-022**: System MUST store draft quiz answers in browser localStorage to support quiz resumption
- **FR-023**: System MUST use profile-specific cache keys for Urdu translations (format: ch{id}_{profile_hash}_{language})

### Key Entities

- **User Account**: Represents an authenticated user. Contains unique identifier, email address, authentication provider (email or Google), hashed password (if email auth), Google account ID (if OAuth), account creation timestamp, last login timestamp.

- **User Profile**: Represents user's background information from 7-question quiz. Contains RTX GPU ownership flag + model string, Jetson board type (none/nano/nx/agx), Ubuntu experience level (beginner/intermediate/expert), ROS2 knowledge (none/basic/advanced), simulation preference (cloud/local/both), learning goal (learn_basics/build_humanoid/research), preferred language (english/urdu), completion timestamp, version number. Stored as structured data (JSONB in Postgres) within User Account record for fast queries.

- **User Session**: Represents an active authenticated session. Contains session token (JWT or secure random string), user identifier reference, session expiration timestamp, creation timestamp. Used to maintain logged-in state across requests.

- **Profile Hash**: Computed value (SHA-256 of sorted profile JSON) used as cache key for personalized content and translations. Not stored directly—computed on-demand from User Profile. Ensures deterministic caching: identical profiles produce identical hashes.

- **Personalized Content**: Represents chapter content adapted for specific user profile. Contains original chapter markdown, transformed markdown (with sections hidden/shown), list of applied rules, profile hash used for transformation, personalization metadata (is_personalized flag, sections_hidden list, sections_shown list). Generated dynamically client-side, not stored in database.

## Success Criteria

### Measurable Outcomes

- **SC-001**: Users can complete account creation (signup + 7-question quiz) in under 3 minutes (p90)
- **SC-002**: Users can log in and see personalized content within 5 seconds of entering credentials (p90 latency from login submit to first personalized chapter view)
- **SC-003**: System maintains 99.5% authentication success rate (excluding user error like wrong password)
- **SC-004**: Profile updates take effect within 500ms (time from "Save Changes" click to profile hash recomputation and cache invalidation)
- **SC-005**: Session persistence works correctly for 95% of users (no unexpected logouts within 7-day session timeout period)
- **SC-006**: Personalization correctly adapts content for 100% of profile combinations tested (covering at least 20 distinct profile variants: no GPU + beginner, RTX 4090 + expert, Jetson NX + intermediate, etc.)
- **SC-007**: Users who skip onboarding quiz can still access all content with default personalization (no blocking errors)
- **SC-008**: Google OAuth authentication has 95% success rate (excluding user cancel/deny consent)
- **SC-009**: System handles 1,000 concurrent authenticated users without performance degradation (login latency <2s, personalization latency <1s)
- **SC-010**: Zero authentication-related security vulnerabilities discovered during security review (SQL injection, XSS, CSRF, session fixation)
- **SC-011**: Profile hash collision rate is below 0.01% for 10,000 unique profiles (validates SHA-256 uniqueness)
- **SC-012**: Translation cache hit rate for personalized content exceeds 60% after 1 week of usage (indicates effective profile-based caching)

### Acceptance Criteria for 50 Bonus Points

To qualify for the "50 bonus points" mentioned in the feature description, the implementation must demonstrate:

1. **Better-Auth React client successfully communicates with FastAPI backend** (not Node.js auth service): All authentication API calls originate from Better-Auth React components in Docusaurus and target FastAPI endpoints at `http://localhost:8000/api/auth/*` (or production equivalent). API contract is documented and judges can verify network requests in browser DevTools.

2. **FastAPI handles all authentication endpoints**: `/api/auth/signup`, `/api/auth/signin`, `/api/auth/session`, `/api/auth/me` routes are implemented in FastAPI (Python), not delegated to external Node.js service. JWT generation, password hashing (bcrypt), session validation all occur within FastAPI codebase.

3. **7 background questions integrated into signup flow**: Not a separate post-signup form—questions appear as part of the signup process. User cannot access personalized content until quiz is completed (or explicitly skipped). Quiz data is immediately saved to Neon Postgres `users.profile_data` JSONB column upon submission.

4. **Personalization works exactly as before**: Users with completed profiles see identical personalization behavior to previous implementation (content adaptation, Urdu translation compatibility, profile hash caching). No regression in personalization quality or performance.

5. **Demonstrates Python backend viability for authentication**: Judges can verify that a Python/FastAPI backend is a valid and functional choice for implementing authentication features in the Physical AI textbook project, not requiring Node.js/Better-Auth library on backend.

## Assumptions

- **Session timeout**: Default session duration is 7 days (168 hours). This is a reasonable default for educational content where users may not visit daily but should remain logged in across multiple learning sessions per week.

- **Password hashing algorithm**: Bcrypt with cost factor 12 will be used for password storage. This is industry standard for web applications and provides strong security without excessive computation time.

- **Email verification**: Email verification is NOT required for MVP (P1 stories). Users can sign up and immediately access content without confirming email. This simplifies initial implementation and reduces friction for educational content (low risk of abuse compared to e-commerce/financial apps). Can be added in P3 as enhancement.

- **HTTPS requirement**: Production deployment will use HTTPS (TLS 1.3) for all authentication endpoints. Development environment may use HTTP on localhost for convenience.

- **Database availability**: Neon Postgres is assumed to be available with 99.9% uptime (based on Neon's documented SLA for paid tiers). Free tier has lower guarantees but is acceptable for MVP testing.

- **Concurrent user limit**: MVP targets 1,000 concurrent users as success criterion (SC-009). This is sufficient for initial launch and organic growth. System can scale beyond this with standard FastAPI/ASGI deployment patterns (Gunicorn + workers, horizontal scaling).

- **Google OAuth consent screen**: Google OAuth is configured with appropriate consent screen details (app name, logo, privacy policy URL, terms of service URL) before public launch. Development environment uses test credentials with unverified consent screen.

- **CORS configuration**: Frontend (Docusaurus) and backend (FastAPI) may be deployed on different domains/ports. CORS (Cross-Origin Resource Sharing) middleware is configured on FastAPI to allow credentials and specific origin headers.

- **Rate limiting**: Login and signup endpoints are protected with rate limiting (e.g., max 5 failed login attempts per IP per 15 minutes, max 3 signup attempts per IP per hour). This prevents brute-force attacks and abuse without requiring complex infrastructure.

- **Data retention policy**: User accounts and profile data are retained indefinitely unless user explicitly requests deletion. Inactive accounts (no login for 2+ years) may receive notification about account deletion policy in future, but MVP does not implement automatic purging.

- **Browser compatibility**: Target modern browsers (Chrome 90+, Firefox 88+, Safari 14+, Edge 90+). No support for Internet Explorer. LocalStorage and Fetch API are assumed available.

- **Profile version numbering**: Starts at 1 for newly completed profiles, increments by 1 on each update. Used for analytics and debugging (can track profile evolution over time). Not used for cache invalidation—profile hash handles that.

## Dependencies

- **Neon Postgres database**: Must be provisioned and accessible before authentication can function. Database URL required in environment configuration. Shared dependency with existing RAG chatbot feature.

- **Existing personalization logic**: This feature builds on top of existing content personalization system (Chapter 4 examples, Urdu translation, profile hash caching). Authentication simply provides the profile data source—personalization engine remains unchanged.

- **Docusaurus site infrastructure**: Frontend components (SignupForm, LoginForm, ProfileDashboard) are integrated into existing Docusaurus React app. Assumes Docusaurus build pipeline supports custom React components and contexts.

- **Better-Auth React library**: npm package `@better-auth/react` must be installed in Docusaurus project. Provides React hooks (`useAuth`, `useSession`) and components that will be adapted to call FastAPI endpoints instead of Better-Auth Node.js library.

- **FastAPI backend**: Existing Python FastAPI application must be extended with new authentication routes. Assumes FastAPI is already running and accessible (either localhost:8000 in development or production URL).

- **Cryptography libraries**: Python `cryptography` and `bcrypt` packages required for password hashing, JWT token generation, and profile hash computation on FastAPI backend.

## Out of Scope

- **Multi-factor authentication (MFA)**: Two-factor authentication (TOTP, SMS codes, email verification codes) is not included in MVP. Could be added as future enhancement if security requirements escalate.

- **Password reset via email**: "Forgot password" flow requiring email verification link is excluded from MVP. Users who forget password must contact support or create new account. Could be added in P2/P3 with email service integration.

- **Account deletion**: User-initiated account deletion (GDPR "right to be forgotten") is not implemented in MVP. Users must contact support to request account removal. Should be added before EU launch.

- **Admin dashboard**: No admin interface for managing users, viewing profiles, or moderating accounts in MVP. All user management happens via direct database access or custom scripts.

- **OAuth providers beyond Google**: Only Google OAuth is supported. GitHub, Microsoft, Apple SSO are out of scope. Could be added incrementally as additional social providers if user demand exists.

- **Email notifications**: No transactional emails (welcome email, password changed notification, new login alert) in MVP. Could be added with SendGrid/AWS SES integration if engagement metrics show value.

- **Activity logging and audit trail**: User actions (login attempts, profile updates, failed authentication) are not logged to persistent audit database. Basic application logging captures events but is not structured for compliance/security analysis. Full audit trail should be added before enterprise adoption.

- **Account merging**: Users who sign up with email/password and later attempt Google OAuth with same email will have two separate accounts. No automatic or manual account merging capability in MVP.

- **Progressive profile completion**: Users cannot partially complete quiz and finish later—quiz must be completed in single session or skipped entirely. "Save & Continue Later" within quiz is out of scope.

- **A/B testing of onboarding questions**: The 7 background questions are fixed. No experimentation framework to test different question sets, orderings, or phrasing for conversion optimization.

## Notes

**FastAPI Backend Integration Strategy**: This feature demonstrates that Better-Auth's React client library can be adapted to work with any backend API, not just Better-Auth's Node.js library. The key insight is that Better-Auth React provides UI components and client-side session management, while the actual authentication logic (password hashing, JWT issuance, database queries) happens on the backend—which can be Python/FastAPI.

**Custom Adapter Approach**: We create a custom adapter layer that intercepts Better-Auth React's API calls and translates them to FastAPI endpoint format. For example, when Better-Auth React calls `auth.signUp({email, password})`, our adapter transforms this to `POST http://localhost:8000/api/auth/signup` with appropriate request body format. FastAPI handles the request, returns JWT token in expected format, and Better-Auth React stores the session client-side.

**Why This Matters for 50 Bonus Points**: The contest judges are evaluating whether Python backends are viable for building modern web apps with authentication. This feature proves that FastAPI can handle authentication just as well as Node.js/Express, while maintaining compatibility with popular React authentication libraries. It showcases Python's growing maturity in full-stack web development beyond just data science and ML APIs.

**Profile Hash for Personalization**: The SHA-256 profile hash is critical for efficient personalization caching. Without it, every unique user profile would require separate Qdrant translation entries, leading to storage bloat and cache misses. With profile hashing, users with identical profiles (e.g., all "beginner + no GPU + cloud-only + English" users) share the same cache entries, dramatically improving cache hit rates and reducing Qdrant storage costs.

**Separation from feature 004-better-auth-personalization**: This feature (005) is an architectural variation of feature 004. While 004 used a separate Node.js authentication microservice (as originally planned), feature 005 consolidates authentication into the existing FastAPI backend. Both features deliver identical user-facing functionality—the difference is purely in backend architecture. Feature 005 is the preferred implementation path if Python/FastAPI ecosystem is the strategic choice for the project.

---

**Specification Status**: ✅ Complete - Ready for validation and planning
**Next Steps**: Create requirements checklist, validate specification quality, then proceed to `/sp.plan`
