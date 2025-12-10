# Task Breakdown: Better-Auth + Personalization System

**Feature Branch**: `004-better-auth-personalization`
**Created**: 2025-12-09
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

## Overview

This task breakdown implements a comprehensive authentication and personalization system organized by user stories to enable independent implementation and testing. The system uses Better-Auth for authentication (email/password + Google OAuth), captures 7-question onboarding profiles, and applies client-side content personalization based on user hardware/software background.

### User Stories (from spec.md)

- **US1** (P1 🎯 MVP): User Authentication with Better-Auth
- **US2** (P1 🎯 MVP): Onboarding Quiz with 7 Background Questions
- **US3** (P1 🎯 MVP): Per-Chapter Personalization Button
- **US4** (P2): Urdu Translation Compatibility with Personalization
- **US5** (P3): Profile Management Dashboard

### MVP Scope

**MVP = US1 + US2 + US3** (Authentication + Quiz + Personalization)

This delivers the core 50-point bonus functionality: working auth, 7 questions, personalized content.

---

## Phase 1: Setup & Infrastructure

**Goal**: Initialize project structure, install dependencies, configure environments

**Duration**: ~2-3 hours

### Dependencies Setup

- [ ] T001 Install Better-Auth packages in backend/auth-service: `npm install better-auth@^0.4.0 @better-auth/postgres @neondatabase/serverless zod express cors dotenv`
- [ ] T002 [P] Install Better-Auth React package in docusaurus: `npm install @better-auth/react react-hook-form zod`
- [ ] T003 [P] Install JWT validation packages in backend (FastAPI): `pip install pyjwt cryptography python-dotenv`

### Project Structure

- [ ] T004 Create backend/auth-service directory structure (src/, tests/, src/db/, src/routes/, src/middleware/, src/utils/)
- [ ] T005 [P] Create migrations directory at project root with 001_create_users_table.sql placeholder
- [ ] T006 [P] Create docusaurus/src/components/Auth/ directory
- [ ] T007 [P] Create docusaurus/src/components/Personalization/ directory
- [ ] T008 [P] Create docusaurus/src/contexts/ directory
- [ ] T009 [P] Create docusaurus/src/hooks/ directory
- [ ] T010 [P] Create docusaurus/src/utils/ directory with personalize.ts and profile-hash.ts placeholders

### Configuration Files

- [ ] T011 Create backend/auth-service/.env.example with NEON_DATABASE_URL, BETTER_AUTH_SECRET, GOOGLE_CLIENT_ID, GOOGLE_CLIENT_SECRET, CORS_ORIGINS placeholders
- [ ] T012 [P] Create backend/auth-service/tsconfig.json with TypeScript 5.3+ config
- [ ] T013 [P] Create backend/auth-service/package.json with scripts (dev, build, test)
- [ ] T014 [P] Create docusaurus/.env.local.example with REACT_APP_AUTH_SERVICE_URL, REACT_APP_GOOGLE_CLIENT_ID placeholders

### Database Migration

- [ ] T015 Create migrations/001_create_users_table.sql with CREATE TABLE users (id UUID PRIMARY KEY, email VARCHAR(255) UNIQUE NOT NULL, password_hash VARCHAR(255), auth_provider VARCHAR(50) DEFAULT 'email', google_id VARCHAR(255) UNIQUE, profile_data JSONB DEFAULT '{}', created_at TIMESTAMPTZ DEFAULT NOW(), updated_at TIMESTAMPTZ DEFAULT NOW(), last_login TIMESTAMPTZ)
- [ ] T016 Add indexes to migration: CREATE INDEX idx_users_email ON users(email); CREATE INDEX idx_users_google_id ON users(google_id); CREATE INDEX idx_users_profile_data ON users USING GIN(profile_data);
- [ ] T017 Run migration against Neon database: `psql $NEON_DATABASE_URL -f migrations/001_create_users_table.sql`

---

## Phase 2: Foundational Components (Blocking Prerequisites)

**Goal**: Implement shared utilities and contexts needed by all user stories

**Duration**: ~3-4 hours

**Dependencies**: Phase 1 must be complete

### Shared TypeScript Types

- [ ] T018 Create backend/auth-service/src/types.ts with AuthProvider, HardwareExperience, GpuAccess, Ros2Experience, PythonLevel, LearningEnvironment, HardwareBudget, LearningGoal enums
- [ ] T019 Add UserProfile interface in backend/auth-service/src/types.ts with 7 quiz fields + completed_at + version
- [ ] T020 [P] Add User interface in backend/auth-service/src/types.ts with id, email, auth_provider, google_id, profile_data, created_at, updated_at, last_login
- [ ] T021 [P] Copy types to docusaurus/src/types.ts (shared between frontend and backend)

### Profile Hash Utility (Shared Logic)

- [ ] T022 Implement computeProfileHash() in backend/auth-service/src/utils/profile-hash.ts using crypto.createHash('sha256') on sorted profile JSON
- [ ] T023 [P] Copy profile-hash logic to docusaurus/src/utils/profile-hash.ts (browser-compatible version using Web Crypto API)
- [ ] T024 Add unit test for profile hash: verify same profile → same hash, different profile → different hash in backend/auth-service/tests/profile-hash.test.ts

### Neon Database Client

- [ ] T025 Implement Neon Postgres connection in backend/auth-service/src/db/client.ts using @neondatabase/serverless with connection pooling
- [ ] T026 Add helper functions in backend/auth-service/src/db/client.ts: getUserByEmail(), getUserById(), createUser(), updateUserProfile()

### Validation Schemas

- [ ] T027 Create Zod schemas in backend/auth-service/src/validation.ts: SignupRequestSchema (email, password), UserProfileSchema (7 quiz fields), QuizSubmissionSchema
- [ ] T028 [P] Export validation schemas to docusaurus via docusaurus/src/validation.ts (copy from backend for client-side validation)

---

## Phase 3: User Story 1 - Authentication with Better-Auth (P1 🎯 MVP)

**Goal**: Users can sign up and log in with email/password OR Google OAuth

**Independent Test**: Create account at /signup, verify user in Neon database, log out, log in again with same credentials. Test Google OAuth flow.

**Duration**: ~6-8 hours

**Dependencies**: Phase 2 complete

### Better-Auth Service Setup

- [ ] T029 [US1] Configure Better-Auth in backend/auth-service/src/auth.config.ts with email provider (bcrypt password hashing) and Google OAuth provider (GOOGLE_CLIENT_ID, GOOGLE_CLIENT_SECRET)
- [ ] T030 [US1] Create Express server in backend/auth-service/src/index.ts with CORS middleware (allow http://localhost:3000), Better-Auth routes mounted at /auth
- [ ] T031 [US1] Add session management config in backend/auth-service/src/auth.config.ts: HTTP-only cookies, 7-day expiration, CSRF protection enabled

### Auth Endpoints

- [ ] T032 [US1] Implement POST /auth/signup endpoint in backend/auth-service/src/routes/auth.ts: validate email/password, hash password, insert user into Neon, return JWT + user_id
- [ ] T033 [US1] Implement POST /auth/login endpoint in backend/auth-service/src/routes/auth.ts: validate credentials, query Neon, return JWT + profile_completed boolean
- [ ] T034 [US1] Implement GET /auth/oauth/google endpoint in backend/auth-service/src/routes/auth.ts: redirect to Google consent screen
- [ ] T035 [US1] Implement GET /auth/oauth/google/callback endpoint in backend/auth-service/src/routes/auth.ts: exchange code for tokens, create/update user in Neon, redirect to frontend with JWT cookie
- [ ] T036 [US1] Implement POST /auth/logout endpoint in backend/auth-service/src/routes/auth.ts: invalidate session token in Neon sessions table

### JWT Middleware

- [ ] T037 [US1] Create JWT validation middleware in backend/auth-service/src/middleware/validate-session.ts: verify JWT signature, extract user_id, attach to request object
- [ ] T038 [P] [US1] Implement JWT validation in backend/app/middleware/auth.py (FastAPI): verify JWT using public key from Better-Auth, extract user_id and profile_hash

### Frontend Auth Components

- [ ] T039 [US1] Create SignupForm component in docusaurus/src/components/Auth/SignupForm.tsx: email/password fields + validation (Zod), "Sign up with Google" button, form submission to POST /auth/signup
- [ ] T040 [US1] Create LoginForm component in docusaurus/src/components/Auth/LoginForm.tsx: email/password fields, "Sign in with Google" button, form submission to POST /auth/login
- [ ] T041 [US1] Create signup page in docusaurus/src/pages/signup.tsx with SignupForm component
- [ ] T042 [US1] Create login page in docusaurus/src/pages/login.tsx with LoginForm component

### Auth Context & Hooks

- [ ] T043 [US1] Create AuthContext in docusaurus/src/contexts/AuthContext.tsx: manage user session (user_id, email, profile_data), isAuthenticated boolean, login/logout/signup functions
- [ ] T044 [US1] Wrap Docusaurus app in AuthProvider in docusaurus/src/theme/Root.tsx: import AuthContext, wrap children with <AuthProvider>
- [ ] T045 [US1] Create useAuth hook in docusaurus/src/hooks/useAuth.ts: export login(), logout(), signup(), getSession() functions using AuthContext

### Startup & Manual Testing

- [ ] T046 [US1] Start Better-Auth service: `cd backend/auth-service && npm run dev` (verify server runs on http://localhost:3001)
- [ ] T047 [US1] Start Docusaurus: `cd docusaurus && npm start` (verify site runs on http://localhost:3000)
- [ ] T048 [US1] **Manual Test**: Navigate to /signup, create account with email test@example.com and password SecurePass123!, verify redirect to /onboarding
- [ ] T049 [US1] **Manual Test**: Query Neon database: `psql $NEON_DATABASE_URL -c "SELECT email, auth_provider FROM users WHERE email='test@example.com';"` (verify user record exists)
- [ ] T050 [US1] **Manual Test**: Log out, log in again with same credentials, verify successful authentication

---

## Phase 4: User Story 2 - Onboarding Quiz (P1 🎯 MVP)

**Goal**: New users complete 7-question quiz about hardware/software background, answers saved to Neon profile_data JSONB

**Independent Test**: After signup, verify quiz appears with exactly 7 questions. Submit answers, verify saved in `users.profile_data` JSONB column. Test "Personalize Later" skip option.

**Duration**: ~5-6 hours

**Dependencies**: US1 complete (authentication working)

### Quiz Component

- [ ] T051 [US2] Create OnboardingQuiz component in docusaurus/src/components/Auth/OnboardingQuiz.tsx with 7 questions as radio button groups (hardware_experience, gpu_access, ros2_experience, python_level, learning_environment, hardware_budget, learning_goal)
- [ ] T052 [US2] Add progress indicator to OnboardingQuiz showing current question (e.g., "Question 3 of 7") or overall progress bar
- [ ] T053 [US2] Implement form validation in OnboardingQuiz: all 7 questions required before "Save & Continue" enabled, display validation errors for missing fields
- [ ] T054 [US2] Add "Personalize Later" button in OnboardingQuiz: skip quiz, save default profile (hardware_experience: 'none', gpu_access: 'none', ros2_experience: 'none', python_level: 'beginner', learning_environment: 'cloud_only', hardware_budget: 'none', learning_goal: 'academic')

### Quiz Submission Logic

- [ ] T055 [US2] Save draft answers to localStorage in OnboardingQuiz: on input change, persist partial answers to localStorage key 'quiz_draft', restore on component mount
- [ ] T056 [US2] Implement quiz submission in OnboardingQuiz: POST to /profile/quiz with answers JSON, handle success (redirect to /docs/ch01-intro), handle validation errors (display inline)

### Profile API Endpoints

- [ ] T057 [US2] Implement POST /profile/quiz endpoint in backend/auth-service/src/routes/profile.ts: validate JWT, validate quiz answers (Zod schema), compute profile hash, update users.profile_data JSONB, increment version to 1, return profile_hash + redirect_url
- [ ] T058 [US2] Add default profile utility in backend/auth-service/src/utils/defaults.ts: getDefaultProfile() function returning default values for skipped quiz

### Quiz Page

- [ ] T059 [US2] Create onboarding page in docusaurus/src/pages/onboarding.tsx with OnboardingQuiz component, protected route (redirect to /login if not authenticated)
- [ ] T060 [US2] Add post-signup redirect in docusaurus/src/components/Auth/SignupForm.tsx: after successful signup, redirect to /onboarding instead of dashboard

### Manual Testing

- [ ] T061 [US2] **Manual Test**: Sign up new user, verify redirect to /onboarding, see 7 questions with correct options (4 choices per question)
- [ ] T062 [US2] **Manual Test**: Fill all 7 questions, click "Save & Continue", verify redirect to /docs/ch01-intro
- [ ] T063 [US2] **Manual Test**: Query Neon: `psql $NEON_DATABASE_URL -c "SELECT profile_data FROM users WHERE email='test@example.com';"` (verify JSONB contains 7 quiz answers + completed_at + version: 1)
- [ ] T064 [US2] **Manual Test**: Start quiz, navigate away, return to /onboarding, verify draft answers restored from localStorage

---

## Phase 5: User Story 3 - Per-Chapter Personalization (P1 🎯 MVP)

**Goal**: Every chapter has "Personalize this chapter" button that instantly adapts content based on user profile (<500ms transformation)

**Independent Test**: Navigate to Chapter 4, click "Personalize this chapter", verify content transforms immediately (cloud vs local instructions, beginner vs advanced sections). Check Urdu button still works.

**Duration**: ~8-10 hours

**Dependencies**: US2 complete (quiz working, profiles in database)

### Personalization Rules Configuration

- [ ] T065 [US3] Create personalization-rules.yaml in docusaurus/static/ with version: "1.0.0", defaults (show_all_sections: true, simplify_jargon: false, default_instructions: "hybrid")
- [ ] T066 [US3] Add beginner-no-gpu rule in personalization-rules.yaml: condition (hardware_experience: [none, some], gpu_access: [none, consumer]), transformations (hide advanced sections, show cloud setup, simplify_jargon: true, add beginner callouts)
- [ ] T067 [US3] Add expert-highend-gpu rule in personalization-rules.yaml: condition (hardware_experience: [expert, proficient], gpu_access: [highend]), transformations (show advanced sections, hide beginner intro, default_instructions: "local", skip_basics: true)
- [ ] T068 [US3] Add ros2-beginner rule, python-beginner rule, academic-learner rule, hardware-budget rules in personalization-rules.yaml (total ~10-15 rules covering 7 profile dimensions)

### Personalization Engine

- [ ] T069 [US3] Implement loadRules() in docusaurus/src/utils/personalize.ts: fetch /personalization-rules.yaml, parse YAML, validate against Zod schema, return PersonalizationRules object
- [ ] T070 [US3] Implement matchesProfile() helper in docusaurus/src/utils/personalize.ts: given rule condition and user profile, return true if all condition fields match profile (AND logic)
- [ ] T071 [US3] Implement applySectionFilter() in docusaurus/src/utils/personalize.ts: hide sections (remove DOM elements with data-section-id), show sections (conditional render)
- [ ] T072 [US3] Implement applyTextReplacements() in docusaurus/src/utils/personalize.ts: replace text patterns (e.g., "local installation" → "AWS EC2 setup" for cloud-first users)
- [ ] T073 [US3] Implement addCallouts() in docusaurus/src/utils/personalize.ts: inject React components for tips/warnings/info callouts at specified positions
- [ ] T074 [US3] Implement personalize() main function in docusaurus/src/utils/personalize.ts: load rules, match profile, apply transformations, return PersonalizedContent object with markdown + profile_hash + rules_applied metadata

### Profile Management in Context

- [ ] T075 [US3] Add profile loading in docusaurus/src/contexts/AuthContext.tsx: on login/signup success, fetch GET /profile, store profile_data in context state
- [ ] T076 [US3] Create useProfile hook in docusaurus/src/hooks/useProfile.ts: export profile, profileHash (computed), isProfileComplete, updateProfile() functions

### Personalization Context & Hook

- [ ] T077 [US3] Create PersonalizationContext in docusaurus/src/contexts/PersonalizationContext.tsx: manage personalization state (isPersonalized boolean, personalizedContent, rules), provide applyPersonalization(), resetPersonalization() functions
- [ ] T078 [US3] Create usePersonalization hook in docusaurus/src/hooks/usePersonalization.ts: export isPersonalized, applyPersonalization(), resetPersonalization(), personalizationMetadata

### Personalization UI Components

- [ ] T079 [US3] Create PersonalizeButton component in docusaurus/src/components/Personalization/PersonalizeButton.tsx: button with onClick → toggle personalization, text changes ("Personalize this chapter" ↔ "Show Original Content"), disabled if user not logged in
- [ ] T080 [US3] Create ProfileIndicator component in docusaurus/src/components/Personalization/ProfileIndicator.tsx: banner showing "Personalized for: Cloud-first Beginner" with link to /profile, only visible when isPersonalized === true

### Chapter Integration

- [ ] T081 [US3] Add PersonalizeButton to docusaurus/docs/ch01-intro.md: import component at top, add <PersonalizeButton /> before main content
- [ ] T082 [US3] Add personalization markers to docusaurus/docs/ch04-gazebo.md: wrap sections in divs with data-section-id (e.g., <div data-section-id="advanced-hardware">), mark cloud vs local instructions
- [ ] T083 [US3] Add PersonalizeButton to remaining 11 chapters: ch02-ros2.md through ch13-capstone.md (copy-paste pattern from ch01)
- [ ] T084 [US3] Update existing TranslateButton component in docusaurus/src/components/Translation/TranslateButton.tsx: include profile hash in Qdrant cache key (ch{id}_{profileHash}_urdu instead of ch{id}_original_urdu)

### Manual Testing

- [ ] T085 [US3] **Manual Test**: Log in as user with profile (hardware_experience: none, gpu_access: none), navigate to /docs/ch04-gazebo, click "Personalize this chapter"
- [ ] T086 [US3] **Manual Test**: Verify content transforms within <500ms (use browser DevTools Performance tab): cloud setup shown, local setup hidden, beginner callouts added, advanced sections hidden
- [ ] T087 [US3] **Manual Test**: Click "Show Original Content", verify page reverts to default (non-personalized) view
- [ ] T088 [US3] **Manual Test**: Create second user with profile (hardware_experience: expert, gpu_access: highend), navigate to ch04, personalize, verify advanced configs shown, local setup prioritized, beginner content hidden

---

## Phase 6: User Story 4 - Urdu Translation Compatibility (P2)

**Goal**: Urdu translation works on personalized content variants with profile-specific Qdrant cache keys

**Independent Test**: Personalize Chapter 6 for cloud-first beginner, click "Translate to Urdu", verify Urdu text reflects personalized variant (cloud instructions, not local). Repeat translation, verify cache hit (<200ms).

**Duration**: ~3-4 hours

**Dependencies**: US3 complete (personalization working)

### Cache Key Updates

- [ ] T089 [P] [US4] Update generateTranslationCacheKey() in docusaurus/src/utils/api-client.ts: change signature to accept profileHash parameter, format key as `ch{id}_{profileHash.substring(0,12)}_{language}`
- [ ] T090 [P] [US4] Update Qdrant query in backend/app/agent.py: when fetching translation cache, include profile_hash in filter query

### Translation Flow Updates

- [ ] T091 [US4] Update TranslateButton onClick handler in docusaurus/src/components/Translation/TranslateButton.tsx: compute profileHash from useProfile(), pass to translation API call
- [ ] T092 [US4] Modify translation API endpoint in backend/app/main.py: accept profile_hash query parameter, use in Qdrant cache key lookup
- [ ] T093 [US4] Add profile hash to translation storage in backend/app/agent.py: when storing new Urdu translation in Qdrant, include profile_hash in payload metadata

### Cache Invalidation Logic

- [ ] T094 [US4] Implement cache invalidation on profile update in backend/auth-service/src/routes/profile.ts: in PUT /profile endpoint, after updating profile_data, check if hash changed, log message "Profile hash changed: {old} → {new}, translation cache auto-invalidated"
- [ ] T095 [US4] Add TTL metadata to Qdrant translation points in backend/app/agent.py: set ttl_days: 30 in payload for client-enforced cache expiry

### Manual Testing

- [ ] T096 [US4] **Manual Test**: Log in, personalize Chapter 6 (Isaac Sim) for cloud-first beginner profile, click "Translate to Urdu", verify Urdu translation reflects cloud setup (not local)
- [ ] T097 [US4] **Manual Test**: Click "Translate to Urdu" again, verify cache hit (translation loads <200ms, check Network tab in DevTools for cache response header)
- [ ] T098 [US4] **Manual Test**: Update profile (change gpu_access from none to highend), re-personalize Chapter 6, translate to Urdu, verify NEW translation generated with updated profile hash (cache miss expected)

---

## Phase 7: User Story 5 - Profile Management Dashboard (P3)

**Goal**: Users can view and edit their 7 quiz answers from profile page, changes immediately reflected in personalization

**Independent Test**: Navigate to /profile, see all 7 answers editable. Change "GPU Access" from none to highend, save, navigate to Chapter 4, personalize, verify local setup shown instead of cloud.

**Duration**: ~4-5 hours

**Dependencies**: US2 complete (profile exists), US3 complete (personalization working)

### Profile Dashboard Component

- [ ] T099 [US5] Create ProfileDashboard component in docusaurus/src/components/Auth/ProfileDashboard.tsx: display user email, created_at date, quiz answers as form with radio buttons (pre-filled with current profile_data)
- [ ] T100 [US5] Add form validation in ProfileDashboard: use react-hook-form + Zod schema, validate all 7 fields, disable "Save Changes" until form is valid and dirty (changed)
- [ ] T101 [US5] Implement profile update submission in ProfileDashboard: on form submit, PUT /profile with updated answers, handle success (show toast "Profile updated"), handle errors (display validation messages)

### Profile Update Endpoint

- [ ] T102 [US5] Implement PUT /profile endpoint in backend/auth-service/src/routes/profile.ts: validate JWT, validate updated profile (Zod schema), compute new profile hash, update users.profile_data, increment version, return new profile_hash + version
- [ ] T103 [US5] Add rate limiting to PUT /profile in backend/auth-service/src/middleware/rate-limit.ts: max 5 updates per hour per user (using express-rate-limit middleware), return 429 if exceeded

### Profile Page

- [ ] T104 [US5] Create profile page in docusaurus/src/pages/profile.tsx with ProfileDashboard component, protected route (redirect to /login if not authenticated)
- [ ] T105 [US5] Add profile link to navigation in docusaurus/docusaurus.config.js: add "Profile" link in navbar pointing to /profile (visible only when authenticated)

### Cache Invalidation on Profile Update

- [ ] T106 [US5] Update AuthContext in docusaurus/src/contexts/AuthContext.tsx: after profile update success, refetch GET /profile to update context state with new profile_data + profile_hash
- [ ] T107 [US5] Clear personalization state in docusaurus/src/contexts/PersonalizationContext.tsx: on profile hash change detected, reset isPersonalized to false, clear personalizedContent cache

### Manual Testing

- [ ] T108 [US5] **Manual Test**: Navigate to /profile, verify all 7 quiz answers displayed with current values
- [ ] T109 [US5] **Manual Test**: Change "GPU Access" from "No GPU" to "High-end GPU", change "Learning Environment" from "Cloud only" to "Local only", click "Save Changes"
- [ ] T110 [US5] **Manual Test**: Verify success message shown, navigate to /docs/ch04-gazebo, click "Personalize this chapter", verify local setup now shown (not cloud), GPU optimization tips visible

---

## Phase 8: Polish & Cross-Cutting Concerns

**Goal**: Final touches, error handling, deployment preparation

**Duration**: ~3-4 hours

**Dependencies**: All user stories complete

### Error Handling & Edge Cases

- [ ] T111 [P] Add error boundary in docusaurus/src/theme/Root.tsx: catch React errors, display friendly error message ("Something went wrong. Please refresh the page.")
- [ ] T112 [P] Implement Neon connection fallback in docusaurus/src/contexts/AuthContext.tsx: if GET /profile fails (Neon unreachable), load profile from localStorage cache, show warning banner "Using cached profile (offline mode)"
- [ ] T113 [P] Add localStorage profile caching in docusaurus/src/hooks/useProfile.ts: on successful profile fetch, save to localStorage key 'profile_cache', expire after 24 hours

### Validation & Testing

- [ ] T114 Validate YAML syntax in personalization-rules.yaml: run `npx js-yaml docusaurus/static/personalization-rules.yaml` (verify no parse errors)
- [ ] T115 Test profile hash consistency in backend/auth-service/tests/profile-hash.test.ts: verify identical profile → identical hash across 1000 iterations, verify hash changes when any field changes
- [ ] T116 **E2E Test** (optional): Playwright test in backend/auth-service/tests/integration/e2e-auth.spec.ts: signup → quiz → login → personalize → logout flow, verify database state at each step

### Documentation

- [ ] T117 [P] Update README.md in project root: add "Authentication & Personalization" section documenting signup flow, quiz questions, personalization usage
- [ ] T118 [P] Create .env.example files: backend/auth-service/.env.example (all required env vars), docusaurus/.env.local.example, backend/.env.example (add BETTER_AUTH_JWT_PUBLIC_KEY)

### Deployment Preparation

- [ ] T119 Build Better-Auth service for production: `cd backend/auth-service && npm run build` (verify TypeScript compiles, dist/ folder created)
- [ ] T120 [P] Build Docusaurus for production: `cd docusaurus && npm run build` (verify static site generated in build/ folder, no build errors)
- [ ] T121 Test production builds locally: serve Better-Auth (`node dist/index.js`) and Docusaurus (`npx serve build`), verify auth flow works with production builds

---

## Dependency Graph

### User Story Completion Order

```
Phase 1 (Setup) → Phase 2 (Foundational)
                        ↓
                  US1 (Authentication)
                        ↓
                  US2 (Quiz)
                        ↓
                  US3 (Personalization)
                        ↓
         ┌──────────────┴──────────────┐
         ↓                             ↓
    US4 (Urdu Compat)          US5 (Profile Dashboard)
         └──────────────┬──────────────┘
                        ↓
                Phase 8 (Polish)
```

### Critical Path

**Phase 1 → Phase 2 → US1 → US2 → US3** (MVP delivery)

All other stories (US4, US5) can proceed in parallel after US3 completes.

### Blocking Tasks

- **T017** (Database migration) blocks all US1 tasks (no database = no auth)
- **T029-T031** (Better-Auth config) blocks all US1 endpoint tasks
- **T057** (POST /profile/quiz endpoint) blocks US2 completion
- **T074** (personalize() main function) blocks US3 completion

---

## Parallel Execution Examples

### Phase 1 - Setup (High Parallelism)

**Sequential**:
- T001 (npm install better-auth) must complete first

**Parallel Group 1** (after T001):
- T002 (npm install @better-auth/react) [P]
- T003 (pip install pyjwt) [P]

**Parallel Group 2** (after T004 creates structure):
- T005 (create migrations/) [P]
- T006 (create Auth/) [P]
- T007 (create Personalization/) [P]
- T008 (create contexts/) [P]
- T009 (create hooks/) [P]
- T010 (create utils/) [P]

**Parallel Group 3** (config files):
- T011 (auth-service .env.example) [P]
- T012 (auth-service tsconfig.json) [P]
- T013 (auth-service package.json) [P]
- T014 (docusaurus .env.local.example) [P]

### Phase 3 - US1 (Moderate Parallelism)

**Sequential**:
- T029-T031 (Better-Auth config) must complete first

**Parallel Group 1** (auth endpoints, independent files):
- T032 (POST /signup) [P]
- T033 (POST /login) [P]
- T034 (GET /oauth/google) [P]
- T035 (GET /oauth/google/callback) [P]
- T036 (POST /logout) [P]

**Parallel Group 2** (frontend + backend middleware):
- T037 (JWT middleware in auth-service)
- T038 (JWT middleware in FastAPI) [P]
- T039 (SignupForm component) [P]
- T040 (LoginForm component) [P]

**Parallel Group 3** (pages):
- T041 (signup page) [P]
- T042 (login page) [P]

### Phase 5 - US3 (High Parallelism)

**Sequential**:
- T065-T068 (personalization rules YAML) must complete first
- T069-T074 (personalization engine) must complete before T077-T080

**Parallel Group 1** (engine implementation, different functions):
- T070 (matchesProfile helper) [P]
- T071 (applySectionFilter) [P]
- T072 (applyTextReplacements) [P]
- T073 (addCallouts) [P]

**Parallel Group 2** (contexts + components):
- T077 (PersonalizationContext) [P]
- T079 (PersonalizeButton) [P]
- T080 (ProfileIndicator) [P]

**Parallel Group 3** (chapter integration, fully parallelizable):
- T081 (add button to ch01) [P]
- T082 (add markers to ch04) [P]
- T083 (add button to ch02-ch13) [P]

---

## Implementation Strategy

### Week 1: MVP (US1 + US2 + US3)

**Day 1-2**: Phase 1 + Phase 2 (setup + foundational)
- Install dependencies, create project structure, database migration
- Implement shared types, profile hash, validation schemas

**Day 3-4**: US1 (Authentication)
- Better-Auth service setup, auth endpoints, JWT middleware
- Frontend signup/login forms, AuthContext

**Day 5**: US2 (Quiz)
- OnboardingQuiz component, POST /profile/quiz endpoint
- Test quiz submission, verify profile_data in Neon

**Day 6-7**: US3 (Personalization)
- Personalization rules YAML, personalize() engine
- PersonalizeButton component, chapter integration

### Week 2: P2/P3 Stories + Polish

**Day 1**: US4 (Urdu Compatibility)
- Update translation cache keys with profile hash
- Test personalized Urdu translations

**Day 2**: US5 (Profile Dashboard)
- ProfileDashboard component, PUT /profile endpoint
- Test profile updates, verify cache invalidation

**Day 3**: Phase 8 (Polish)
- Error handling, edge cases, localStorage fallback
- Validation, E2E tests (optional)

**Day 4-5**: Deployment & Verification
- Build production artifacts, deploy to Vercel/Railway
- Run full /sp.check manual verification

---

## Verification Checklist (Manual Testing)

After completing all phases, run these verification steps:

### Authentication (US1)

- [ ] Navigate to http://localhost:3000/signup
- [ ] Create account with email test1@example.com, password SecurePass123!
- [ ] Verify redirect to /onboarding
- [ ] Log out, log in again with same credentials
- [ ] Click "Sign in with Google", complete OAuth flow, verify redirect
- [ ] Query Neon: `SELECT email, auth_provider FROM users;` (verify both email and google users exist)

### Onboarding Quiz (US2)

- [ ] Create new account test2@example.com
- [ ] See 7 questions: Hardware Experience, GPU Access, ROS 2 Knowledge, Python Proficiency, Learning Environment, Hardware Budget, Learning Goals
- [ ] Answer all questions: none, none, none, beginner, cloud_only, none, academic
- [ ] Click "Save & Continue", verify redirect to /docs/ch01-intro
- [ ] Query Neon: `SELECT profile_data FROM users WHERE email='test2@example.com';` (verify JSONB with 7 fields + version: 1)

### Personalization (US3)

- [ ] Log in as test2@example.com (cloud-only, beginner profile)
- [ ] Navigate to /docs/ch04-gazebo
- [ ] Click "Personalize this chapter"
- [ ] Verify within <500ms: Cloud setup (AWS EC2) shown, local setup hidden, beginner callouts added
- [ ] Create new account test3@example.com with expert profile (highend GPU, local-only)
- [ ] Navigate to /docs/ch04-gazebo, personalize
- [ ] Verify: Local setup shown, advanced configs visible, beginner content hidden

### Urdu Translation (US4)

- [ ] Log in as test2@example.com, navigate to /docs/ch06-isaac-sim
- [ ] Click "Personalize this chapter" (cloud-first)
- [ ] Click "Translate to Urdu"
- [ ] Verify Urdu text reflects cloud setup (not local)
- [ ] Click "Translate to Urdu" again, verify cache hit (<200ms, check Network tab)

### Profile Dashboard (US5)

- [ ] Log in as test2@example.com, navigate to /profile
- [ ] See 7 quiz answers editable
- [ ] Change "GPU Access" from "No GPU" to "High-end GPU"
- [ ] Change "Learning Environment" from "Cloud only" to "Local only"
- [ ] Click "Save Changes", see success message
- [ ] Navigate to /docs/ch04-gazebo, click "Personalize this chapter"
- [ ] Verify local setup now shown (updated profile applied)

---

## Summary

**Total Tasks**: 121 tasks

**Task Breakdown by Phase**:
- Phase 1 (Setup): 17 tasks (~2-3 hours)
- Phase 2 (Foundational): 10 tasks (~3-4 hours)
- Phase 3 (US1 - Auth): 22 tasks (~6-8 hours)
- Phase 4 (US2 - Quiz): 14 tasks (~5-6 hours)
- Phase 5 (US3 - Personalization): 24 tasks (~8-10 hours)
- Phase 6 (US4 - Urdu Compat): 10 tasks (~3-4 hours)
- Phase 7 (US5 - Profile Dashboard): 12 tasks (~4-5 hours)
- Phase 8 (Polish): 12 tasks (~3-4 hours)

**Parallel Opportunities**: 47 tasks marked [P] (39% parallelizable)

**MVP Scope**: Phase 1 + Phase 2 + US1 + US2 + US3 = 87 tasks (~32-38 hours)

**Independent Tests**: Each user story has clear independent test criteria, enabling separate implementation and verification.

**Ready for Implementation**: All tasks follow strict checklist format with Task IDs, Story labels, file paths, and clear descriptions. Immediately executable by LLM or human developer.

---

**Tasks Status**: ✅ Complete and ready for execution
**Next Step**: Begin Phase 1 (Setup) or run `/sp.implement` to start automated task execution
