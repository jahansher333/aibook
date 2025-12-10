# Task Breakdown: Better-Auth with FastAPI Backend Integration

**Feature Branch**: `005-better-auth-fastapi`
**Created**: 2025-12-09
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

## Overview

This task breakdown implements Better-Auth React integration with FastAPI backend authentication endpoints. The system uses a **custom adapter** (not the Better-Auth Node.js library) to route authentication requests from Docusaurus frontend to FastAPI backend. User profiles with 7 background questions are stored in Neon Postgres and used for content personalization.

### User Stories (from spec.md)

- **US1** (P1 🎯 MVP): User Registration with 7 Background Questions
- **US2** (P1 🎯 MVP): User Authentication and Session Management
- **US3** (P1 🎯 MVP): Content Personalization Based on Profile
- **US4** (P2): Profile Management and Updates
- **US5** (P3): Google OAuth Authentication

### MVP Scope

**MVP = US1 + US2 + US3** (Authentication + Quiz + Personalization)

This delivers the core "50 bonus points" functionality: FastAPI handles all auth, 7 questions captured, personalized content works.

---

## Phase 1: Setup & Infrastructure

**Goal**: Install dependencies, create database schema, configure environment

**Duration**: ~2 hours

### Backend Dependencies

- [ ] T001 Install FastAPI authentication packages: `cd backend && pip install python-jose[cryptography] passlib[bcrypt] python-multipart`
- [ ] T002 [P] Install database packages: `pip install sqlalchemy psycopg[binary] alembic`
- [ ] T003 [P] Verify existing FastAPI server runs: `uvicorn app.main:app --reload --port 8000`

### Frontend Dependencies

- [ ] T004 Install Better-Auth React: `cd docusaurus && npm install @better-auth/react`
- [ ] T005 [P] Verify Docusaurus builds: `npm run build`

### Database Schema

- [ ] T006 Create database migration file: `backend/alembic/versions/001_create_auth_tables.py` with users and user_profiles tables
- [ ] T007 Define users table: id (UUID), email (VARCHAR unique), password_hash (VARCHAR nullable), provider (VARCHAR), google_id (VARCHAR nullable unique), created_at, last_login
- [ ] T008 Define user_profiles table: id (UUID), user_id (FK to users), rtx_gpu (JSONB), jetson_board, ubuntu_experience, ros2_knowledge, sim_preference, learning_goal, preferred_language, completed_at, version (INT), profile_hash (VARCHAR 64)
- [ ] T009 Add indexes: users.email (unique), users.google_id (unique), user_profiles.user_id (unique), user_profiles.profile_hash (btree)
- [ ] T010 Run migration: `alembic upgrade head`

### Environment Configuration

- [ ] T011 [P] Add JWT environment variables to backend/.env: JWT_SECRET_KEY (min 32 chars), JWT_ALGORITHM=HS256, JWT_EXPIRE_DAYS=7
- [ ] T012 [P] Add frontend API URL to docusaurus/.env.local: REACT_APP_API_URL=http://localhost:8000

---

## Phase 2: Foundational Components (Blocking Prerequisites)

**Goal**: Implement shared utilities needed by all user stories

**Duration**: ~3-4 hours

**Dependencies**: Phase 1 must be complete

### SQLAlchemy Models

- [ ] T013 Create backend/app/models.py with User model (SQLAlchemy ORM mapping to users table)
- [ ] T014 [P] Add UserProfile model to backend/app/models.py (SQLAlchemy ORM with relationship to User)
- [ ] T015 [P] Create backend/app/database.py with engine, SessionLocal, Base, get_db dependency

### Pydantic Schemas

- [ ] T016 Create backend/app/schemas.py with UserCreate schema (email, password)
- [ ] T017 [P] Add ProfileCreate schema to backend/app/schemas.py (7 quiz fields: rtx_gpu, jetson_board, ubuntu_experience, ros2_knowledge, sim_preference, learning_goal, preferred_language)
- [ ] T018 [P] Add UserResponse, TokenResponse, ProfileResponse schemas to backend/app/schemas.py

### Authentication Utilities

- [ ] T019 Create backend/app/auth.py with password hashing functions: hash_password(password) using bcrypt cost=12, verify_password(plain, hashed)
- [ ] T020 Add JWT functions to backend/app/auth.py: create_access_token(data: dict, expires_delta: timedelta), decode_access_token(token: str) returns user_id
- [ ] T021 Add profile hash function to backend/app/auth.py: compute_profile_hash(profile: dict) returns SHA-256 hex string (sorted keys for determinism)
- [ ] T022 Create get_current_user dependency in backend/app/auth.py: extracts JWT from Authorization header, validates, returns user_id

### CRUD Operations

- [ ] T023 Create backend/app/crud.py with get_user_by_email(db, email) returns User or None
- [ ] T024 [P] Add create_user(db, user_create: UserCreate) to backend/app/crud.py: hash password, insert user, return User
- [ ] T025 [P] Add create_profile(db, user_id, profile_create: ProfileCreate) to backend/app/crud.py: compute hash, insert profile, return UserProfile
- [ ] T026 [P] Add get_profile_by_user_id(db, user_id) to backend/app/crud.py: returns UserProfile or None

---

## Phase 3: User Story 1 - User Registration with 7 Background Questions (P1 🎯 MVP)

**Goal**: Users can sign up with email/password and answer 7 background questions about hardware/software setup

**Independent Test**: Navigate to /signup, enter email/password, answer all 7 questions (RTX GPU, Jetson, Ubuntu, ROS2, simulation pref, goal, language), submit. Verify user exists in database with profile_data. Log out, log in again with same credentials.

**Duration**: ~6-8 hours

**Dependencies**: Phase 2 complete

### Backend: Signup Endpoint

- [ ] T027 [US1] Create backend/app/routes/auth.py with FastAPI router
- [ ] T028 [US1] Implement POST /api/auth/sign-up endpoint in backend/app/routes/auth.py: validate email (RFC 5322), validate password (min 8 chars), check if email exists (return 400 if duplicate)
- [ ] T029 [US1] In sign-up endpoint: call create_user(db, user_create), create JWT token with create_access_token({"user_id": user.id}), return {user, token}
- [ ] T030 [US1] Add optional profile parameter to sign-up endpoint: if profile provided, call create_profile(db, user.id, profile), compute profile_hash, return {user, token, profile_hash}
- [ ] T031 [US1] Register auth router in backend/app/main.py: `app.include_router(auth_router, prefix="/api/auth", tags=["auth"])`

### Frontend: Custom Better-Auth Adapter

- [ ] T032 [US1] Create docusaurus/src/lib/authClient.ts with custom FastAPI adapter object
- [ ] T033 [US1] In authClient.ts: implement fetch wrapper that calls FastAPI endpoints (baseUrl: process.env.REACT_APP_API_URL || 'http://localhost:8000')
- [ ] T034 [US1] In authClient.ts: implement signUp(email, password, profile) method that POSTs to /api/auth/sign-up, stores JWT in localStorage.setItem('auth_token', token)
- [ ] T035 [US1] Export useAuth() and useSession() hooks from authClient.ts (wraps @better-auth/react hooks with custom adapter)

### Frontend: Signup Form with 7 Questions

- [ ] T036 [US1] Create docusaurus/src/components/auth/SignupForm.tsx with email/password fields (validation: email format, password min 8 chars)
- [ ] T037 [US1] Add 7 background questions to SignupForm.tsx: (1) RTX GPU (boolean + model text), (2) Jetson board (select: none/nano/nx/agx), (3) Ubuntu experience (select: beginner/intermediate/expert), (4) ROS2 knowledge (select: none/basic/advanced), (5) Simulation preference (select: cloud/local/both), (6) Learning goal (select: learn_basics/build_humanoid/research), (7) Preferred language (select: english/urdu)
- [ ] T038 [US1] Implement form submission in SignupForm.tsx: call authClient.signUp({email, password, profile: {7 answers}}), on success redirect to /docs/ch01-intro, on error display message
- [ ] T039 [US1] Add "Personalize Later" button to SignupForm.tsx: calls signUp with empty profile (backend assigns default: no GPU, beginner, cloud, English), redirects to /docs/ch01-intro
- [ ] T040 [US1] Create docusaurus/src/pages/signup.tsx page route that renders SignupForm component

### Frontend: Auth Context Provider

- [ ] T041 [US1] Create docusaurus/src/contexts/AuthContext.tsx with AuthProvider component: wraps children with Better-Auth context, provides auth state (user, isAuthenticated, profile_hash)
- [ ] T042 [US1] Wrap Docusaurus app in AuthProvider: modify docusaurus/src/theme/Root.tsx to export `export default function Root({children}) { return <AuthProvider adapter={customAdapter}>{children}</AuthProvider>; }`

### Manual Testing

- [ ] T043 [US1] **Manual Test**: Navigate to http://localhost:3000/signup, create account with test@example.com / SecurePass123!, answer 7 questions, verify redirect to Chapter 1
- [ ] T044 [US1] **Manual Test**: Query database: `psql $NEON_DATABASE_URL -c "SELECT email, created_at FROM users WHERE email='test@example.com';"` (verify user exists)
- [ ] T045 [US1] **Manual Test**: Query profile: `psql $NEON_DATABASE_URL -c "SELECT rtx_gpu, jetson_board, ubuntu_experience FROM user_profiles WHERE user_id IN (SELECT id FROM users WHERE email='test@example.com');"` (verify 7 answers saved)

---

## Phase 4: User Story 2 - User Authentication and Session Management (P1 🎯 MVP)

**Goal**: Registered users can log in with email/password, session persists across page reloads

**Independent Test**: After signup, log out. Return to /login, enter credentials, verify successful login and redirect. Close browser, reopen site, verify session still active (within 7-day timeout).

**Duration**: ~4-5 hours

**Dependencies**: US1 complete (signup working)

### Backend: Login Endpoint

- [ ] T046 [US2] Implement POST /api/auth/sign-in endpoint in backend/app/routes/auth.py: accept {email, password}
- [ ] T047 [US2] In sign-in endpoint: call get_user_by_email(db, email), verify password with verify_password(password, user.password_hash), return 401 if invalid
- [ ] T048 [US2] In sign-in endpoint: if valid, create JWT token, update user.last_login = NOW(), return {user, token}
- [ ] T049 [US2] Add profile_hash to login response: if user has profile, get_profile_by_user_id(db, user.id), compute_profile_hash(profile), return {user, token, profile_hash}

### Backend: Session Validation Endpoint

- [ ] T050 [US2] Implement GET /api/auth/session endpoint in backend/app/routes/auth.py: use get_current_user dependency (validates JWT from Authorization header)
- [ ] T051 [US2] In session endpoint: return {user: {id, email, provider}, profile_completed: bool, expires_at: JWT expiration timestamp}

### Backend: User Profile Endpoint

- [ ] T052 [US2] Implement GET /api/auth/me endpoint in backend/app/routes/auth.py: use get_current_user dependency
- [ ] T053 [US2] In /me endpoint: get_profile_by_user_id(db, user_id), return {user: {...}, profile: {7 fields + version + profile_hash}} or profile: null if not completed

### Frontend: Login Form

- [ ] T054 [US2] Create docusaurus/src/components/auth/LoginForm.tsx with email/password fields
- [ ] T055 [US2] Implement form submission in LoginForm.tsx: call authClient.signIn({email, password}), store JWT in localStorage, on success redirect to /docs/ch01-intro, on error display "Invalid email or password"
- [ ] T056 [US2] Create docusaurus/src/pages/login.tsx page route that renders LoginForm component

### Frontend: Session Management

- [ ] T057 [US2] Implement getSession() method in docusaurus/src/lib/authClient.ts: read token from localStorage, call GET /api/auth/session with Authorization: Bearer {token}, return {user, isAuthenticated}
- [ ] T058 [US2] Add session restoration in AuthContext: on component mount, call authClient.getSession(), if valid restore user state, if 401 clear localStorage
- [ ] T059 [US2] Implement signOut() method in authClient.ts: remove 'auth_token' from localStorage, clear auth state in context

### Frontend: Logout UI

- [ ] T060 [US2] Add logout button to docusaurus navigation: modify docusaurus/docusaurus.config.js to add custom navbar item "Logout" (visible only when authenticated), onClick calls authClient.signOut() and redirects to /login

### Manual Testing

- [ ] T061 [US2] **Manual Test**: Log in as test@example.com, verify redirect to Chapter 1, check localStorage.getItem('auth_token') in DevTools (should show JWT)
- [ ] T062 [US2] **Manual Test**: Reload page, verify session persists (user still logged in, no redirect to login)
- [ ] T063 [US2] **Manual Test**: Click logout, verify token removed from localStorage, redirected to /login
- [ ] T064 [US2] **Manual Test**: Try login with wrong password, verify error message "Invalid email or password" displayed

---

## Phase 5: User Story 3 - Content Personalization Based on Profile (P1 🎯 MVP)

**Goal**: Users with completed profiles see personalized chapter content based on their hardware/software setup

**Independent Test**: Log in as user with no RTX GPU + cloud preference. Navigate to Chapter 4, click "Personalize this chapter". Verify cloud setup shown, local GPU hidden, callout recommends AWS g5.2xlarge. Test with different profile (RTX 4090 + local) to verify opposite behavior.

**Duration**: ~6-8 hours

**Dependencies**: US2 complete (login + session working)

### Frontend: Profile Data Access

- [ ] T065 [US3] Implement getUser() method in docusaurus/src/lib/authClient.ts: call GET /api/auth/me with JWT token, return {user, profile}
- [ ] T066 [US3] Create useProfile() hook in docusaurus/src/hooks/useProfile.ts: calls authClient.getUser(), returns {profile, profileHash, isProfileComplete}

### Frontend: Profile Hash Computation

- [ ] T067 [US3] Create docusaurus/src/utils/profileHash.ts with computeProfileHash(profile) function: sorts profile keys, JSON.stringify(), SHA-256 hash using Web Crypto API (browser-compatible)
- [ ] T068 [US3] Verify profile hash matches backend: frontend and backend must produce identical hashes for same profile (test with sample profile in browser console vs Python)

### Frontend: Personalization Engine Integration

- [ ] T069 [US3] Modify existing docusaurus/src/components/PersonalizeButton.tsx: add dependency on useProfile() hook
- [ ] T070 [US3] In PersonalizeButton onClick: call useProfile().profile, pass to existing personalization engine (if !profile, show "Please complete profile" message)
- [ ] T071 [US3] Update personalization logic in docusaurus/src/utils/personalize.ts: accept profile parameter, apply rules based on profile.rtx_gpu (if false, show cloud alternatives), profile.jetson_board (if not 'none', show Jetson deployment), profile.ubuntu_experience (if 'beginner', hide advanced sections)

### Personalization Rules Implementation

- [ ] T072 [US3] Add rule in personalize.ts: if profile.rtx_gpu === false AND profile.sim_preference === 'cloud', show AWS g5.2xlarge callout, hide local GPU setup sections (data-section-id="local-gpu")
- [ ] T073 [US3] Add rule in personalize.ts: if profile.jetson_board !== 'none', inject callout "Deploy to Jetson: flash weights locally" in deployment chapters
- [ ] T074 [US3] Add rule in personalize.ts: if profile.ubuntu_experience === 'beginner', hide advanced shell commands (data-section-id="advanced-shell"), add beginner tooltips
- [ ] T075 [US3] Add rule in personalize.ts: if profile.ros2_experience === 'none', show ROS2 basics sections, hide advanced node lifecycle management

### Translation Cache Integration

- [ ] T076 [US3] Update translation cache key generation in docusaurus/src/components/Translation/TranslateButton.tsx: include profile_hash in cache key format: ch{chapter_id}_{profile_hash.substring(0,12)}_{language}
- [ ] T077 [US3] Verify Urdu translations work with personalized content: profileHash ensures users with same profile share translation cache, different profiles get separate cache entries

### Manual Testing

- [ ] T078 [US3] **Manual Test**: Log in as test@example.com (no RTX, cloud preference), navigate to /docs/ch04-gazebo, click "Personalize this chapter"
- [ ] T079 [US3] **Manual Test**: Verify cloud setup instructions visible, local GPU sections hidden, AWS g5.2xlarge callout present
- [ ] T080 [US3] **Manual Test**: Create second user test2@example.com with RTX 4090 + local preference, navigate to ch04, personalize, verify local GPU instructions shown, cloud hidden
- [ ] T081 [US3] **Manual Test**: Test Urdu translation on personalized content: click "Translate to Urdu", verify translation reflects personalized version (cloud vs local), check Network tab for cache key format ch04_{hash}_urdu

---

## Phase 6: User Story 4 - Profile Management and Updates (P2)

**Goal**: Users can view and edit their 7 profile answers from profile dashboard

**Independent Test**: Log in, navigate to /profile, change answers (e.g., GPU from None to RTX 4090, sim preference from cloud to local), save. Navigate to ch04, personalize, verify new settings applied (local GPU shown, not cloud).

**Duration**: ~4-5 hours

**Dependencies**: US3 complete (personalization working)

### Backend: Profile Update Endpoint

- [ ] T082 [US4] Implement PUT /api/auth/profile endpoint in backend/app/routes/auth.py: use get_current_user dependency, accept ProfileUpdate schema (7 fields, all optional)
- [ ] T083 [US4] In profile update endpoint: get existing profile, merge updates, increment version number, recompute profile_hash, update database, return {profile: {...updated...}, version, profile_hash}
- [ ] T084 [US4] Add validation: if no fields changed, return 400 with message "No changes detected"

### Frontend: Profile Dashboard

- [ ] T085 [US4] Create docusaurus/src/components/auth/ProfileDashboard.tsx: display all 7 background answers as editable form (same fields as SignupForm but pre-filled with current values)
- [ ] T086 [US4] Implement form submission in ProfileDashboard.tsx: call authClient.updateProfile({changed fields}), on success show toast "Profile updated successfully", update local profile state with new profile_hash
- [ ] T087 [US4] Add "Cancel" button to ProfileDashboard: resets form to original values, no API call
- [ ] T088 [US4] Create docusaurus/src/pages/profile.tsx page route: renders ProfileDashboard, protected route (redirect to /login if not authenticated)

### Cache Invalidation

- [ ] T089 [US4] Add cache invalidation logic in AuthContext: when profile_hash changes (detected via useProfile()), clear any cached personalized content, set flag isPersonalizationStale = true
- [ ] T090 [US4] Update PersonalizeButton: if isPersonalizationStale === true, show badge "Profile updated - re-personalize" next to button

### Manual Testing

- [ ] T091 [US4] **Manual Test**: Navigate to /profile, see current answers (e.g., no RTX, cloud, beginner)
- [ ] T092 [US4] **Manual Test**: Change answers (RTX 4090, local, intermediate), click "Save Changes", verify success message, verify profile_hash changed in Network tab
- [ ] T093 [US4] **Manual Test**: Navigate to /docs/ch04-gazebo, click "Personalize this chapter", verify NEW profile settings applied (local GPU sections visible, not cloud)
- [ ] T094 [US4] **Manual Test**: Update profile again but change nothing, click "Save Changes", verify "No changes detected" message

---

## Phase 7: User Story 5 - Google OAuth Authentication (P3)

**Goal**: Users can sign up/log in with Google account as alternative to email/password

**Independent Test**: Click "Sign in with Google" on signup page, complete OAuth consent, verify onboarding quiz appears. Complete quiz, verify profile saved. Log out, click "Sign in with Google" again, verify instant login (no quiz).

**Duration**: ~5-6 hours

**Dependencies**: US2 complete (login working)

### Backend: Google OAuth Setup

- [ ] T095 [US5] Add Google OAuth environment variables to backend/.env: GOOGLE_CLIENT_ID, GOOGLE_CLIENT_SECRET
- [ ] T096 [US5] Install Google OAuth library: `pip install google-auth google-auth-oauthlib google-auth-httplib2`
- [ ] T097 [US5] Implement POST /api/auth/google/initiate endpoint in backend/app/routes/auth.py: generates Google OAuth consent URL with correct scopes (email, profile), returns {auth_url}

### Backend: Google OAuth Callback

- [ ] T098 [US5] Implement GET /api/auth/google/callback endpoint in backend/app/routes/auth.py: receives authorization code from Google
- [ ] T099 [US5] In OAuth callback: exchange code for access token with Google, fetch user info (email, google_id), check if user exists (get_user_by_google_id(db, google_id))
- [ ] T100 [US5] If user exists: create JWT token, update last_login, return {user, token, profile_completed: bool}
- [ ] T101 [US5] If new user: create user with provider='google', google_id, password_hash=NULL, create JWT, return {user, token, profile_completed: false} (frontend redirects to onboarding quiz)

### Frontend: Google OAuth Button

- [ ] T102 [US5] Create docusaurus/src/components/auth/GoogleOAuthButton.tsx: button with "Sign in with Google" text + Google logo SVG
- [ ] T103 [US5] Implement onClick handler in GoogleOAuthButton: call authClient.initiateGoogleOAuth() → fetches /api/auth/google/initiate → redirects browser to auth_url (Google consent screen)
- [ ] T104 [US5] Add GoogleOAuthButton to SignupForm.tsx (below email/password fields, separated by "or" divider)
- [ ] T105 [US5] Add GoogleOAuthButton to LoginForm.tsx (same placement)

### Frontend: OAuth Callback Handling

- [ ] T106 [US5] Create docusaurus/src/pages/auth/google-callback.tsx: receives authorization code from Google redirect (query param: ?code=...)
- [ ] T107 [US5] In google-callback page: call authClient.handleGoogleCallback(code) → calls backend /api/auth/google/callback → stores JWT in localStorage
- [ ] T108 [US5] After callback success: if profile_completed === false, redirect to /onboarding (quiz), else redirect to /docs/ch01-intro

### Manual Testing

- [ ] T109 [US5] **Manual Test**: Click "Sign in with Google" on /signup, complete Google OAuth consent screen (allow access to email, profile)
- [ ] T110 [US5] **Manual Test**: Verify redirect to /onboarding quiz, complete 7 questions, submit, verify redirect to Chapter 1
- [ ] T111 [US5] **Manual Test**: Log out, click "Sign in with Google" again, verify instant login (no quiz), redirect to Chapter 1
- [ ] T112 [US5] **Manual Test**: Query database: `psql $NEON_DATABASE_URL -c "SELECT email, provider, google_id FROM users WHERE provider='google';"` (verify OAuth user created with google_id)

---

## Phase 8: Polish & Cross-Cutting Concerns

**Goal**: Final touches, error handling, deployment preparation

**Duration**: ~3-4 hours

**Dependencies**: All user stories complete

### Error Handling

- [ ] T113 [P] Add error boundary in docusaurus/src/theme/Root.tsx: catch React errors, display friendly message "Something went wrong. Please refresh."
- [ ] T114 [P] Implement rate limiting in FastAPI: add middleware to limit POST /api/auth/sign-in to 5 attempts per IP per 15 minutes (prevent brute-force)
- [ ] T115 [P] Add database connection fallback in AuthContext: if GET /api/auth/me returns 503 (database unavailable), show warning "Using cached profile (offline mode)", load profile from localStorage backup

### Production Preparation

- [ ] T116 Build FastAPI for production: add Dockerfile in backend/ with python:3.11-slim, uvicorn with gunicorn workers
- [ ] T117 [P] Build Docusaurus for production: `cd docusaurus && npm run build` (verify static site generated in build/ folder)
- [ ] T118 [P] Configure CORS for production: update backend/app/main.py CORS middleware to allow production frontend URL (e.g., https://physical-ai-textbook.github.io)

### Documentation

- [ ] T119 [P] Update README.md: add "Authentication & Personalization" section documenting signup flow, 7 questions, FastAPI endpoints
- [ ] T120 [P] Create deployment guide in specs/005-better-auth-fastapi/DEPLOYMENT.md: document environment variables, database migration steps, FastAPI + Docusaurus deployment to Vercel/Netlify/GitHub Pages

### Validation

- [ ] T121 **E2E Test**: Complete flow from scratch: signup → answer 7 questions → log out → log in → personalize chapter → update profile → re-personalize → verify new settings applied

---

## Dependency Graph

### User Story Completion Order

```
Phase 1 (Setup) → Phase 2 (Foundational)
                        ↓
                  US1 (Registration + 7 Questions)
                        ↓
                  US2 (Authentication + Session)
                        ↓
                  US3 (Personalization)
                        ↓
         ┌──────────────┴──────────────┐
         ↓                             ↓
    US4 (Profile Dashboard)     US5 (Google OAuth)
         └──────────────┬──────────────┘
                        ↓
                Phase 8 (Polish)
```

### Critical Path

**Phase 1 → Phase 2 → US1 → US2 → US3** (MVP delivery)

All other stories (US4, US5) can proceed in parallel after US3 completes.

### Blocking Tasks

- **T010** (Database migration) blocks all user stories (no database = no auth)
- **T022** (get_current_user dependency) blocks US2 endpoints (session validation)
- **T042** (AuthProvider wrapper) blocks all frontend auth functionality
- **T069** (PersonalizeButton + useProfile integration) blocks US3 completion

---

## Parallel Execution Examples

### Phase 1 - Setup (High Parallelism)

**Sequential**:
- T001 (pip install python-jose passlib) must complete first

**Parallel Group 1** (after T001):
- T002 (pip install sqlalchemy psycopg alembic) [P]
- T003 (verify FastAPI runs) [P]
- T004 (npm install @better-auth/react) [P]
- T005 (verify Docusaurus builds) [P]

**Parallel Group 2** (database schema):
- T006-T010 can run sequentially (migration file creation → define tables → run migration)

### Phase 2 - Foundational (Moderate Parallelism)

**Sequential**:
- T013 (User model) must complete first

**Parallel Group 1** (after T013):
- T014 (UserProfile model) [P]
- T015 (database.py engine setup) [P]

**Parallel Group 2** (schemas):
- T016-T018 (Pydantic schemas) can run in parallel after models complete

**Parallel Group 3** (auth utilities):
- T019 (password hashing) [P]
- T020 (JWT functions) [P]
- T021 (profile hash) [P]

### Phase 3 - US1 (Moderate Parallelism)

**Sequential**:
- T027-T031 (backend signup endpoint) must complete first

**Parallel Group 1** (after backend complete):
- T032-T035 (authClient.ts custom adapter) [P]
- T036-T040 (SignupForm.tsx) [P]
- T041-T042 (AuthContext.tsx) [P]

---

## Implementation Strategy

### Week 1: MVP (US1 + US2 + US3)

**Day 1-2**: Phase 1 + Phase 2 (setup + foundational)
- Install dependencies, create database schema, implement models/schemas/auth utilities

**Day 3-4**: US1 (Registration + 7 Questions)
- Backend signup endpoint, custom adapter, SignupForm with 7 questions, AuthProvider wrapper

**Day 5**: US2 (Authentication + Session)
- Login endpoint, session validation, LoginForm, logout functionality

**Day 6-7**: US3 (Personalization)
- Profile data access, profile hash computation, PersonalizeButton integration, personalization rules

### Week 2: P2/P3 Stories + Polish

**Day 1**: US4 (Profile Dashboard)
- Profile update endpoint, ProfileDashboard component, cache invalidation logic

**Day 2**: US5 (Google OAuth)
- Google OAuth setup, initiate/callback endpoints, GoogleOAuthButton component

**Day 3**: Phase 8 (Polish)
- Error handling, rate limiting, CORS configuration, production builds

**Day 4-5**: Testing & Deployment
- E2E testing, database migration on production, deploy FastAPI + Docusaurus

---

## Verification Checklist (Manual Testing)

After completing all phases, run these verification steps:

### Authentication (US1 + US2)

- [ ] Navigate to http://localhost:3000/signup
- [ ] Create account with test@example.com, password SecurePass123!
- [ ] Answer all 7 questions (RTX GPU: No, Jetson: None, Ubuntu: Beginner, ROS2: None, Sim: Cloud, Goal: Learn basics, Language: English)
- [ ] Verify redirect to Chapter 1
- [ ] Log out, log in again with same credentials
- [ ] Query database: `psql $NEON_DATABASE_URL -c "SELECT email, created_at FROM users WHERE email='test@example.com';"` (verify user exists)
- [ ] Query profile: `psql $NEON_DATABASE_URL -c "SELECT rtx_gpu, profile_hash FROM user_profiles WHERE user_id IN (SELECT id FROM users WHERE email='test@example.com');"` (verify profile exists with hash)

### Personalization (US3)

- [ ] Log in as test@example.com (no RTX, cloud preference)
- [ ] Navigate to /docs/ch04-gazebo
- [ ] Click "Personalize this chapter"
- [ ] Verify cloud setup (AWS g5.2xlarge) shown, local GPU sections hidden
- [ ] Create second user test2@example.com with RTX 4090 + local preference
- [ ] Navigate to ch04, personalize, verify local GPU instructions shown, cloud hidden

### Profile Management (US4)

- [ ] Navigate to /profile
- [ ] Change answers (RTX GPU: Yes, model: RTX 4090, Sim: Local)
- [ ] Click "Save Changes", verify success message
- [ ] Navigate to ch04, personalize, verify NEW settings applied (local GPU visible)

### Google OAuth (US5)

- [ ] Click "Sign in with Google" on signup page
- [ ] Complete OAuth consent, verify redirect to onboarding quiz
- [ ] Complete quiz, verify redirect to Chapter 1
- [ ] Log out, click "Sign in with Google" again, verify instant login (no quiz)

---

## Summary

**Total Tasks**: 121 tasks

**Task Breakdown by Phase**:
- Phase 1 (Setup): 12 tasks (~2 hours)
- Phase 2 (Foundational): 14 tasks (~3-4 hours)
- Phase 3 (US1 - Registration): 19 tasks (~6-8 hours)
- Phase 4 (US2 - Authentication): 19 tasks (~4-5 hours)
- Phase 5 (US3 - Personalization): 17 tasks (~6-8 hours)
- Phase 6 (US4 - Profile Dashboard): 13 tasks (~4-5 hours)
- Phase 7 (US5 - Google OAuth): 18 tasks (~5-6 hours)
- Phase 8 (Polish): 9 tasks (~3-4 hours)

**Parallel Opportunities**: 45 tasks marked [P] (37% parallelizable)

**MVP Scope**: Phase 1 + Phase 2 + US1 + US2 + US3 = 81 tasks (~30-40 hours)

**Independent Tests**: Each user story has clear independent test criteria, enabling separate implementation and verification.

**Ready for Implementation**: All tasks follow strict checklist format with Task IDs, Story labels, file paths, and clear descriptions. Immediately executable by LLM or human developer.

---

**Tasks Status**: ✅ Complete and ready for execution
**Next Step**: Begin Phase 1 (Setup) or run `/sp.implement` to start automated task execution
