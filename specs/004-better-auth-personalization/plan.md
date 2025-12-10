# Implementation Plan: Better-Auth + Personalization System

**Branch**: `004-better-auth-personalization` | **Date**: 2025-12-09 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-better-auth-personalization/spec.md`

## Summary

Implement comprehensive authentication and personalization system for the Physical AI textbook that enables user accounts via Better-Auth (email/password + Google OAuth), captures 7-question onboarding profile, and dynamically adapts chapter content based on user's hardware/software background. System stores profiles in Neon Postgres JSONB, applies client-side personalization rules (<500ms transformation), and maintains Urdu translation compatibility with personalized content via Qdrant caching.

**Core Components**:
1. Better-Auth v0.4+ integration with Neon Postgres for authentication
2. Onboarding quiz React component capturing 7 profile dimensions
3. Client-side personalization engine with rule-based content transformation
4. Profile hash-based caching for personalized Urdu translations
5. User dashboard for profile management and settings

## Technical Context

**Language/Version**:
- **Frontend**: TypeScript 5.3+, React 18+, Docusaurus 3.x
- **Backend**: Node.js 20+ (Better-Auth service), Python 3.11+ (existing FastAPI)
- **Database**: PostgreSQL 15+ (Neon Serverless)

**Primary Dependencies**:
- `better-auth@0.4+` - Authentication library with OAuth support
- `@better-auth/postgres` - Postgres adapter for Better-Auth
- `@neondatabase/serverless` - Neon Postgres client
- `react@18+`, `react-dom@18+` - UI framework
- `crypto` (Node.js built-in) - Profile hash computation
- `zod@3.x` - Schema validation for profile data
- Existing: `qdrant-client`, `litellm`, Docusaurus ecosystem

**Storage**:
- **Neon Postgres**: Users table with JSONB profile_data, Better-Auth sessions
- **Qdrant Cloud**: Translation cache with profile-specific keys
- **Browser localStorage**: Draft quiz answers, profile cache (fallback)

**Testing**:
- **Unit**: Jest + React Testing Library (quiz components, personalization rules)
- **Integration**: Playwright E2E (auth flow, personalization pipeline)
- **Load**: k6 (10k concurrent personalization requests)

**Target Platform**:
- **Frontend**: Modern browsers (Chrome 90+, Firefox 88+, Safari 14+, Edge 90+)
- **Backend**: Node.js runtime (Vercel serverless, Railway, or standalone)
- **Mobile**: Responsive web (iOS Safari, Android Chrome)

**Project Type**: Web application (frontend + backend microservices)

**Performance Goals**:
- Auth flow: <1s signup, <500ms login (p95)
- Personalization: <500ms content transformation (p95)
- Translation cache hit: <200ms (p95)
- JSONB profile queries: <50ms (p95 on Neon free tier)
- Concurrent users: 10,000 simultaneous personalized page views

**Constraints**:
- Better-Auth requires Node.js runtime (cannot run in Python FastAPI directly)
- Neon free tier: 3 GB storage, 100 hours compute/month
- Qdrant free tier: 1 GB vectors
- Client-side personalization to avoid server load
- Must not break existing Urdu translation or RAG chatbot features

**Scale/Scope**:
- Users: 10,000+ students/educators
- Profiles: 7 questions × 4 options = 16,384 possible combinations (hash-based deduplication)
- Chapters: 13 chapters × personalization variants
- Translations: ~260 cache entries (13 chapters × 10 common profile variants × 2 languages)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Helpful and Impactful
- ✅ **Personalization maximizes educational value**: Content adapts to user's hardware (cloud vs local), experience level (beginner vs expert), and goals (academic vs hobby)
- ✅ **Onboarding quiz captures critical dimensions**: 7 questions cover hardware access, software skills, budget, and learning goals
- ✅ **Instant adaptation (<500ms)**: No page reload required, immediate feedback

### Principle II: Honest and Accurate
- ✅ **No hallucinated APIs**: Better-Auth is real library (v0.4+), Neon Postgres is production-ready
- ✅ **Accurate hardware costs referenced**: Quiz options align with constitution's hardware tiers ($0, <$500, $500-$1500, $1500+)
- ✅ **Security best practices**: bcrypt password hashing, HTTP-only cookies, CSRF protection (Better-Auth defaults)

### Principle III: Harmless and Inclusive
- ✅ **Urdu translation compatibility**: Personalized content variants are translatable, cache keys include profile hash
- ✅ **Accessible onboarding**: Quiz meets WCAG 2.1 AA (keyboard nav, screen reader labels, color contrast)
- ✅ **No forced personalization**: Users can skip quiz ("Personalize Later") and update profile anytime
- ✅ **Free tier services**: Neon (3 GB free), Qdrant (1 GB free), Google OAuth (10k req/day free)

### Principle IV: Spec-Driven and AI-Native
- ✅ **Follows Spec-Kit Plus workflow**: Spec → Plan → Tasks → Implementation
- ✅ **Architectural decisions documented**: Better-Auth architecture (Node.js microservice), client-side personalization rationale
- ⚠️ **ADR Suggestion**: "Personalization Architecture: Client-Side vs Server-Side Content Transformation" (impact: performance, alternatives: SSR vs CSR, scope: all 13 chapters)

### Principle V: Structured and Comprehensive
- ✅ **Complete user flows**: Signup → Quiz → Personalization → Profile Update
- ✅ **Database schema defined**: Users table with JSONB profile_data, GIN index for queries
- ✅ **API contracts specified**: See Phase 1 contracts/ directory
- ✅ **Error handling**: Edge cases documented (skipped quiz, OAuth conflicts, Neon unreachable)

### Principle VI: Efficient and Scalable
- ✅ **Free tier optimization**: Neon (3 GB), Qdrant (1 GB), Better-Auth (self-hosted)
- ✅ **Client-side personalization**: Reduces server load, enables 10k concurrent users
- ✅ **Profile hash caching**: SHA-256 hash deduplicates 16k possible combinations to ~10 common variants
- ✅ **Lazy loading**: Draft quiz answers saved to localStorage (offline-capable)

### Principle VII: Innovative yet Practical
- ✅ **Rule-based personalization**: Predictable, maintainable (YAML config), fast (<500ms)
- ✅ **Profile versioning**: `profile_data.version` field enables backward-compatible migrations
- ✅ **Graceful degradation**: Skipped quiz → default profile, Neon down → localStorage fallback
- ✅ **Cross-browser compatibility**: Modern browsers + mobile responsive

**Gate Result**: ✅ PASS - No violations. ADR suggested for personalization architecture decision.

## Project Structure

### Documentation (this feature)

```text
specs/004-better-auth-personalization/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output: Better-Auth patterns, personalization strategies
├── data-model.md        # Phase 1 output: ER diagram, JSONB schema, profile hash logic
├── quickstart.md        # Phase 1 output: Setup guide for developers
├── contracts/           # Phase 1 output: API specs, TypeScript types
│   ├── auth-api.yaml        # Better-Auth endpoints (signup, login, OAuth callback)
│   ├── profile-api.yaml     # Quiz submission, profile CRUD
│   ├── personalize-api.yaml # Personalization engine interface
│   └── types.ts             # Shared TypeScript types (UserProfile, PersonalizationRules)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Backend: Better-Auth microservice (NEW - Node.js)
backend/auth-service/
├── src/
│   ├── index.ts              # Express/Fastify server with Better-Auth
│   ├── auth.config.ts        # Better-Auth configuration (providers, callbacks)
│   ├── db/
│   │   ├── schema.sql        # Users table DDL, indexes
│   │   └── client.ts         # Neon Postgres connection
│   ├── middleware/
│   │   ├── validate-session.ts   # JWT validation for protected routes
│   │   └── rate-limit.ts         # Profile update rate limiting
│   ├── routes/
│   │   ├── auth.ts           # POST /signup, /login, /logout, /oauth/google
│   │   └── profile.ts        # GET/PUT /profile, POST /profile/quiz
│   └── utils/
│       ├── profile-hash.ts   # SHA-256 hash computation from profile
│       └── defaults.ts       # Default profile for skipped quiz
├── tests/
│   ├── auth.test.ts          # Auth flow tests (signup, login, OAuth)
│   ├── profile.test.ts       # Quiz submission, profile updates
│   └── integration/
│       └── e2e-auth.spec.ts  # Playwright: signup → quiz → login
├── package.json              # better-auth, @neondatabase/serverless, zod
├── tsconfig.json
└── .env.example              # NEON_DATABASE_URL, GOOGLE_CLIENT_ID/SECRET

# Backend: Existing FastAPI (MODIFIED)
backend/
├── app/
│   ├── main.py               # MODIFIED: Add auth middleware (validate Better-Auth JWT)
│   ├── config.py             # MODIFIED: Add BETTER_AUTH_JWT_PUBLIC_KEY
│   ├── middleware/
│   │   └── auth.py           # NEW: JWT validation, extract user_id from token
│   └── agent.py              # MODIFIED: Pass user profile to RAG agent for personalization
└── requirements.txt          # Add: pyjwt, cryptography (for JWT validation)

# Frontend: Docusaurus site (MODIFIED)
docusaurus/
├── src/
│   ├── components/
│   │   ├── Auth/
│   │   │   ├── SignupForm.tsx         # NEW: Email/password + Google OAuth button
│   │   │   ├── LoginForm.tsx          # NEW: Login form with Better-Auth
│   │   │   ├── OnboardingQuiz.tsx     # NEW: 7-question quiz component
│   │   │   └── ProfileDashboard.tsx   # NEW: View/edit profile
│   │   ├── Personalization/
│   │   │   ├── PersonalizeButton.tsx  # NEW: Per-chapter personalize toggle
│   │   │   ├── PersonalizationEngine.ts # NEW: Apply transformation rules
│   │   │   └── ProfileIndicator.tsx   # NEW: "Personalized for: X" banner
│   │   └── Translation/
│   │       └── TranslateButton.tsx    # MODIFIED: Include profile hash in cache key
│   ├── contexts/
│   │   ├── AuthContext.tsx            # NEW: User session, profile state
│   │   └── PersonalizationContext.tsx # NEW: Personalization state, rules
│   ├── hooks/
│   │   ├── useAuth.ts                 # NEW: Login, logout, session management
│   │   ├── useProfile.ts              # NEW: Fetch/update profile, compute hash
│   │   └── usePersonalization.ts      # NEW: Apply/reset personalization
│   ├── utils/
│   │   ├── personalize.ts             # NEW: Rule engine (YAML config → content transform)
│   │   ├── profile-hash.ts            # NEW: SHA-256 hash (matches backend logic)
│   │   └── api-client.ts              # MODIFIED: Add auth headers (JWT bearer token)
│   ├── theme/
│   │   └── Root.tsx                   # MODIFIED: Wrap app with AuthContext provider
│   └── pages/
│       ├── signup.tsx                 # NEW: Signup page
│       ├── login.tsx                  # NEW: Login page
│       ├── onboarding.tsx             # NEW: Quiz page (post-signup redirect)
│       └── profile.tsx                # NEW: Profile dashboard
├── docs/
│   ├── ch01-intro.md                  # MODIFIED: Add PersonalizeButton component
│   ├── ch02-ros2.md                   # MODIFIED: Add PersonalizeButton + personalization markers
│   └── ...                            # MODIFIED: All 13 chapters get PersonalizeButton
├── static/
│   └── personalization-rules.yaml     # NEW: Personalization rule configuration
├── docusaurus.config.js               # MODIFIED: Add auth routes, custom plugins
└── package.json                       # Add: react-hook-form, zod, @better-auth/react

# Database migrations (NEW)
migrations/
└── 001_create_users_table.sql         # Users table DDL with JSONB profile_data
```

**Structure Decision**:

This is a **web application with microservices architecture**:

1. **Better-Auth microservice** (Node.js/TypeScript): Dedicated auth service because Better-Auth requires Node.js runtime. Handles signup/login/OAuth, issues JWTs, manages sessions in Neon Postgres. Deployed separately (Vercel serverless or Railway).

2. **Existing FastAPI backend** (Python): Modified to validate Better-Auth JWTs via public key. Continues to handle RAG chatbot, Qdrant queries. Now receives user profile in requests for personalized RAG responses.

3. **Docusaurus frontend** (React/TypeScript): Modified with new auth components (signup/login forms, onboarding quiz), personalization engine, and React contexts for state management. Existing Urdu translation updated to include profile hash in cache keys.

4. **Neon Postgres**: Single database shared by Better-Auth (users, sessions) and FastAPI (chapter metadata, existing data). Connection string configured in both services.

5. **Separation rationale**:
   - Better-Auth requires Node.js → separate microservice avoids mixing Python/Node.js in single codebase
   - FastAPI backend remains Python-focused, only adds JWT validation (minimal change)
   - Docusaurus frontend acts as orchestrator, calling both auth service (login) and FastAPI (RAG, translations)

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations detected.** All constitution principles satisfied. Suggested ADR for personalization architecture is best practice, not a violation.

---

**Plan Status**: ✅ Complete - Ready for Phase 0 research and Phase 1 design artifacts generation
**Constitution Check**: ✅ PASS - No violations
**Next Commands**: Generate research.md, data-model.md, contracts/, quickstart.md
