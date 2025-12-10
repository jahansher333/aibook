# Data Model & Entity Relationships

**Feature**: Better-Auth + Personalization System
**Date**: 2025-12-09
**Phase**: 1 (Design)

## Entity-Relationship Diagram

```
┌─────────────────────────────────────┐
│            User                     │
├─────────────────────────────────────┤
│ PK  id: UUID                        │
│     email: VARCHAR(255) UNIQUE      │
│     password_hash: VARCHAR(255)?    │ ← Nullable for OAuth-only
│     auth_provider: ENUM             │ ← 'email' | 'google'
│     google_id: VARCHAR(255)? UNIQUE │
│     profile_data: JSONB             │ ← UserProfile embedded
│     created_at: TIMESTAMPTZ         │
│     updated_at: TIMESTAMPTZ         │
│     last_login: TIMESTAMPTZ?        │
└─────────────┬───────────────────────┘
              │
              │ 1:1 embedded
              ▼
┌─────────────────────────────────────┐
│       UserProfile (JSONB)           │
├─────────────────────────────────────┤
│ hardware_experience: ENUM           │ ← 7 quiz questions
│ gpu_access: ENUM                    │
│ ros2_experience: ENUM               │
│ python_level: ENUM                  │
│ learning_environment: ENUM          │
│ hardware_budget: ENUM               │
│ learning_goal: ENUM                 │
│ completed_at: TIMESTAMP             │
│ version: INTEGER                    │ ← Quiz version (1, 2, ...)
└─────────────────────────────────────┘
              │
              │ computes
              ▼
┌─────────────────────────────────────┐
│       ProfileHash                   │
│  (SHA-256 of sorted profile)        │
└─────────────┬───────────────────────┘
              │
              │ used for
              ▼
┌─────────────────────────────────────┐
│    TranslationCache (Qdrant)        │
├─────────────────────────────────────┤
│ cache_key: STRING                   │ ← ch{id}_{hash}_{lang}
│ chapter_id: STRING                  │
│ profile_hash: STRING                │
│ language: ENUM                      │ ← 'english' | 'urdu'
│ translated_text: TEXT               │
│ created_at: TIMESTAMP               │
│ ttl_days: INTEGER                   │ ← 30 days
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│     Session (Better-Auth managed)   │
├─────────────────────────────────────┤
│ PK  id: VARCHAR                     │
│ FK  user_id: UUID → User.id         │
│     token: VARCHAR                  │
│     expires_at: TIMESTAMPTZ         │
│     created_at: TIMESTAMPTZ         │
└─────────────────────────────────────┘
              │
              │ N:1
              ▼
              User
```

## Database Schemas

### Neon Postgres Tables

#### Users Table

```sql
CREATE TABLE users (
  -- Primary key
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),

  -- Authentication fields
  email VARCHAR(255) UNIQUE NOT NULL,
  password_hash VARCHAR(255), -- Nullable for OAuth-only users
  auth_provider VARCHAR(50) NOT NULL DEFAULT 'email',
  google_id VARCHAR(255) UNIQUE, -- Google OAuth sub claim

  -- Profile data (JSONB for flexibility)
  profile_data JSONB DEFAULT '{}',

  -- Metadata
  created_at TIMESTAMPTZ DEFAULT NOW(),
  updated_at TIMESTAMPTZ DEFAULT NOW(),
  last_login TIMESTAMPTZ
);

-- Indexes for fast lookups
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_google_id ON users(google_id);
CREATE INDEX idx_users_profile_data ON users USING GIN(profile_data);

-- Example JSONB profile_data structure:
-- {
--   "hardware_experience": "none",
--   "gpu_access": "consumer",
--   "ros2_experience": "beginner",
--   "python_level": "intermediate",
--   "learning_environment": "cloud_preferred",
--   "hardware_budget": "economy",
--   "learning_goal": "academic",
--   "completed_at": "2025-12-09T12:00:00Z",
--   "version": 1
-- }
```

#### Sessions Table (Better-Auth Auto-Generated)

Better-Auth automatically creates and manages this table. No manual DDL required.

**Expected Schema** (for reference):
```sql
CREATE TABLE sessions (
  id VARCHAR(255) PRIMARY KEY,
  user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
  token TEXT NOT NULL,
  expires_at TIMESTAMPTZ NOT NULL,
  created_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX idx_sessions_user_id ON sessions(user_id);
CREATE INDEX idx_sessions_token ON sessions(token);
```

### Qdrant Collection Schema

#### Translation Cache Collection

**Collection Name**: `translations`

**Point Structure**:
```python
{
  "id": "ch04_a3f7c2e1b4d9_urdu",  # Unique ID (chapter_profile_hash_language)
  "vector": [],  # Empty (no vector similarity, metadata-only)
  "payload": {
    "chapter_id": "ch04",
    "profile_hash": "a3f7c2e1b4d9...",  # SHA-256 hash (64 hex chars)
    "language": "urdu",
    "translated_text": "...",  # Full Urdu translation
    "original_length": 5000,  # Characters in original
    "translated_length": 5200,  # Characters in translation
    "created_at": "2025-12-09T12:00:00Z",
    "ttl_days": 30,  # Client-enforced TTL
    "personalization_level": "beginner",  # For analytics
    "environment_preference": "cloud"  # For analytics
  }
}
```

## TypeScript Type Definitions

### Core Types

```typescript
// contracts/types.ts

export type AuthProvider = 'email' | 'google';

// Onboarding quiz enums (7 questions)
export type HardwareExperience = 'none' | 'some' | 'proficient' | 'expert';
export type GpuAccess = 'none' | 'consumer' | 'midrange' | 'highend';
export type Ros2Experience = 'none' | 'beginner' | 'intermediate' | 'advanced';
export type PythonLevel = 'beginner' | 'intermediate' | 'advanced' | 'expert';
export type LearningEnvironment = 'cloud_only' | 'cloud_preferred' | 'local_preferred' | 'local_only';
export type HardwareBudget = 'none' | 'minimal' | 'economy' | 'full';
export type LearningGoal = 'academic' | 'hobby' | 'career_transition' | 'professional';

export interface UserProfile {
  hardware_experience: HardwareExperience;
  gpu_access: GpuAccess;
  ros2_experience: Ros2Experience;
  python_level: PythonLevel;
  learning_environment: LearningEnvironment;
  hardware_budget: HardwareBudget;
  learning_goal: LearningGoal;
  completed_at: string; // ISO 8601 timestamp
  version: number; // Quiz version (starts at 1)
}

export interface User {
  id: string; // UUID
  email: string;
  auth_provider: AuthProvider;
  google_id?: string;
  profile_data: UserProfile | null; // Null if quiz not completed
  created_at: string;
  updated_at: string;
  last_login: string | null;
}

export interface AuthResponse {
  token: string; // JWT
  user_id: string;
  profile_completed: boolean;
  redirect_url: string; // '/onboarding' or '/docs/ch01-intro'
}

export interface PersonalizedContent {
  markdown: string; // Transformed content
  profile_hash: string; // For cache keys
  rules_applied: string[]; // Rule IDs (for debugging)
  metadata: {
    is_personalized: boolean;
    personalization_level: 'beginner' | 'intermediate' | 'advanced' | 'expert';
    environment_preference: 'cloud' | 'local' | 'hybrid';
    sections_hidden: string[];
    sections_shown: string[];
  };
}
```

### Personalization Rules

```typescript
export interface PersonalizationRule {
  id: string; // Unique rule identifier
  condition: Partial<UserProfile>; // Match condition (AND logic)
  priority: number; // Higher priority wins on conflict
  transformations: {
    hide_sections?: string[]; // Section IDs to hide
    show_sections?: string[]; // Section IDs to show
    replace_text?: Record<string, string>; // Text replacements
    add_callouts?: Array<{
      type: 'tip' | 'warning' | 'info';
      text: string;
      position: 'top' | 'bottom' | 'before-section';
      section_id?: string; // If position is 'before-section'
    }>;
    default_instructions?: 'cloud' | 'local' | 'hybrid';
    simplify_jargon?: boolean;
    skip_basics?: boolean;
  };
}

export interface PersonalizationRules {
  version: string; // Rules version (e.g., '1.0.0')
  defaults: {
    show_all_sections: boolean;
    simplify_jargon: boolean;
    default_instructions: 'cloud' | 'local' | 'hybrid';
  };
  rules: PersonalizationRule[];
}
```

## Data Validation

### Zod Schemas

```typescript
import { z } from 'zod';

// User profile schema (for quiz submission)
export const UserProfileSchema = z.object({
  hardware_experience: z.enum(['none', 'some', 'proficient', 'expert']),
  gpu_access: z.enum(['none', 'consumer', 'midrange', 'highend']),
  ros2_experience: z.enum(['none', 'beginner', 'intermediate', 'advanced']),
  python_level: z.enum(['beginner', 'intermediate', 'advanced', 'expert']),
  learning_environment: z.enum(['cloud_only', 'cloud_preferred', 'local_preferred', 'local_only']),
  hardware_budget: z.enum(['none', 'minimal', 'economy', 'full']),
  learning_goal: z.enum(['academic', 'hobby', 'career_transition', 'professional']),
  completed_at: z.string().datetime(),
  version: z.number().int().min(1),
});

// Signup request schema
export const SignupRequestSchema = z.object({
  email: z.string().email(),
  password: z.string().min(8).max(100),
});

// Quiz submission schema
export const QuizSubmissionSchema = z.object({
  answers: UserProfileSchema.omit({ completed_at: true, version: true }),
});
```

## Profile Hash Algorithm

### Hash Computation

```typescript
import crypto from 'crypto';

/**
 * Compute deterministic SHA-256 hash of user profile
 * Same profile → same hash (cache hit guaranteed)
 */
export function computeProfileHash(profile: UserProfile): string {
  // Sort fields alphabetically for deterministic hashing
  const sorted = {
    gpu_access: profile.gpu_access,
    hardware_budget: profile.hardware_budget,
    hardware_experience: profile.hardware_experience,
    learning_environment: profile.learning_environment,
    learning_goal: profile.learning_goal,
    python_level: profile.python_level,
    ros2_experience: profile.ros2_experience,
    // Omit completed_at and version (not part of cache key)
  };

  const json = JSON.stringify(sorted);
  return crypto.createHash('sha256').update(json, 'utf8').digest('hex');
}

// Example:
// Input: {
//   hardware_experience: 'none',
//   gpu_access: 'consumer',
//   ros2_experience: 'beginner',
//   python_level: 'intermediate',
//   learning_environment: 'cloud_preferred',
//   hardware_budget: 'economy',
//   learning_goal: 'academic'
// }
// Output: "a3f7c2e1b4d9c8a7f6e5d4c3b2a1... " (64 hex chars)
```

### Cache Key Formats

```typescript
/**
 * Generate Qdrant cache key for personalized translation
 */
export function generateTranslationCacheKey(
  chapterId: string,
  profileHash: string,
  language: 'english' | 'urdu'
): string {
  return `ch${chapterId}_${profileHash.substring(0, 12)}_${language}`;
}

// Examples:
// ch04_a3f7c2e1b4d9_urdu  (personalized Urdu translation of Chapter 4)
// ch04_original_urdu      (original non-personalized Urdu translation)
// ch07_f3e8d2c1a9b8_english (personalized English, rarely used but supported)
```

## Database Queries

### Common Queries

```sql
-- 1. Find user by email (login)
SELECT id, email, password_hash, auth_provider, profile_data
FROM users
WHERE email = $1;

-- 2. Find user by Google ID (OAuth)
SELECT id, email, auth_provider, profile_data
FROM users
WHERE google_id = $1;

-- 3. Get user profile with hash (personalization)
SELECT id, profile_data
FROM users
WHERE id = $1;

-- 4. Update user profile (quiz submission)
UPDATE users
SET profile_data = $1, updated_at = NOW()
WHERE id = $2
RETURNING profile_data;

-- 5. Find users by profile characteristics (analytics, optional)
SELECT COUNT(*)
FROM users
WHERE profile_data @> '{"gpu_access": "none", "ros2_experience": "beginner"}';

-- 6. Get profile completion stats (analytics, optional)
SELECT
  COUNT(*) FILTER (WHERE profile_data != '{}') AS completed,
  COUNT(*) FILTER (WHERE profile_data = '{}') AS incomplete
FROM users;
```

### Performance Optimization

```sql
-- EXPLAIN ANALYZE for profile query (verify GIN index usage)
EXPLAIN ANALYZE
SELECT id, email, profile_data
FROM users
WHERE profile_data @> '{"gpu_access": "none"}';

-- Expected output:
-- Bitmap Index Scan on idx_users_profile_data (cost=12.00..16.02 rows=1)
--   Index Cond: (profile_data @> '{"gpu_access": "none"}'::jsonb)
--   Planning Time: 0.5ms
--   Execution Time: 18-25ms (p50), 40-55ms (p95)
```

## Data Flow Diagrams

### Quiz Submission Flow

```
User Completes Quiz
    ↓
Frontend Validates (Zod)
    ↓
POST /profile/quiz
    ↓
Backend Validates (Zod)
    ↓
Compute Hash (SHA-256)
    ↓
Update users.profile_data (JSONB)
    ↓
Return {profile_hash, version, redirect}
    ↓
Frontend Stores Profile in AuthContext
    ↓
Redirect to Chapter 1
```

### Personalization Flow

```
User Clicks "Personalize"
    ↓
Get Profile from AuthContext (cached)
    ↓
Compute Profile Hash (if not cached)
    ↓
Load Personalization Rules (YAML)
    ↓
Match Rules to Profile
    ↓
Apply Transformations
    ├─ Hide Sections (DOM manipulation)
    ├─ Show Sections (conditional render)
    ├─ Replace Text (string replace)
    └─ Add Callouts (inject components)
    ↓
Re-Render React Component
    ↓
User Sees Personalized Content (<500ms)
```

---

**Data Model Status**: ✅ Complete
**Entity Relationships**: Defined
**Database Schemas**: PostgreSQL + Qdrant
**TypeScript Types**: Comprehensive
**Next**: Generate API contracts and quickstart guide
