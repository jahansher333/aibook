# API Specification: Better-Auth + Personalization

**Version**: 1.0.0
**Base URLs**:
- Auth Service: `http://localhost:3001` (dev), `https://auth.yoursite.com` (prod)
- FastAPI Backend: `http://localhost:8000` (dev), `https://api.yoursite.com` (prod)

## Authentication Endpoints (Better-Auth Service)

### POST /auth/signup

Create new user account with email/password.

**Request**:
```json
{
  "email": "student@university.edu",
  "password": "SecurePass123!"
}
```

**Response (201 Created)**:
```json
{
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "profile_completed": false,
  "redirect_url": "/onboarding"
}
```

**Errors**:
- `400`: Validation error (email invalid, password too short)
- `409`: Email already exists

---

### POST /auth/login

Authenticate user with email/password.

**Request**:
```json
{
  "email": "student@university.edu",
  "password": "SecurePass123!"
}
```

**Response (200 OK)**:
```json
{
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "profile_completed": true,
  "redirect_url": "/docs/ch01-intro"
}
```

**Errors**:
- `401`: Invalid credentials

---

### GET /auth/oauth/google

Initiate Google OAuth flow.

**Response (302 Redirect)**:
Redirects to Google consent screen: `https://accounts.google.com/o/oauth2/v2/auth?client_id=...`

---

### GET /auth/oauth/google/callback

Google OAuth callback (handled automatically by Better-Auth).

**Query Parameters**:
- `code`: Authorization code from Google

**Response (302 Redirect)**:
Redirects to frontend with token in cookie: `http://localhost:3000/onboarding?auth=success`

---

### POST /auth/logout

Invalidate user session.

**Headers**:
```
Authorization: Bearer <JWT>
```

**Response (204 No Content)**

---

## Profile Endpoints (Better-Auth Service)

### GET /profile

Get current user profile.

**Headers**:
```
Authorization: Bearer <JWT>
```

**Response (200 OK)**:
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "email": "student@university.edu",
  "auth_provider": "email",
  "profile_data": {
    "hardware_experience": "none",
    "gpu_access": "consumer",
    "ros2_experience": "beginner",
    "python_level": "intermediate",
    "learning_environment": "cloud_preferred",
    "hardware_budget": "economy",
    "learning_goal": "academic",
    "completed_at": "2025-12-09T12:00:00Z",
    "version": 1
  },
  "created_at": "2025-12-09T10:00:00Z",
  "last_login": "2025-12-09T11:00:00Z"
}
```

**Errors**:
- `401`: Unauthorized (invalid/expired JWT)

---

### POST /profile/quiz

Submit onboarding quiz answers.

**Headers**:
```
Authorization: Bearer <JWT>
```

**Request**:
```json
{
  "answers": {
    "hardware_experience": "none",
    "gpu_access": "consumer",
    "ros2_experience": "beginner",
    "python_level": "intermediate",
    "learning_environment": "cloud_preferred",
    "hardware_budget": "economy",
    "learning_goal": "academic"
  }
}
```

**Response (201 Created)**:
```json
{
  "profile_hash": "a3f7c2e1b4d9c8a7f6e5d4c3b2a1...",
  "version": 1,
  "redirect_url": "/docs/ch01-intro"
}
```

**Errors**:
- `400`: Validation error (missing fields, invalid enum values)
- `401`: Unauthorized

---

### PUT /profile

Update user profile (edit existing quiz answers).

**Headers**:
```
Authorization: Bearer <JWT>
```

**Request**:
```json
{
  "hardware_experience": "some",
  "gpu_access": "midrange",
  "ros2_experience": "intermediate",
  "python_level": "advanced",
  "learning_environment": "local_preferred",
  "hardware_budget": "economy",
  "learning_goal": "career_transition"
}
```

**Response (200 OK)**:
```json
{
  "profile_hash": "f3e8d2c1a9b8c7d6e5f4a3b2...",
  "version": 2,
  "message": "Profile updated successfully"
}
```

**Errors**:
- `400`: Validation error
- `429`: Rate limit exceeded (max 5 updates/hour)

---

## Client-Side Personalization (TypeScript Module, No HTTP API)

### PersonalizationEngine Interface

```typescript
// docusaurus/src/utils/personalize.ts

export interface PersonalizationEngine {
  /**
   * Apply personalization rules to chapter content
   */
  personalize(
    chapterMarkdown: string,
    userProfile: UserProfile,
    chapterId: string
  ): PersonalizedContent;

  /**
   * Compute profile hash for cache keys
   */
  computeProfileHash(userProfile: UserProfile): string;

  /**
   * Load personalization rules from YAML config
   */
  loadRules(): PersonalizationRules;

  /**
   * Check if content is personalizable (has markers)
   */
  isPersonalizable(chapterMarkdown: string): boolean;
}

// Usage:
import { personalize, computeProfileHash } from '@/utils/personalize';
import { useProfile } from '@/hooks/useProfile';

function ChapterPage({ markdown, chapterId }) {
  const { profile } = useProfile();
  const [isPersonalized, setIsPersonalized] = useState(false);

  const personalizedContent = useMemo(() => {
    if (isPersonalized && profile) {
      return personalize(markdown, profile, chapterId);
    }
    return { markdown, profile_hash: '', rules_applied: [] };
  }, [markdown, profile, chapterId, isPersonalized]);

  return (
    <div>
      <PersonalizeButton onClick={() => setIsPersonalized(!isPersonalized)} />
      <MarkdownRenderer content={personalizedContent.markdown} />
    </div>
  );
}
```

---

## JWT Structure

**Header**:
```json
{
  "alg": "HS256",
  "typ": "JWT"
}
```

**Payload**:
```json
{
  "sub": "550e8400-e29b-41d4-a716-446655440000",
  "email": "student@university.edu",
  "profile_hash": "a3f7c2e1b4d9...",
  "iat": 1702128000,
  "exp": 1702214400
}
```

**Signature**: HMACSHA256(base64UrlEncode(header) + "." + base64UrlEncode(payload), secret)

---

## Error Response Format

All errors follow this structure:

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Email address is invalid",
    "details": [
      {
        "field": "email",
        "issue": "must be valid email format"
      }
    ]
  }
}
```

**Error Codes**:
- `VALIDATION_ERROR`: Input validation failed
- `UNAUTHORIZED`: Missing or invalid JWT
- `FORBIDDEN`: Insufficient permissions
- `NOT_FOUND`: Resource not found
- `CONFLICT`: Resource already exists (e.g., email duplicate)
- `RATE_LIMIT_EXCEEDED`: Too many requests
- `INTERNAL_ERROR`: Server error

---

## Rate Limits

| Endpoint | Limit | Window |
|----------|-------|--------|
| POST /auth/signup | 5 requests | 1 hour |
| POST /auth/login | 10 requests | 5 minutes |
| PUT /profile | 5 requests | 1 hour |
| All other | 100 requests | 1 minute |

---

## Personalization Matrix

| Profile Dimension | Value | Content Transformation |
|-------------------|-------|------------------------|
| **hardware_experience** | none | Hide advanced sections, add beginner callouts |
| | some/proficient | Standard content |
| | expert | Show advanced configs, skip basics |
| **gpu_access** | none/consumer | Prioritize cloud instructions (AWS EC2) |
| | midrange | Show both cloud and local |
| | highend | Prioritize local setup, GPU optimization |
| **ros2_experience** | none/beginner | Add ROS 2 primer, define terms |
| | intermediate | Standard content |
| | advanced | Skip basics, link to advanced patterns |
| **python_level** | beginner | Add syntax explanations in comments |
| | intermediate/advanced | Standard code |
| | expert | Show Pythonic alternatives |
| **learning_environment** | cloud_only/cloud_preferred | AWS EC2 instructions first |
| | local_preferred/local_only | Local installation first |
| **hardware_budget** | none | Hide Jetson deployment, simulation-only |
| | minimal/economy | Standard hardware guides |
| | full | Include premium hardware options |
| **learning_goal** | academic | Add assessment reminders |
| | hobby | Emphasize fun projects |
| | career_transition | Highlight industry skills |
| | professional | Link to production best practices |

---

**API Specification Status**: ✅ Complete
**Endpoints Defined**: Auth (5) + Profile (3)
**Personalization Logic**: Client-side TypeScript module
**Next**: Generate quickstart.md for developer setup
