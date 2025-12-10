# Quickstart Guide: Better-Auth + Personalization Setup

**Feature**: Better-Auth + Personalization System
**Target Audience**: Developers setting up local development environment
**Est. Setup Time**: 30-45 minutes

## Prerequisites

- [x] Node.js 20+ installed (`node --version`)
- [x] Python 3.11+ installed (`python --version`)
- [x] Git installed
- [x] Neon Postgres account created (free tier: https://neon.tech)
- [x] Google Cloud Console project created (for OAuth)
- [x] Code editor (VS Code recommended)

## Step 1: Clone Repository

```bash
git clone https://github.com/your-org/physical-ai-textbook.git
cd physical-ai-textbook
git checkout 004-better-auth-personalization  # Feature branch
```

## Step 2: Database Setup (Neon Postgres)

### 2.1 Create Neon Database

1. Sign up at https://neon.tech (free tier, no credit card)
2. Create new project: "Physical AI Textbook"
3. Copy connection string:
   ```
   postgresql://user:password@ep-cool-voice-123456.us-east-2.aws.neon.tech/neondb?sslmode=require
   ```

### 2.2 Run Database Migration

```bash
# Set environment variable
export NEON_DATABASE_URL="postgresql://user:password@..."

# Run migration (creates users table with indexes)
psql $NEON_DATABASE_URL -f migrations/001_create_users_table.sql

# Verify table created
psql $NEON_DATABASE_URL -c "\d users"
```

**Expected Output**:
```
                    Table "public.users"
     Column     |           Type           | Nullable | Default
----------------+--------------------------+----------+---------
 id             | uuid                     | not null | gen_random_uuid()
 email          | character varying(255)   | not null |
 password_hash  | character varying(255)   |          |
 auth_provider  | character varying(50)    | not null | 'email'
 google_id      | character varying(255)   |          |
 profile_data   | jsonb                    |          | '{}'
 created_at     | timestamp with time zone |          | now()
 updated_at     | timestamp with time zone |          | now()
 last_login     | timestamp with time zone |          |
Indexes:
    "users_pkey" PRIMARY KEY, btree (id)
    "idx_users_email" btree (email)
    "idx_users_google_id" btree (google_id)
    "idx_users_profile_data" gin (profile_data)
```

## Step 3: Google OAuth Setup

### 3.1 Create OAuth 2.0 Credentials

1. Go to https://console.cloud.google.com/apis/credentials
2. Click "Create Credentials" → "OAuth 2.0 Client ID"
3. Application type: "Web application"
4. Name: "Physical AI Textbook (Dev)"
5. Authorized JavaScript origins:
   - `http://localhost:3000` (Docusaurus)
   - `http://localhost:3001` (Better-Auth service)
6. Authorized redirect URIs:
   - `http://localhost:3001/auth/oauth/google/callback`
7. Click "Create" and save:
   - **Client ID**: `123456789-abcdef.apps.googleusercontent.com`
   - **Client Secret**: `GOCSPX-abcdef123456...`

### 3.2 Configure OAuth Consent Screen

1. Go to "OAuth consent screen" tab
2. User Type: "External" (for testing, max 100 test users)
3. Fill in app details:
   - App name: "Physical AI Textbook"
   - User support email: your@email.com
   - Developer contact: your@email.com
4. Scopes: Add `email`, `profile` (default)
5. Test users: Add your Gmail address for testing

## Step 4: Backend Setup (Better-Auth Service)

### 4.1 Install Dependencies

```bash
cd backend/auth-service
npm install
```

### 4.2 Configure Environment Variables

```bash
# Copy example env file
cp .env.example .env

# Edit .env file
nano .env
```

**`.env` Contents**:
```env
# Neon Postgres
NEON_DATABASE_URL="postgresql://user:password@ep-cool-voice-123456.us-east-2.aws.neon.tech/neondb?sslmode=require"

# Better-Auth
BETTER_AUTH_SECRET="your-secret-key-at-least-32-chars-long"  # Generate with: openssl rand -hex 32
BETTER_AUTH_URL="http://localhost:3001"

# Google OAuth
GOOGLE_CLIENT_ID="123456789-abcdef.apps.googleusercontent.com"
GOOGLE_CLIENT_SECRET="GOCSPX-abcdef123456..."

# CORS
CORS_ORIGINS="http://localhost:3000"  # Docusaurus frontend

# Environment
NODE_ENV="development"
```

### 4.3 Start Auth Service

```bash
npm run dev
```

**Expected Output**:
```
[Better-Auth] Starting server on http://localhost:3001
[Better-Auth] Database connected: ep-cool-voice-123456.us-east-2.aws.neon.tech
[Better-Auth] OAuth providers: google
[Better-Auth] Routes registered: /auth/signup, /auth/login, /auth/logout, /profile
✓ Auth service ready!
```

**Test Endpoint**:
```bash
curl http://localhost:3001/health
# Expected: {"status":"ok","database":"connected"}
```

## Step 5: Frontend Setup (Docusaurus)

### 5.1 Install Dependencies

```bash
cd ../../docusaurus  # From auth-service directory
npm install
```

### 5.2 Configure Environment Variables

```bash
# Copy example env file
cp .env.example .env.local

# Edit .env.local
nano .env.local
```

**`.env.local` Contents**:
```env
# Auth Service URL
REACT_APP_AUTH_SERVICE_URL="http://localhost:3001"

# Google OAuth Client ID (public, safe for frontend)
REACT_APP_GOOGLE_CLIENT_ID="123456789-abcdef.apps.googleusercontent.com"
```

### 5.3 Start Docusaurus Dev Server

```bash
npm start
```

**Expected Output**:
```
[Docusaurus] Starting dev server on http://localhost:3000
[Docusaurus] Compiling client...
✓ Client compiled successfully in 5s
✓ Open http://localhost:3000
```

## Step 6: Test Authentication Flow

### 6.1 Test Signup (Email/Password)

1. Navigate to http://localhost:3000/signup
2. Enter email: `test@example.com`
3. Enter password: `SecurePass123!`
4. Click "Create Account"
5. **Expected**: Redirect to `/onboarding` with quiz

### 6.2 Test Google OAuth

1. Navigate to http://localhost:3000/login
2. Click "Sign in with Google"
3. **Expected**: Google consent screen appears
4. Select Google account (must be in test users list)
5. **Expected**: Redirect to `/onboarding` (first-time) or `/docs/ch01-intro` (returning)

### 6.3 Test Onboarding Quiz

1. After signup, you should see 7 questions:
   - Hardware Experience: Select "No experience"
   - GPU Access: Select "No GPU"
   - ROS 2 Knowledge: Select "Never used ROS"
   - Python Proficiency: Select "Beginner"
   - Learning Environment: Select "Cloud only"
   - Hardware Budget: Select "$0 (simulation only)"
   - Learning Goals: Select "Academic"
2. Click "Save & Continue"
3. **Expected**: Redirect to `/docs/ch01-intro`, profile saved

### 6.4 Verify Profile in Database

```bash
psql $NEON_DATABASE_URL -c "SELECT email, profile_data FROM users WHERE email = 'test@example.com';"
```

**Expected Output**:
```
      email       |                              profile_data
------------------+------------------------------------------------------------------------
 test@example.com | {"hardware_experience": "none", "gpu_access": "none", "ros2_experience": "none", ...}
```

## Step 7: Test Personalization

### 7.1 Navigate to Any Chapter

1. Go to http://localhost:3000/docs/ch04-gazebo
2. **Expected**: "Personalize this chapter" button visible in top-right

### 7.2 Click Personalize Button

1. Click "Personalize this chapter"
2. **Expected**: Content transforms within <500ms:
   - Cloud setup instructions shown (AWS EC2)
   - Local setup instructions hidden (no GPU)
   - Beginner callouts added ("New to Gazebo? Follow these steps...")
   - Advanced sections hidden

### 7.3 Verify Profile Hash

Open browser console (F12):
```javascript
// Check AuthContext
localStorage.getItem('profile_hash')
// Expected: "a3f7c2e1b4d9..." (64 hex chars)
```

## Step 8: Test Profile Update

### 8.1 Navigate to Profile Page

1. Go to http://localhost:3000/profile
2. **Expected**: See all 7 quiz answers pre-filled

### 8.2 Update Profile

1. Change "GPU Access" from "No GPU" to "High-end GPU"
2. Change "Learning Environment" from "Cloud only" to "Local only"
3. Click "Save Changes"
4. **Expected**: Profile updated, new hash computed

### 8.3 Re-Test Personalization

1. Go back to http://localhost:3000/docs/ch04-gazebo
2. Click "Personalize this chapter"
3. **Expected**: Content now shows local setup (not cloud)

## Troubleshooting

### Issue: "Database connection failed"

**Solution**:
- Verify `NEON_DATABASE_URL` in `.env` is correct
- Check Neon dashboard: https://console.neon.tech
- Test connection: `psql $NEON_DATABASE_URL -c "SELECT 1;"`

### Issue: "Google OAuth redirect_uri_mismatch"

**Solution**:
- Verify redirect URI in Google Console matches exactly:
  `http://localhost:3001/auth/oauth/google/callback`
- No trailing slash, correct port (3001 not 3000)
- Clear browser cookies and try again

### Issue: "Personalization not working"

**Solution**:
- Check browser console for errors (F12 → Console)
- Verify profile loaded: `console.log(useProfile())`
- Check personalization rules file exists:
  `ls docusaurus/static/personalization-rules.yaml`
- Verify rules YAML syntax: `npx js-yaml personalization-rules.yaml`

### Issue: "JWT validation failed in FastAPI"

**Solution**:
- Ensure Better-Auth public key is copied to FastAPI `.env`:
  ```bash
  cat backend/auth-service/keys/public.pem
  # Copy output to backend/.env as BETTER_AUTH_JWT_PUBLIC_KEY
  ```
- Restart FastAPI: `python -m app.main`

## Next Steps

1. **Run Tests**:
   ```bash
   # Auth service tests
   cd backend/auth-service && npm test

   # Frontend tests
   cd docusaurus && npm test
   ```

2. **Load Sample Data**:
   ```bash
   # Generate 100 sample users with random profiles
   cd backend/auth-service
   npm run seed:users -- --count 100
   ```

3. **Test Personalization Rules**:
   ```bash
   # Validate YAML syntax
   npx js-yaml docusaurus/static/personalization-rules.yaml

   # Test rule matching
   npm run test:personalization
   ```

4. **Deploy to Staging**:
   - See `docs/deployment.md` for Vercel/Railway deployment instructions

## Development Workflow

### Adding New Personalization Rules

1. Edit `docusaurus/static/personalization-rules.yaml`
2. Add new rule:
   ```yaml
   - id: "my-new-rule"
     condition:
       hardware_budget: ["full"]
     transformations:
       show_sections: ["premium-hardware-options"]
   ```
3. Validate YAML: `npx js-yaml personalization-rules.yaml`
4. Test in browser: Refresh page, click "Personalize"

### Modifying Quiz Questions

**⚠️ WARNING**: Changing quiz structure requires profile version increment!

1. Update `contracts/types.ts` with new question
2. Update `UserProfileSchema` in `backend/auth-service/src/validation.ts`
3. Increment `QUIZ_VERSION` constant (e.g., 1 → 2)
4. Update default profile logic to handle v1 profiles
5. Run migration to add version field if needed

---

**Quickstart Guide Status**: ✅ Complete
**Setup Time**: ~30-45 minutes
**Dependencies**: Neon, Google Cloud, Node.js, Python
**Next**: Run `/sp.tasks` to generate implementation tasks
