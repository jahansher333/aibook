-- Migration: Create users table with JSONB profile_data for Better-Auth + Personalization
-- Created: 2025-12-09
-- Feature: 004-better-auth-personalization

-- Drop table if exists (for clean migration)
DROP TABLE IF EXISTS users CASCADE;

-- Create users table
CREATE TABLE users (
  -- Primary key
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),

  -- Authentication fields
  email VARCHAR(255) UNIQUE NOT NULL,
  password_hash VARCHAR(255), -- Nullable for OAuth-only users (Google)
  auth_provider VARCHAR(50) NOT NULL DEFAULT 'email', -- 'email' or 'google'
  google_id VARCHAR(255) UNIQUE, -- Google OAuth sub claim

  -- Profile data (7-question quiz answers stored as JSONB)
  profile_data JSONB DEFAULT '{}',

  -- Metadata
  created_at TIMESTAMPTZ DEFAULT NOW(),
  updated_at TIMESTAMPTZ DEFAULT NOW(),
  last_login TIMESTAMPTZ
);

-- Create indexes for fast lookups
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_google_id ON users(google_id);

-- Create GIN index on profile_data for fast JSONB queries
-- Enables queries like: WHERE profile_data @> '{"gpu_access": "none"}'
CREATE INDEX idx_users_profile_data ON users USING GIN(profile_data);

-- Example profile_data structure (JSONB):
-- {
--   "hardware_experience": "none" | "some" | "proficient" | "expert",
--   "gpu_access": "none" | "consumer" | "midrange" | "highend",
--   "ros2_experience": "none" | "beginner" | "intermediate" | "advanced",
--   "python_level": "beginner" | "intermediate" | "advanced" | "expert",
--   "learning_environment": "cloud_only" | "cloud_preferred" | "local_preferred" | "local_only",
--   "hardware_budget": "none" | "minimal" | "economy" | "full",
--   "learning_goal": "academic" | "hobby" | "career_transition" | "professional",
--   "completed_at": "2025-12-09T12:00:00Z",
--   "version": 1
-- }

-- Grant permissions (adjust based on your Neon database user)
-- GRANT ALL PRIVILEGES ON TABLE users TO neondb_owner;

-- Create trigger to update updated_at timestamp automatically
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ language 'plpgsql';

CREATE TRIGGER update_users_updated_at BEFORE UPDATE ON users
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- Insert sample user for testing (optional - remove in production)
-- INSERT INTO users (email, password_hash, auth_provider, profile_data)
-- VALUES (
--   'test@example.com',
--   '$2b$10$abcdefghijklmnopqrstuv', -- Placeholder bcrypt hash
--   'email',
--   '{
--     "hardware_experience": "none",
--     "gpu_access": "none",
--     "ros2_experience": "none",
--     "python_level": "beginner",
--     "learning_environment": "cloud_only",
--     "hardware_budget": "none",
--     "learning_goal": "academic",
--     "completed_at": "2025-12-09T10:00:00Z",
--     "version": 1
--   }'::jsonb
-- );

-- Verify table created
SELECT COUNT(*) as user_count FROM users;
