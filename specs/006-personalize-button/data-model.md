# Data Model: Personalize Button System

## Overview
Data model for the personalization system that adapts chapter content based on user profile attributes. The system uses client-side data processing with profile data retrieved from Better-Auth.

## Core Entities

### UserProfile
**Description**: User profile data containing background information collected during signup

**Fields**:
- `id` (string): Unique user identifier from Better-Auth
- `hardware_experience` (enum): User's hardware experience level
  - Values: `'none' | 'some' | 'proficient' | 'expert'`
- `gpu_access` (enum): User's GPU access level
  - Values: `'none' | 'consumer' | 'midrange' | 'highend'`
- `ros2_knowledge` (enum): User's ROS2 knowledge level
  - Values: `'none' | 'basic' | 'intermediate' | 'advanced'`
- `learning_goal` (enum): User's learning objective
  - Values: `'academic' | 'hobby' | 'career_transition' | 'professional'`
- `python_level` (enum): User's Python proficiency
  - Values: `'beginner' | 'intermediate' | 'advanced' | 'expert'`
- `learning_environment` (enum): User's preferred learning environment
  - Values: `'cloud_only' | 'cloud_preferred' | 'local_preferred' | 'local_only'`
- `profile_hash` (string): SHA-256 hash of sorted profile values for caching
- `timestamp` (string): ISO date string when profile was last updated

**Relationships**:
- One-to-one with Better-Auth user account
- Referenced by PersonalizationRule for content transformation

**Validation Rules**:
- All fields are required
- Enum values must match defined options
- profile_hash must be valid SHA-256 format

### PersonalizationRule
**Description**: Configuration rule that maps profile attributes to content transformations

**Fields**:
- `rule_id` (string): Unique identifier for the rule
- `profile_key` (string): The profile field to match against
- `profile_value` (string | string[]): The expected value(s) for the profile field
- `content_selector` (string): CSS selector for identifying content to transform
- `transformation_type` (enum): Type of transformation to apply
  - Values: `'show' | 'hide' | 'emphasize' | 'replace_text' | 'add_content'`
- `parameters` (object, optional): Additional parameters for the transformation

**Relationships**:
- References UserProfile.profile_key and profile_value for matching
- Applied to content sections identified by content_selector

**Validation Rules**:
- rule_id must be unique
- profile_key must match a valid UserProfile field
- transformation_type must be one of the defined values
- content_selector must be a valid CSS selector

### PersonalizedContentState
**Description**: Runtime state for managing personalized content in React components

**Fields**:
- `isPersonalized` (boolean): Whether the content is currently personalized
- `originalContent` (string): Cached original content for toggling back
- `transformedContent` (string): The current personalized content
- `profileSummary` (string): Human-readable summary of the applied profile
- `appliedRules` (string[]): List of rule IDs that were applied
- `loading` (boolean): Whether personalization is in progress
- `error` (string | null): Error message if personalization failed

**Relationships**:
- Derived from UserProfile and PersonalizationRule
- Used by UI components to display appropriate content

**State Transitions**:
- `initial` → `loading`: When personalization button is clicked
- `loading` → `personalized`: When transformation completes successfully
- `loading` → `error`: When transformation fails
- `personalized` → `original`: When "Show Original Content" is clicked

### ProfileCache
**Description**: Client-side caching structure for user profile data

**Fields**:
- `profile` (UserProfile): The cached profile object
- `timestamp` (number): Unix timestamp when cache was created
- `expiresAt` (number): Unix timestamp when cache expires
- `profileHash` (string): Copy of profile hash for quick comparison

**Validation Rules**:
- Cache must expire after 24 hours
- profileHash must match the stored profile's hash
- Profile must be valid according to UserProfile validation rules

## Data Flow

### Profile Retrieval Flow
1. User logs in → Better-Auth provides user ID
2. Fetch profile from API using user ID
3. Validate profile structure and values
4. Store in ProfileCache with expiration
5. Make available via React Context

### Personalization Flow
1. User clicks "Personalize" button
2. Retrieve UserProfile from cache/context
3. Load PersonalizationRules from configuration
4. Match rules against user profile
5. Apply matching transformations to content
6. Update PersonalizedContentState
7. Re-render UI with transformed content

### Content Toggle Flow
1. User clicks "Show Original Content" or "Personalize"
2. Switch between originalContent and transformedContent in state
3. Update isPersonalized flag
4. Re-render UI with appropriate content

## Validation Rules

### Profile Validation
- All profile fields must have valid enum values
- Profile hash must match the computed hash of the profile data
- Profile must contain all required fields

### Rule Validation
- Rule selectors must match actual content elements
- Transformation types must be valid
- Profile key references must match actual profile fields

### State Validation
- Personalized content must be derived from original content
- Applied rules must exist in the rule configuration
- Error states must have appropriate error messages

## Constraints

### Performance Constraints
- Profile retrieval must complete within 1 second
- Content transformation must complete within 500ms
- State updates must not cause React rendering delays >100ms

### Memory Constraints
- Profile cache must not exceed 1MB
- Personalized content state must be cleared when leaving chapter
- Rule configuration should be <100KB

### Security Constraints
- Profile data must not be exposed to unauthorized users
- Content transformations must be validated to prevent injection
- Cache keys must not expose sensitive profile information

## Relationships

### UserProfile → PersonalizationRule
- One-to-many: One profile can match multiple rules
- Profile values determine which rules are applied

### PersonalizationRule → PersonalizedContentState
- One-to-many: One rule can affect multiple content elements
- Rules determine the transformations applied to content

### ProfileCache → UserProfile
- One-to-one: Cache stores one profile object
- Provides fast access to profile data