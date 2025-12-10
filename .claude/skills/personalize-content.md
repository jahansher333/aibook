# Personalize Content Skill

**Skill Type**: Content Personalization
**Model**: groq/llama-3.3-70b-versatile
**Data Source**: Neon Postgres (user profiles)

## Purpose

Adapt textbook content based on user's hardware profile, experience level, and learning preferences stored in Neon database.

## How It Works

1. **Input**: Original content + user_id
2. **Load Profile**: Query Neon for user's hardware, experience, preferences
3. **Analyze Content**: Identify hardware-dependent sections
4. **Adapt Content**:
   - No RTX GPU → Add cloud alternatives
   - Beginner → Simplify explanations, add more examples
   - Advanced → Add optional deep-dive sections
5. **Output**: Personalized content variant

## User Profile Schema (Neon)

```sql
CREATE TABLE user_profiles (
    user_id UUID PRIMARY KEY,
    hardware_gpu VARCHAR(50),      -- 'none', 'rtx-2060', 'rtx-3060', 'rtx-4070', etc.
    hardware_jetson VARCHAR(50),   -- 'none', 'orin-nano', 'orin-agx'
    hardware_camera VARCHAR(50),   -- 'none', 'realsense-d435i', 'webcam'
    experience_level VARCHAR(20),  -- 'beginner', 'intermediate', 'advanced'
    preferred_simulator VARCHAR(20), -- 'gazebo', 'unity', 'isaac-sim'
    learning_pace VARCHAR(20),     -- 'fast', 'normal', 'thorough'
    created_at TIMESTAMP DEFAULT NOW()
);

-- Example user
INSERT INTO user_profiles VALUES (
    '123e4567-e89b-12d3-a456-426614174000',
    'none',                -- No RTX GPU
    'orin-nano',           -- Has Jetson Orin Nano
    'realsense-d435i',     -- Has RealSense camera
    'intermediate',        -- Intermediate experience
    'gazebo',              -- Prefers Gazebo
    'normal',              -- Normal learning pace
    NOW()
);
```

## Personalization Rules

### Hardware-Based Adaptations

**Original Content**:
```markdown
## 6.3 Running Isaac Sim

Install Isaac Sim on your RTX GPU:
```bash
./isaac-sim.sh
```
```

**Personalized for User with No GPU**:
```markdown
## 6.3 Running Isaac Sim

⚠️ **Your Hardware**: No RTX GPU detected

**Option 1: Cloud Streaming** (Recommended)
Use NVIDIA Omniverse Cloud ($1/hour, no installation needed):
1. Visit https://www.nvidia.com/en-us/omniverse/cloud/
2. Start Isaac Sim instance
3. Follow cloud-based tutorial

**Option 2: Use Gazebo Instead**
For this chapter's learning objectives, Gazebo provides 80% of the value:
```bash
gz sim empty.world
```

**Why This Matters**: Isaac Sim needs RTX GPU for physics simulation. You can learn the concepts with Gazebo and try Isaac Sim later.
```

### Experience-Based Adaptations

**Beginner**:
- Add glossary boxes for technical terms
- Include more code comments
- Provide step-by-step command explanations
- Add "Common Mistakes" sections

**Intermediate**:
- Standard content as-is
- Optional "Deep Dive" expandable sections

**Advanced**:
- Add research paper references
- Include optimization techniques
- Provide performance benchmarking tasks

## Implementation (Python)

```python
from typing import Dict, Any
import asyncpg
import os

async def get_user_profile(user_id: str) -> Dict[str, Any]:
    """Fetch user profile from Neon Postgres"""
    conn = await asyncpg.connect(os.getenv("NEON_DATABASE_URL"))

    try:
        profile = await conn.fetchrow(
            "SELECT * FROM user_profiles WHERE user_id = $1",
            user_id
        )
        return dict(profile) if profile else None
    finally:
        await conn.close()


async def personalize_content(content: str, user_id: str) -> str:
    """
    Personalize content based on user profile.

    Args:
        content: Original chapter content
        user_id: User UUID

    Returns:
        Personalized content variant
    """
    # Load user profile
    profile = await get_user_profile(user_id)

    if not profile:
        return content  # Return original if no profile

    # Build personalization context
    context = f"""
User Profile:
- GPU: {profile['hardware_gpu']}
- Jetson: {profile['hardware_jetson']}
- Camera: {profile['hardware_camera']}
- Experience: {profile['experience_level']}
- Preferred Sim: {profile['preferred_simulator']}
- Learning Pace: {profile['learning_pace']}

Adapt the content below based on this profile:
- If no RTX GPU: Add cloud alternatives for GPU-heavy tasks
- If beginner: Simplify technical terms, add more examples
- If advanced: Add optional deep-dive sections
- Adjust simulator examples to preferred_simulator when possible

Original Content:
{content}

Output: Personalized content maintaining markdown structure.
"""

    # Call Groq via LiteLLM
    response = await completion(
        model="groq/llama-3.3-70b-versatile",
        messages=[
            {
                "role": "system",
                "content": "You are a content personalization expert. Adapt technical content based on user profiles."
            },
            {
                "role": "user",
                "content": context
            }
        ],
        api_key=os.getenv("GROQ_API_KEY"),
        temperature=0.5
    )

    return response.choices[0].message.content


# FastAPI endpoint
@router.post("/skills/personalize-content")
async def personalize_content_endpoint(request: PersonalizeRequest):
    """
    Personalize content based on user profile.

    Request body:
    {
        "content": "Chapter content or section",
        "user_id": "UUID of user"
    }

    Response:
    {
        "personalized_content": "Adapted content",
        "profile_used": {...},
        "adaptations_made": ["added cloud alternative", "simplified terminology"]
    }
    """
    personalized = await personalize_content(request.content, request.user_id)

    return {
        "personalized_content": personalized,
        "profile_used": await get_user_profile(request.user_id),
        "adaptations_made": extract_adaptations(personalized)
    }
```

## Usage in Docusaurus

### Button Component

```typescript
// src/components/PersonalizeButton.tsx
import React, { useState, useContext } from 'react';
import { UserContext } from '../context/UserContext';

export default function PersonalizeButton({ content }: { content: string }) {
  const { userId } = useContext(UserContext);
  const [personalized, setPersonalized] = useState<string>('');
  const [loading, setLoading] = useState(false);

  const handlePersonalize = async () => {
    setLoading(true);
    try {
      const res = await fetch('http://localhost:8000/skills/personalize-content', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ content, user_id: userId })
      });
      const data = await res.json();
      setPersonalized(data.personalized_content);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="personalize-container">
      <button onClick={handlePersonalize} disabled={loading}>
        {loading ? 'Personalizing...' : '🎯 Personalize for My Hardware'}
      </button>
      {personalized && (
        <div className="personalized-content">
          <div dangerouslySetInnerHTML={{ __html: personalized }} />
        </div>
      )}
    </div>
  );
}
```

### In Chapter MDX

```markdown
<!-- ch06.md -->
# Chapter 6: NVIDIA Isaac Sim

[Standard content for all users...]

---

**Want this chapter adapted to your hardware?**

<PersonalizeButton content={currentChapterContent} />
```

## Reusability

**Usage Across Chapters**:
- All 13 chapters can be personalized
- Same skill, different content + user profiles
- Cached per user_id + content_hash for performance

**Personalization Scenarios**:
1. **User A** (No GPU): Gets cloud alternatives in Chapters 4, 5, 6
2. **User B** (RTX 4070): Gets full local development path
3. **User C** (Jetson only): Gets edge-focused content in all chapters
4. **User D** (Beginner): Gets simplified explanations + more examples

**Total Personalizations**: 13 chapters × N users = Infinite variants from one skill

---

**Status**: ✅ Skill complete with Neon integration
