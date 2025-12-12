"""
Personalization Agent using LiteLLM with Groq API
Uses OpenAI Agents SDK pattern with LitellmModel for Groq integration
Adapts educational content based on user profile (experience, hardware, learning goals)
"""

import hashlib
import time
from datetime import datetime, timedelta
from typing import Dict, Any, Optional, Tuple
from dataclasses import dataclass

# Import OpenAI Agents SDK components
from agents import Agent, Runner

# Import LiteLLM model extension for Agents SDK
from agents.extensions.models.litellm_model import LitellmModel

from .config import settings

# In-memory cache (for single-instance deployment)
_personalization_cache: Dict[str, Dict] = {}

CACHE_TTL_HOURS = 1
MAX_CONTENT_LENGTH = 50000


class PersonalizationError(Exception):
    """Base exception for personalization service"""
    pass


class RateLimitError(PersonalizationError):
    """Raised when API rate limit exceeded"""
    pass


class GroqAPIError(PersonalizationError):
    """Raised when Groq API returns error"""
    pass


@dataclass
class UserProfile:
    """User profile for content personalization"""
    hardware_experience: str = "none"  # none, some, experienced
    gpu_access: str = "none"  # none, cloud, local_nvidia
    ros2_knowledge: str = "none"  # none, basic, intermediate, advanced
    learning_goal: str = "academic"  # academic, hobbyist, professional, research
    python_level: str = "beginner"  # beginner, intermediate, advanced
    learning_environment: str = "cloud_only"  # cloud_only, cloud_preferred, local_only, hybrid


class PersonalizationAgent:
    """
    Content Personalization Agent using OpenAI Agents SDK with LiteLLM/Groq.

    This agent adapts robotics/AI educational content based on user's:
    - Hardware experience level
    - GPU access availability
    - ROS2 knowledge
    - Learning goals
    - Python proficiency
    - Preferred learning environment
    """

    SYSTEM_INSTRUCTIONS = """You are an expert educational content personalizer for a Physical AI & Robotics textbook.
Your task is to adapt technical content based on the learner's profile while maintaining accuracy and educational value.

OUTPUT FORMAT: You MUST output well-formatted, styled HTML for beautiful display. Use these exact HTML patterns:

CALLOUT BOXES (use these div structures with inline styles):
- Cloud alternative: <div style="background: linear-gradient(135deg, #dbeafe, #bfdbfe); border-left: 4px solid #3b82f6; padding: 1rem; border-radius: 8px; margin: 1rem 0; color: #1e40af;"><span style="font-size: 1.25rem;">☁️</span> <strong>Cloud Alternative:</strong> [content]</div>
- Beginner tip: <div style="background: linear-gradient(135deg, #fef3c7, #fde68a); border-left: 4px solid #f59e0b; padding: 1rem; border-radius: 8px; margin: 1rem 0; color: #92400e;"><span style="font-size: 1.25rem;">💡</span> <strong>Beginner Tip:</strong> [content]</div>
- Safety warning: <div style="background: linear-gradient(135deg, #fed7aa, #fdba74); border-left: 4px solid #f97316; padding: 1rem; border-radius: 8px; margin: 1rem 0; color: #9a3412;"><span style="font-size: 1.25rem;">🔧</span> <strong>Safety First:</strong> [content]</div>
- ROS2 primer: <div style="background: linear-gradient(135deg, #e0e7ff, #c7d2fe); border-left: 4px solid #6366f1; padding: 1rem; border-radius: 8px; margin: 1rem 0; color: #3730a3;"><span style="font-size: 1.25rem;">📚</span> <strong>ROS2 Primer:</strong> [content]</div>
- Common mistake: <div style="background: linear-gradient(135deg, #fee2e2, #fecaca); border-left: 4px solid #ef4444; padding: 1rem; border-radius: 8px; margin: 1rem 0; color: #991b1b;"><span style="font-size: 1.25rem;">⚠️</span> <strong>Common Mistake:</strong> [content]</div>
- Quick win: <div style="background: linear-gradient(135deg, #d1fae5, #a7f3d0); border-left: 4px solid #10b981; padding: 1rem; border-radius: 8px; margin: 1rem 0; color: #065f46;"><span style="font-size: 1.25rem;">🎯</span> <strong>Quick Win:</strong> [content]</div>

FORMATTING (with inline styles):
- Headers: <h2 style="color: #065f46; margin-top: 1.5rem; margin-bottom: 0.75rem; font-size: 1.5rem;">Title</h2>
- Paragraphs: <p style="margin: 0.75rem 0; line-height: 1.8;">text</p>
- Lists: <ul style="padding-left: 1.5rem; margin: 0.75rem 0;"><li style="margin-bottom: 0.5rem;">item</li></ul>
- Code blocks: <pre style="background-color: #1f2937; border-radius: 8px; padding: 1rem; overflow-x: auto; margin: 1rem 0;"><code style="color: #e5e7eb; font-family: 'Fira Code', monospace; font-size: 0.9rem;">[code]</code></pre>
- Inline code: <code style="background-color: #f3f4f6; padding: 0.2rem 0.4rem; border-radius: 4px; font-family: monospace; color: #1f2937;">[code]</code>
- Bold: <strong style="color: #047857;">text</strong>
- Important sections: <div style="background: linear-gradient(135deg, #fef9c3, #fef08a); padding: 1rem; border-radius: 8px; margin: 1rem 0; border: 1px solid #fde047;">[content]</div>

PERSONALIZATION STRATEGIES:

1. **For GPU Access = "none"**:
   - Replace GPU-intensive instructions with cloud alternatives (Google Colab, cloud VMs)
   - Add cloud alternative callout boxes for NVIDIA Isaac Sim, CUDA operations
   - Suggest Gazebo as a lighter simulation alternative where applicable

2. **For Learning Environment = "cloud_only" or "cloud_preferred"**:
   - Prioritize cloud setup instructions (Google Colab, AWS RoboMaker, etc.)
   - Add step-by-step cloud platform setup guides
   - Minimize or de-emphasize local installation steps

3. **For ROS2 Knowledge = "none"**:
   - Add ROS2 primer boxes explaining concepts like nodes, topics, services
   - Define technical terms inline
   - Include prerequisite reading suggestions

4. **For Python Level = "beginner"**:
   - Add detailed code comments explaining each significant line
   - Include common mistakes tips for tricky syntax
   - Provide simpler code alternatives when complex patterns are used

5. **For Hardware Experience = "none" or "some"**:
   - Add safety warning callouts for hardware sections
   - Simplify wiring/assembly instructions with extra detail
   - Include beginner tip callouts
   - Recommend simulation-first approaches before real hardware

6. **For Learning Goal = "hobbyist"**:
   - Emphasize fun, achievable projects
   - Reduce academic theory, focus on practical results
   - Add quick win sections for immediate gratification

7. **For Learning Goal = "professional"**:
   - Include industry best practices and standards
   - Add production-readiness considerations

CRITICAL RULES:
- Output valid HTML only - no markdown
- Keep ALL code blocks intact - only ADD comments, never change code logic
- Keep technical terms in English (ROS2, Isaac Sim, URDF, etc.)
- Add relevant callout boxes based on the user's profile
- If content doesn't need significant adaptation, make minimal helpful additions
- Do NOT add generic disclaimers - only specific, actionable adaptations
- Output ONLY the HTML content, no explanations or meta-commentary"""

    def __init__(self):
        """Initialize the personalization agent with LiteLLM model."""
        self.model_name = settings.LITELLM_MODEL
        self.api_key = settings.GROQ_API_KEY

        # Create the LiteLLM model for OpenAI Agents SDK
        self.litellm_model = LitellmModel(
            model=self.model_name,
            api_key=self.api_key
        )

        # Create the Agent instance
        self.agent = Agent(
            name="ContentPersonalizer",
            instructions=self.SYSTEM_INSTRUCTIONS,
            model=self.litellm_model,
        )

        print(f"[personalization_agent] Initialized PersonalizationAgent with {self.model_name}")

    def _compute_cache_key(self, content: str, profile: UserProfile) -> str:
        """Compute cache key from content hash and profile."""
        profile_str = f"{profile.hardware_experience}_{profile.gpu_access}_{profile.ros2_knowledge}_{profile.learning_goal}_{profile.python_level}_{profile.learning_environment}"
        content_hash = hashlib.sha256(content.encode()).hexdigest()[:16]
        return f"{content_hash}_{profile_str}"

    def _get_cached(self, cache_key: str) -> Optional[Dict]:
        """Get cached personalization if valid (not expired)."""
        if cache_key not in _personalization_cache:
            print(f"[personalization_agent] Cache miss")
            return None

        cache_entry = _personalization_cache[cache_key]

        # Check if expired
        expiry_time = cache_entry.get("expiryTime")
        if expiry_time and datetime.fromisoformat(expiry_time) < datetime.utcnow():
            print(f"[personalization_agent] Cache expired")
            del _personalization_cache[cache_key]
            return None

        print(f"[personalization_agent] Cache hit")
        return cache_entry

    def _set_cached(
        self,
        cache_key: str,
        personalized_content: str,
        adaptations: list,
        duration_ms: int,
    ) -> None:
        """Store personalization in cache with metadata."""
        expiry_time = (datetime.utcnow() + timedelta(hours=CACHE_TTL_HOURS)).isoformat()

        _personalization_cache[cache_key] = {
            "personalizedContent": personalized_content,
            "adaptations": adaptations,
            "timestamp": datetime.utcnow().isoformat(),
            "expiryTime": expiry_time,
            "durationMs": duration_ms,
        }

        print(f"[personalization_agent] Cached personalization (expires: {expiry_time})")

    def _extract_adaptations(self, original: str, personalized: str, profile: UserProfile) -> list:
        """Extract list of adaptations made based on profile and content changes."""
        adaptations = []

        # Check for cloud alternatives
        if profile.gpu_access == "none":
            if "cloud" in personalized.lower() or "colab" in personalized.lower():
                adaptations.append("Added cloud alternatives for GPU-intensive tasks")
            if "gazebo" in personalized.lower() and "gazebo" not in original.lower():
                adaptations.append("Suggested Gazebo as lightweight simulation alternative")

        # Check for ROS2 explanations
        if profile.ros2_knowledge == "none":
            if "primer" in personalized.lower() or "what is" in personalized.lower():
                adaptations.append("Added ROS2 concept explanations for beginners")
            if "node" in personalized.lower() and "component that" in personalized.lower():
                adaptations.append("Included ROS2 terminology definitions")

        # Check for code comments
        if profile.python_level == "beginner":
            # Count comment lines in personalized vs original
            orig_comments = original.count("# ")
            pers_comments = personalized.count("# ")
            if pers_comments > orig_comments:
                adaptations.append("Added explanatory code comments for beginners")
            if "common mistake" in personalized.lower():
                adaptations.append("Included common mistakes to avoid")

        # Check for cloud setup prioritization
        if profile.learning_environment in ["cloud_only", "cloud_preferred"]:
            adaptations.append("Prioritized cloud-based setup instructions")

        # Check for hardware safety tips
        if profile.hardware_experience in ["none", "some"]:
            if "safety" in personalized.lower() or "careful" in personalized.lower():
                adaptations.append("Added hardware safety guidelines")
            if "tip" in personalized.lower() or "beginner" in personalized.lower():
                adaptations.append("Included beginner-friendly tips")

        # Check for learning goal adaptations
        if profile.learning_goal == "hobbyist":
            if "fun" in personalized.lower() or "quick" in personalized.lower():
                adaptations.append("Emphasized practical, achievable projects")

        if profile.learning_goal == "professional":
            if "production" in personalized.lower() or "best practice" in personalized.lower():
                adaptations.append("Added professional best practices")

        # Default adaptation if nothing specific detected
        if not adaptations:
            adaptations.append("Content optimized for your learning profile")

        return adaptations

    async def personalize(
        self,
        chapter_id: str,
        content: str,
        profile: UserProfile,
        skip_cache: bool = False
    ) -> Dict[str, Any]:
        """
        Personalize content using the OpenAI Agents SDK with LiteLLM.

        Args:
            chapter_id: Chapter identifier
            content: Original content to personalize
            profile: User's learning profile
            skip_cache: Whether to bypass cache

        Returns:
            Dict with personalized_content, adaptations, duration_ms, cached

        Raises:
            PersonalizationError: On validation or processing errors
            RateLimitError: When rate limit exceeded
            GroqAPIError: When Groq API returns error
        """
        # Validate content
        if not content or len(content) < 50:
            raise PersonalizationError("Content must be at least 50 characters")

        if len(content) > MAX_CONTENT_LENGTH:
            raise PersonalizationError(
                f"Content exceeds maximum length of {MAX_CONTENT_LENGTH} characters"
            )

        cache_key = self._compute_cache_key(content, profile)

        # Check cache first (unless skipped)
        if not skip_cache:
            cached_result = self._get_cached(cache_key)
            if cached_result:
                return {
                    "personalized_content": cached_result["personalizedContent"],
                    "adaptations": cached_result["adaptations"],
                    "duration_ms": 0,
                    "cached": True,
                }

        # Build the personalization prompt with user profile
        user_prompt = f"""Personalize the following educational content for a learner with this profile:

LEARNER PROFILE:
- Hardware Experience: {profile.hardware_experience} (none/some/experienced)
- GPU Access: {profile.gpu_access} (none/cloud/local_nvidia)
- ROS2 Knowledge: {profile.ros2_knowledge} (none/basic/intermediate/advanced)
- Learning Goal: {profile.learning_goal} (academic/hobbyist/professional/research)
- Python Level: {profile.python_level} (beginner/intermediate/advanced)
- Learning Environment: {profile.learning_environment} (cloud_only/cloud_preferred/local_only/hybrid)

ORIGINAL CONTENT:
---
{content}
---

Adapt this content according to the learner's profile. Apply relevant personalization strategies based on their experience levels and preferences. Output ONLY the personalized content."""

        start_time = time.time()

        try:
            print(f"[personalization_agent] Personalizing {chapter_id} via OpenAI Agents SDK + LiteLLM ({self.model_name})...")
            print(f"[personalization_agent] Profile: {profile.learning_environment}/{profile.hardware_experience}/{profile.python_level}")

            # Run the agent using OpenAI Agents SDK Runner
            result = await Runner.run(
                self.agent,
                user_prompt
            )

            duration_ms = int((time.time() - start_time) * 1000)

            # Extract the final output from the agent result
            personalized_content = result.final_output

            if not personalized_content:
                raise PersonalizationError("Empty response from personalization agent")

            # Clean up the response
            personalized_content = personalized_content.strip()

            # Extract adaptations made
            adaptations = self._extract_adaptations(content, personalized_content, profile)

            print(f"[personalization_agent] Personalization completed in {duration_ms}ms")
            print(f"[personalization_agent] Adaptations: {adaptations}")

            # Cache the result
            self._set_cached(cache_key, personalized_content, adaptations, duration_ms)

            return {
                "personalized_content": personalized_content,
                "adaptations": adaptations,
                "duration_ms": duration_ms,
                "cached": False,
            }

        except Exception as e:
            duration_ms = int((time.time() - start_time) * 1000)
            error_str = str(e)

            # Check for rate limit errors
            if "rate" in error_str.lower() or "429" in error_str:
                print(f"[personalization_agent] Rate limit exceeded: {error_str}")
                raise RateLimitError(
                    "Groq API rate limit exceeded. Please try again in 60 seconds."
                ) from e

            # Check for API errors
            if "401" in error_str or "Unauthorized" in error_str:
                print(f"[personalization_agent] Auth error: {error_str}")
                raise GroqAPIError(
                    "Invalid Groq API key. Check GROQ_API_KEY environment variable."
                ) from e

            if "503" in error_str or "unavailable" in error_str.lower():
                print(f"[personalization_agent] Service unavailable: {error_str}")
                raise GroqAPIError(
                    "Groq API is temporarily unavailable. Please try again later."
                ) from e

            print(f"[personalization_agent] Unexpected error: {error_str}")
            raise PersonalizationError(f"Personalization failed: {error_str}") from e


# Singleton instance
_agent_instance: Optional[PersonalizationAgent] = None


def get_personalization_agent() -> PersonalizationAgent:
    """Get or create singleton personalization agent instance."""
    global _agent_instance
    if _agent_instance is None:
        _agent_instance = PersonalizationAgent()
        print("[personalization_agent] Created PersonalizationAgent instance")
    return _agent_instance


# Helper function for routes
async def personalize_content(
    chapter_id: str,
    content: str,
    profile_dict: Dict[str, str],
    skip_cache: bool = False
) -> Dict[str, Any]:
    """
    Personalize content using the Personalization Agent.

    Args:
        chapter_id: Chapter identifier
        content: Original content
        profile_dict: User profile as dictionary
        skip_cache: Whether to bypass cache

    Returns:
        Dict with personalized_content, adaptations, duration_ms, cached
    """
    agent = get_personalization_agent()

    profile = UserProfile(
        hardware_experience=profile_dict.get("hardware_experience", "none"),
        gpu_access=profile_dict.get("gpu_access", "none"),
        ros2_knowledge=profile_dict.get("ros2_knowledge", "none"),
        learning_goal=profile_dict.get("learning_goal", "academic"),
        python_level=profile_dict.get("python_level", "beginner"),
        learning_environment=profile_dict.get("learning_environment", "cloud_only"),
    )

    return await agent.personalize(chapter_id, content, profile, skip_cache)


def clear_cache() -> int:
    """Clear all cache entries. Returns number of entries cleared."""
    count = len(_personalization_cache)
    _personalization_cache.clear()
    print(f"[personalization_agent] Cleared all {count} cache entries")
    return count


def get_cache_stats() -> Dict:
    """Get cache statistics for debugging."""
    total_entries = len(_personalization_cache)

    return {
        "totalEntries": total_entries,
        "cacheKeys": list(_personalization_cache.keys())[:10],  # First 10 keys
    }
