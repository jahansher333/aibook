"""
Urdu Translation Agent using LiteLLM with Groq API
Uses OpenAI Agents SDK pattern with LitellmModel for Groq integration
"""

import hashlib
import time
import asyncio
from datetime import datetime, timedelta
from typing import Dict, Any, Optional, Tuple

# Import OpenAI Agents SDK components
from agents import Agent, Runner

# Import LiteLLM model extension for Agents SDK
from agents.extensions.models.litellm_model import LitellmModel

from .config import settings

# In-memory cache (for single-instance deployment)
_translation_cache: Dict[str, Dict] = {}

CACHE_TTL_DAYS = 30
MAX_CHAPTER_LENGTH = 50000


class TranslationError(Exception):
    """Base exception for translation service"""
    pass


class RateLimitError(TranslationError):
    """Raised when API rate limit exceeded"""
    pass


class GroqAPIError(TranslationError):
    """Raised when Groq API returns error"""
    pass


class UrduTranslationAgent:
    """
    Urdu Translation Agent using OpenAI Agents SDK with LiteLLM/Groq.

    This agent translates robotics/AI educational content to Urdu
    while preserving code blocks, technical terms, and formatting.
    """

    SYSTEM_INSTRUCTIONS = """You are a professional technical translator specializing in robotics and AI education.
Your task is to translate English educational content to Urdu for Pakistani students (Grade 10-12 reading level).

CRITICAL RULES:
1. Keep ALL technical terms in English: ROS 2, Isaac Sim, URDF, Gazebo, TF2, Nav2, SLAM, GPT, Whisper, CLIP, MoveIt, Jetson, Ubuntu, Python, Linux, Docker, C++, etc.
2. Preserve ALL code blocks and command-line examples EXACTLY AS-IS - do NOT translate code, variable names, or keywords
3. Preserve ALL markdown formatting: #headers, **bold**, *italics*, - lists, [links], ![images]
4. Write clear, simple Urdu with standard vocabulary (avoid overly literary Urdu)
5. Do not translate function/variable names, library names, or filenames
6. Keep mathematical expressions, formulas, and technical notation unchanged
7. Maintain proper Urdu grammar and punctuation (علامتیں)
8. Standard translations:
   - "robot" → "روبوٹ"
   - "simulation" → "سمولیشن"
   - "sensor" → "سینسر"
   - "control" → "کنٹرول"
   - "navigation" → "نیویگیشن"
   - "localization" → "لوکلائزیشن"
   - "mapping" → "میپنگ"

OUTPUT: Return ONLY the translated Urdu content. No explanations, no prefixes like "Here is the translation:". Just the content."""

    def __init__(self):
        """Initialize the translation agent with LiteLLM model."""
        self.model_name = settings.LITELLM_MODEL
        self.api_key = settings.GROQ_API_KEY

        # Create the LiteLLM model for OpenAI Agents SDK
        self.litellm_model = LitellmModel(
            model=self.model_name,
            api_key=self.api_key
        )

        # Create the Agent instance
        self.agent = Agent(
            name="UrduTranslator",
            instructions=self.SYSTEM_INSTRUCTIONS,
            model=self.litellm_model,
        )

        print(f"[translation_agent] Initialized UrduTranslationAgent with {self.model_name}")

    def _compute_content_hash(self, content: str) -> str:
        """Compute SHA-256 hash of content for cache validation."""
        return hashlib.sha256(content.encode()).hexdigest()

    def _get_cached_translation(
        self, chapter_id: str, content_hash: str
    ) -> Optional[Tuple[str, int]]:
        """
        Get cached translation if valid (not expired, hash matches).
        Returns: (urdu_translation, cache_age_minutes) or None
        """
        if chapter_id not in _translation_cache:
            print(f"[translation_agent] Cache miss for {chapter_id}")
            return None

        cache_entry = _translation_cache[chapter_id]

        # Check if expired
        expiry_time = cache_entry.get("expiryTime")
        if expiry_time and datetime.fromisoformat(expiry_time) < datetime.utcnow():
            print(f"[translation_agent] Cache expired for {chapter_id}")
            del _translation_cache[chapter_id]
            return None

        # Check if content hash matches
        cached_hash = cache_entry.get("originalContentHash")
        if cached_hash != content_hash:
            print(f"[translation_agent] Content hash mismatch for {chapter_id}, invalidating cache")
            del _translation_cache[chapter_id]
            return None

        # Cache is valid
        timestamp = cache_entry.get("timestamp")
        age_minutes = (
            (datetime.utcnow() - datetime.fromisoformat(timestamp)).total_seconds() / 60
        )
        print(f"[translation_agent] Cache hit for {chapter_id} (age: {age_minutes:.1f}min)")
        return cache_entry.get("urduTranslation"), int(age_minutes)

    def _set_cached_translation(
        self,
        chapter_id: str,
        original_content: str,
        urdu_translation: str,
        duration_ms: int,
    ) -> None:
        """Store translation in cache with metadata."""
        content_hash = self._compute_content_hash(original_content)
        expiry_time = (datetime.utcnow() + timedelta(days=CACHE_TTL_DAYS)).isoformat()

        _translation_cache[chapter_id] = {
            "chapterId": chapter_id,
            "originalContent": original_content,
            "originalContentHash": content_hash,
            "urduTranslation": urdu_translation,
            "timestamp": datetime.utcnow().isoformat(),
            "expiryTime": expiry_time,
            "translationDuration": duration_ms,
        }

        print(f"[translation_agent] Cached translation for {chapter_id} (expires: {expiry_time})")

    async def translate(
        self,
        chapter_id: str,
        content: str,
        skip_cache: bool = False
    ) -> Dict[str, Any]:
        """
        Translate content to Urdu using the OpenAI Agents SDK with LiteLLM.

        Args:
            chapter_id: Chapter identifier (ch01-ch13)
            content: English content to translate
            skip_cache: Whether to bypass cache

        Returns:
            Dict with urdu_content, duration_ms, from_cache, expiry_time

        Raises:
            TranslationError: On validation or processing errors
            RateLimitError: When rate limit exceeded
            GroqAPIError: When Groq API returns error
        """
        # Validate content
        if not content or len(content) < 100:
            raise TranslationError("Content must be at least 100 characters")

        if len(content) > MAX_CHAPTER_LENGTH:
            raise TranslationError(
                f"Content exceeds maximum length of {MAX_CHAPTER_LENGTH} characters"
            )

        content_hash = self._compute_content_hash(content)

        # Check cache first (unless skipped)
        if not skip_cache:
            cached_result = self._get_cached_translation(chapter_id, content_hash)
            if cached_result:
                urdu_translation, _ = cached_result
                expiry_time = _translation_cache[chapter_id]["expiryTime"]
                return {
                    "urdu_content": urdu_translation,
                    "duration_ms": 0,
                    "from_cache": True,
                    "expiry_time": expiry_time,
                    "content_hash": content_hash
                }

        # Build the translation prompt
        user_prompt = f"""Translate the following robotics and AI educational content to Urdu.

CONTENT TO TRANSLATE:
---
{content}
---

Remember: Keep ALL code blocks, technical terms, and markdown formatting unchanged. Output ONLY the Urdu translation."""

        start_time = time.time()

        try:
            print(f"[translation_agent] Translating {chapter_id} via OpenAI Agents SDK + LiteLLM ({self.model_name})...")

            # Run the agent using OpenAI Agents SDK Runner
            result = await Runner.run(
                self.agent,
                user_prompt
            )

            duration_ms = int((time.time() - start_time) * 1000)

            # Extract the final output from the agent result
            urdu_translation = result.final_output

            if not urdu_translation:
                raise TranslationError("Empty response from translation agent")

            # Clean up the response (remove any leading/trailing whitespace)
            urdu_translation = urdu_translation.strip()

            print(f"[translation_agent] Translation completed in {duration_ms}ms")

            # Cache the result
            self._set_cached_translation(chapter_id, content, urdu_translation, duration_ms)

            expiry_time = _translation_cache[chapter_id]["expiryTime"]

            return {
                "urdu_content": urdu_translation,
                "duration_ms": duration_ms,
                "from_cache": False,
                "expiry_time": expiry_time,
                "content_hash": content_hash
            }

        except Exception as e:
            duration_ms = int((time.time() - start_time) * 1000)
            error_str = str(e)

            # Check for rate limit errors
            if "rate" in error_str.lower() or "429" in error_str:
                print(f"[translation_agent] Rate limit exceeded: {error_str}")
                raise RateLimitError(
                    "Groq API rate limit exceeded. Please try again in 60 seconds."
                ) from e

            # Check for API errors
            if "401" in error_str or "Unauthorized" in error_str:
                print(f"[translation_agent] Auth error: {error_str}")
                raise GroqAPIError(
                    "Invalid Groq API key. Check GROQ_API_KEY environment variable."
                ) from e

            if "503" in error_str or "unavailable" in error_str.lower():
                print(f"[translation_agent] Service unavailable: {error_str}")
                raise GroqAPIError(
                    "Groq API is temporarily unavailable. Please try again later."
                ) from e

            print(f"[translation_agent] Unexpected error: {error_str}")
            raise TranslationError(f"Translation failed: {error_str}") from e


# Singleton instance
_agent_instance: Optional[UrduTranslationAgent] = None


def get_translation_agent() -> UrduTranslationAgent:
    """Get or create singleton translation agent instance."""
    global _agent_instance
    if _agent_instance is None:
        _agent_instance = UrduTranslationAgent()
        print("[translation_agent] Created UrduTranslationAgent instance")
    return _agent_instance


# Helper functions for backwards compatibility with translation_service.py
async def translate_to_urdu(
    chapter_id: str, content: str, skip_cache: bool = False
) -> Tuple[str, int, bool, str]:
    """
    Translate content to Urdu using the Translation Agent.

    Returns: (urdu_translation, duration_ms, from_cache, expiry_time_iso)
    """
    agent = get_translation_agent()
    result = await agent.translate(chapter_id, content, skip_cache)
    return (
        result["urdu_content"],
        result["duration_ms"],
        result["from_cache"],
        result["expiry_time"]
    )


def clear_cache(chapter_id: Optional[str] = None) -> int:
    """
    Clear cache entries.
    If chapter_id is None, clear all cache.
    Returns: Number of entries cleared
    """
    if chapter_id:
        if chapter_id in _translation_cache:
            del _translation_cache[chapter_id]
            print(f"[translation_agent] Cleared cache for {chapter_id}")
            return 1
        return 0
    else:
        count = len(_translation_cache)
        _translation_cache.clear()
        print(f"[translation_agent] Cleared all {count} cache entries")
        return count


def get_cache_stats() -> Dict:
    """Get cache statistics for debugging."""
    total_entries = len(_translation_cache)
    total_size_kb = 0

    entries = []
    for chapter_id, cache_entry in _translation_cache.items():
        size_bytes = len(str(cache_entry).encode())
        total_size_kb += size_bytes / 1024
        entries.append({
            "chapterId": chapter_id,
            "sizeKB": size_bytes / 1024,
            "expiryTime": cache_entry.get("expiryTime"),
        })

    return {
        "totalEntries": total_entries,
        "totalSizeKB": round(total_size_kb, 2),
        "entries": entries,
    }


# Export cache for routes to access
def get_translation_cache() -> Dict[str, Dict]:
    """Get the translation cache dictionary."""
    return _translation_cache
