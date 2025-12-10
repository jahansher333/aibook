"""
Translation service using LiteLLM with Groq API
Handles Urdu translation with caching support
"""
import logging
import hashlib
import time
from datetime import datetime, timedelta
from typing import Optional, Dict, Tuple
import litellm
from .config import settings

logger = logging.getLogger(__name__)

# In-memory cache (for single-instance deployment)
# For production, use Redis or Qdrant
_translation_cache: Dict[str, Dict] = {}

# Use model from config or fall back to default
GROQ_MODEL = settings.LITELLM_MODEL if hasattr(settings, 'LITELLM_MODEL') else "groq/llama-3.1-70b-instant"
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


def get_translation_prompt(content: str) -> str:
    """
    Build translation prompt with best practices
    Instructs model to preserve code blocks and technical terms
    """
    return f"""Translate the following robotics and AI educational content into natural, easy-to-read Urdu.

IMPORTANT RULES:
1. Keep technical terms in English: ROS 2, Isaac Sim, URDF, Gazebo, TF2, Nav2, SLAM, GPT, Whisper, CLIP, MoveIt, Jetson, Ubuntu, Python, Linux, Docker, C++, etc.
2. Preserve ALL code blocks and command-line examples EXACTLY AS-IS (do NOT translate code, variable names, or keywords)
3. Preserve markdown formatting: #headers, **bold**, *italics*, - lists, [links], ![images]
4. Write for Grade 10-12 reading level (clear, simple Urdu with standard vocabulary)
5. Do not translate function/variable names, library names, or filenames
6. Keep mathematical expressions, formulas, and technical notation unchanged
7. Maintain proper Urdu grammar and punctuation (علامتیں)
8. Translate "robot" as "روبوٹ", "simulation" as "نقل", "sensor" as "حسّاس", "control" as "کنٹرول"

CONTENT TO TRANSLATE:
{content}

TRANSLATED CONTENT (Urdu only, preserve all code and technical terms unchanged):"""


def compute_content_hash(content: str) -> str:
    """
    Compute SHA-256 hash of content for cache validation
    """
    return hashlib.sha256(content.encode()).hexdigest()


def get_cached_translation(
    chapter_id: str, content_hash: str
) -> Optional[Tuple[str, int]]:
    """
    Get cached translation if valid (not expired, hash matches)
    Returns: (urdu_translation, cache_age_minutes) or None
    """
    if chapter_id not in _translation_cache:
        logger.debug(f"Cache miss for {chapter_id}")
        return None

    cache_entry = _translation_cache[chapter_id]

    # Check if expired
    expiry_time = cache_entry.get("expiryTime")
    if expiry_time and datetime.fromisoformat(expiry_time) < datetime.utcnow():
        logger.info(f"Cache expired for {chapter_id}")
        del _translation_cache[chapter_id]
        return None

    # Check if content hash matches
    cached_hash = cache_entry.get("originalContentHash")
    if cached_hash != content_hash:
        logger.info(f"Content hash mismatch for {chapter_id}, invalidating cache")
        del _translation_cache[chapter_id]
        return None

    # Cache is valid
    timestamp = cache_entry.get("timestamp")
    age_minutes = (
        (datetime.utcnow() - datetime.fromisoformat(timestamp)).total_seconds() / 60
    )
    logger.info(f"Cache hit for {chapter_id} (age: {age_minutes:.1f}min)")
    return cache_entry.get("urduTranslation"), int(age_minutes)


def set_cached_translation(
    chapter_id: str,
    original_content: str,
    urdu_translation: str,
    duration_ms: int,
) -> None:
    """
    Store translation in cache with metadata
    """
    content_hash = compute_content_hash(original_content)
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

    logger.info(f"Cached translation for {chapter_id} (expires: {expiry_time})")


async def translate_to_urdu(
    chapter_id: str, content: str, skip_cache: bool = False
) -> Tuple[str, int, bool, str]:
    """
    Translate content to Urdu using Groq LiteLLM
    Returns: (urdu_translation, duration_ms, from_cache, expiry_time_iso)

    Raises:
        TranslationError: On API or processing errors
        RateLimitError: When rate limit exceeded
    """
    if not content or len(content) < 100:
        raise TranslationError("Content must be at least 100 characters")

    if len(content) > MAX_CHAPTER_LENGTH:
        raise TranslationError(
            f"Content exceeds maximum length of {MAX_CHAPTER_LENGTH} characters"
        )

    content_hash = compute_content_hash(content)

    # Check cache first (unless skipped)
    if not skip_cache:
        cached_result = get_cached_translation(chapter_id, content_hash)
        if cached_result:
            urdu_translation, _ = cached_result
            expiry_time = _translation_cache[chapter_id]["expiryTime"]
            return urdu_translation, 0, True, expiry_time

    # Call Groq API via LiteLLM
    prompt = get_translation_prompt(content)
    start_time = time.time()

    try:
        logger.info(f"Translating {chapter_id} via Groq API...")

        response = litellm.completion(
            model=GROQ_MODEL,
            messages=[
                {
                    "role": "system",
                    "content": "You are a professional technical translator. Translate educational robotics content to Urdu. Preserve code blocks and technical terms in English. Write for Grade 10-12 reading level.",
                },
                {"role": "user", "content": prompt},
            ],
            temperature=0.3,
            max_tokens=4096,
            api_key=settings.GROQ_API_KEY,
        )

        duration_ms = int((time.time() - start_time) * 1000)

        # Extract translated text
        if not response.choices or not response.choices[0].message:
            raise TranslationError("Invalid response structure from Groq API")

        urdu_translation = response.choices[0].message.content.strip()

        if not urdu_translation:
            raise TranslationError("Empty response from Groq API")

        logger.info(f"Translation completed in {duration_ms}ms")

        # Cache the result
        set_cached_translation(chapter_id, content, urdu_translation, duration_ms)

        expiry_time = _translation_cache[chapter_id]["expiryTime"]
        return urdu_translation, duration_ms, False, expiry_time

    except litellm.RateLimitError as e:
        duration_ms = int((time.time() - start_time) * 1000)
        logger.error(f"Rate limit exceeded: {str(e)}")
        raise RateLimitError(f"Groq API rate limit exceeded. Please try again in 60 seconds.") from e

    except litellm.APIError as e:
        duration_ms = int((time.time() - start_time) * 1000)
        logger.error(f"API error: {str(e)}")

        if "401" in str(e) or "Unauthorized" in str(e):
            raise GroqAPIError("Invalid Groq API key. Check GROQ_API_KEY environment variable.") from e
        elif "503" in str(e):
            raise GroqAPIError("Groq API is temporarily unavailable. Please try again later.") from e
        else:
            raise GroqAPIError(f"Groq API error: {str(e)}") from e

    except Exception as e:
        duration_ms = int((time.time() - start_time) * 1000)
        logger.error(f"Unexpected error during translation: {str(e)}")
        raise TranslationError(f"Translation failed: {str(e)}") from e


def clear_cache(chapter_id: Optional[str] = None) -> int:
    """
    Clear cache entries
    If chapter_id is None, clear all cache
    Returns: Number of entries cleared
    """
    if chapter_id:
        if chapter_id in _translation_cache:
            del _translation_cache[chapter_id]
            logger.info(f"Cleared cache for {chapter_id}")
            return 1
        return 0
    else:
        count = len(_translation_cache)
        _translation_cache.clear()
        logger.info(f"Cleared all {count} cache entries")
        return count


def get_cache_stats() -> Dict:
    """
    Get cache statistics for debugging
    """
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
