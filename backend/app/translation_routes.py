"""
Translation API routes
Handles Urdu translation requests via FastAPI using the UrduTranslationAgent
(OpenAI Agents SDK + LiteLLM/Groq)
"""
import logging
from datetime import datetime
from fastapi import APIRouter, HTTPException, Query
from .translation_models import (
    TranslationRequest,
    TranslationResponse,
    CacheStatusResponse,
    ErrorResponse,
)
# Use the new translation agent (OpenAI Agents SDK + LiteLLM)
from .translation_agent import (
    translate_to_urdu,
    clear_cache,
    get_cache_stats,
    get_translation_cache,
    TranslationError,
    RateLimitError,
    GroqAPIError,
)

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/v1", tags=["translation"])


@router.post(
    "/translate",
    response_model=TranslationResponse,
    responses={
        400: {"model": ErrorResponse},
        429: {"model": ErrorResponse},
        500: {"model": ErrorResponse},
        503: {"model": ErrorResponse},
    },
)
async def translate_chapter(request: TranslationRequest):
    """
    Translate chapter content to Urdu

    **Request body**:
    - `chapterId`: Chapter ID (ch01-ch13)
    - `content`: Chapter HTML/Markdown content (100-50000 chars)
    - `skipCache`: (optional) Bypass cache and fetch fresh translation

    **Response**:
    - `chapterId`: Echo of requested chapter
    - `urduContent`: Translated content in Urdu
    - `duration`: Time taken in milliseconds
    - `cached`: Whether result came from cache
    - `cacheExpiryTime`: When translation expires (30 days from creation)
    - `contentHash`: SHA-256 hash of original content

    **Errors**:
    - 400: Bad request (invalid chapter ID, content too short/long)
    - 429: Rate limit exceeded (Groq API)
    - 500: Internal error
    - 503: Service unavailable (Groq API down)
    """
    try:
        # Validate chapter ID
        if not request.chapterId.startswith("ch") or not request.chapterId[2:].isdigit():
            raise HTTPException(
                status_code=400,
                detail=ErrorResponse(
                    error="INVALID_CHAPTER_ID",
                    message="Chapter ID must be in format ch01-ch13",
                    timestamp=datetime.utcnow(),
                ).model_dump(mode='json'),
            )

        chapter_num = int(request.chapterId[2:])
        if chapter_num < 1 or chapter_num > 13:
            raise HTTPException(
                status_code=400,
                detail=ErrorResponse(
                    error="INVALID_CHAPTER_ID",
                    message="Chapter number must be between 01 and 13",
                    timestamp=datetime.utcnow(),
                ).model_dump(mode='json'),
            )

        # Call translation service
        urdu_content, duration_ms, from_cache, expiry_time = await translate_to_urdu(
            request.chapterId, request.content, request.skipCache
        )

        # Compute content hash for client validation
        import hashlib
        content_hash = hashlib.sha256(request.content.encode()).hexdigest()

        return TranslationResponse(
            chapterId=request.chapterId,
            urduContent=urdu_content,
            duration=duration_ms,
            cached=from_cache,
            cacheExpiryTime=datetime.fromisoformat(expiry_time),
            contentHash=content_hash,
        )

    except RateLimitError as e:
        logger.error(f"Rate limit error: {str(e)}")
        raise HTTPException(
            status_code=429,
            detail=ErrorResponse(
                error="RATE_LIMITED",
                message=str(e),
                timestamp=datetime.utcnow(),
                retryAfter=60,
            ).model_dump(mode='json'),
        )

    except GroqAPIError as e:
        logger.error(f"Groq API error: {str(e)}")
        status_code = 503 if "unavailable" in str(e).lower() else 500
        raise HTTPException(
            status_code=status_code,
            detail=ErrorResponse(
                error="SERVICE_UNAVAILABLE" if status_code == 503 else "INTERNAL_ERROR",
                message=str(e),
                timestamp=datetime.utcnow(),
            ).model_dump(mode='json'),
        )

    except TranslationError as e:
        logger.error(f"Translation error: {str(e)}")
        raise HTTPException(
            status_code=400,
            detail=ErrorResponse(
                error="BAD_REQUEST",
                message=str(e),
                timestamp=datetime.utcnow(),
            ).model_dump(mode='json'),
        )

    except Exception as e:
        logger.error(f"Unexpected error: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=ErrorResponse(
                error="INTERNAL_ERROR",
                message="An unexpected error occurred. Please try again later.",
                timestamp=datetime.utcnow(),
            ).model_dump(mode='json'),
        )


@router.get(
    "/translate/{chapter_id}/status",
    response_model=CacheStatusResponse,
    responses={404: {"model": ErrorResponse}},
)
async def check_cache_status(chapter_id: str):
    """
    Check if a chapter has a cached translation

    **Parameters**:
    - `chapter_id`: Chapter ID (ch01-ch13)

    **Response**:
    - `chapterId`: Chapter ID
    - `isCached`: Whether translation is cached
    - `cacheExpiry`: When cache expires (if cached)
    - `age`: Human-readable age of cache
    """
    # Get cache from translation agent
    _translation_cache = get_translation_cache()

    if chapter_id not in _translation_cache:
        raise HTTPException(
            status_code=404,
            detail=ErrorResponse(
                error="NOT_FOUND",
                message=f"No cached translation for {chapter_id}",
                timestamp=datetime.utcnow(),
            ).model_dump(mode='json'),
        )

    cache_entry = _translation_cache[chapter_id]
    timestamp_str = cache_entry.get("timestamp")
    timestamp = datetime.fromisoformat(timestamp_str) if timestamp_str else None

    age = None
    if timestamp:
        age_delta = datetime.utcnow() - timestamp
        if age_delta.total_seconds() < 60:
            age = f"{int(age_delta.total_seconds())}s"
        elif age_delta.total_seconds() < 3600:
            age = f"{int(age_delta.total_seconds() / 60)}m"
        else:
            age = f"{int(age_delta.total_seconds() / 3600)}h"

    return CacheStatusResponse(
        chapterId=chapter_id,
        isCached=True,
        cacheExpiry=datetime.fromisoformat(cache_entry.get("expiryTime")),
        age=age,
    )


@router.delete("/translate/{chapter_id}")
async def clear_translation_cache(chapter_id: str = None):
    """
    Clear cached translation for a chapter

    **Parameters**:
    - `chapter_id`: Chapter ID to clear (ch01-ch13), or leave empty to clear all

    **Response**: 204 No Content on success
    """
    cleared = clear_cache(chapter_id)
    if cleared == 0 and chapter_id:
        raise HTTPException(
            status_code=404,
            detail=ErrorResponse(
                error="NOT_FOUND",
                message=f"No cache found for {chapter_id}",
                timestamp=datetime.utcnow(),
            ).model_dump(mode='json'),
        )

    return {"message": f"Cleared {cleared} cache entries"}


@router.get("/cache/stats")
async def get_cache_statistics():
    """
    Get cache statistics (for debugging)

    **Response**:
    - `totalEntries`: Number of cached translations
    - `totalSizeKB`: Total cache size in kilobytes
    - `entries`: List of cached chapters with details
    """
    stats = get_cache_stats()
    return {
        "timestamp": datetime.utcnow(),
        "cacheStats": stats,
    }


@router.get("/health")
async def translation_health():
    """
    Health check for translation service
    """
    return {
        "status": "ok",
        "service": "Urdu Translation API",
        "model": "groq/llama-3.1-70b-instant",
        "timestamp": datetime.utcnow(),
    }
