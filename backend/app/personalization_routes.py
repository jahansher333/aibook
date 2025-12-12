"""
Personalization API routes
Handles content personalization based on user profile using PersonalizationAgent
(OpenAI Agents SDK + LiteLLM/Groq)
"""
import logging
from datetime import datetime
from typing import Dict, Any, List
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from .personalization_agent import (
    personalize_content,
    clear_cache,
    get_cache_stats,
    PersonalizationError,
    RateLimitError,
    GroqAPIError,
)
from .config import settings

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/v1", tags=["personalization"])


class UserProfile(BaseModel):
    """User profile for personalization"""
    hardware_experience: str = Field(default="none", description="Hardware experience level: none, some, experienced")
    gpu_access: str = Field(default="none", description="GPU access level: none, cloud, local_nvidia")
    ros2_knowledge: str = Field(default="none", description="ROS2 knowledge level: none, basic, intermediate, advanced")
    learning_goal: str = Field(default="academic", description="Learning goal: academic, hobbyist, professional, research")
    python_level: str = Field(default="beginner", description="Python skill level: beginner, intermediate, advanced")
    learning_environment: str = Field(default="cloud_only", description="Preferred environment: cloud_only, cloud_preferred, local_only, hybrid")


class PersonalizeRequest(BaseModel):
    """Request body for personalization endpoint"""
    content: str = Field(..., min_length=50, description="Chapter content to personalize")
    chapterId: str = Field(..., description="Chapter ID")
    userProfile: UserProfile = Field(..., description="User's profile for personalization")


class PersonalizeResponse(BaseModel):
    """Response body for personalization"""
    personalizedContent: str
    chapterId: str
    duration: int  # milliseconds
    cached: bool
    adaptations: List[str]


@router.post(
    "/personalize",
    response_model=PersonalizeResponse,
    responses={
        400: {"description": "Bad request - content too short or invalid"},
        429: {"description": "Rate limit exceeded"},
        500: {"description": "Internal server error"},
        503: {"description": "Service unavailable"},
    },
)
async def personalize_endpoint(request: PersonalizeRequest):
    """
    Personalize chapter content based on user profile using PersonalizationAgent.

    The agent uses OpenAI Agents SDK with LiteLLM/Groq to adapt content based on:
    - Hardware experience level
    - GPU access availability
    - ROS2 knowledge
    - Learning goals
    - Python proficiency
    - Preferred learning environment

    **Request body**:
    - `content`: Chapter content to personalize (min 50 characters)
    - `chapterId`: Chapter ID (e.g., "ch01")
    - `userProfile`: User's profile with hardware, experience, preferences

    **Response**:
    - `personalizedContent`: Adapted content with callouts and explanations
    - `chapterId`: Echo of chapter ID
    - `duration`: Processing time in milliseconds
    - `cached`: Whether result came from cache
    - `adaptations`: List of adaptations made (e.g., "Added cloud alternatives")

    **Example adaptations**:
    - For GPU access = "none": Adds cloud alternatives for GPU tasks
    - For Python level = "beginner": Adds explanatory code comments
    - For ROS2 knowledge = "none": Adds concept explanations
    """
    try:
        logger.info(f"[personalization] Personalizing {request.chapterId}")
        logger.info(f"[personalization] Profile: {request.userProfile.learning_environment}/{request.userProfile.hardware_experience}/{request.userProfile.python_level}")

        # Convert Pydantic model to dict for the agent
        profile_dict = {
            "hardware_experience": request.userProfile.hardware_experience,
            "gpu_access": request.userProfile.gpu_access,
            "ros2_knowledge": request.userProfile.ros2_knowledge,
            "learning_goal": request.userProfile.learning_goal,
            "python_level": request.userProfile.python_level,
            "learning_environment": request.userProfile.learning_environment,
        }

        # Call the personalization agent
        result = await personalize_content(
            chapter_id=request.chapterId,
            content=request.content,
            profile_dict=profile_dict,
        )

        logger.info(f"[personalization] Completed {request.chapterId} in {result['duration_ms']}ms (cached: {result['cached']})")
        logger.info(f"[personalization] Adaptations: {result['adaptations']}")

        return PersonalizeResponse(
            personalizedContent=result["personalized_content"],
            chapterId=request.chapterId,
            duration=result["duration_ms"],
            cached=result["cached"],
            adaptations=result["adaptations"],
        )

    except RateLimitError as e:
        logger.warning(f"[personalization] Rate limit: {str(e)}")
        raise HTTPException(
            status_code=429,
            detail=str(e)
        )

    except GroqAPIError as e:
        logger.error(f"[personalization] Groq API error: {str(e)}")
        raise HTTPException(
            status_code=503,
            detail=str(e)
        )

    except PersonalizationError as e:
        logger.error(f"[personalization] Personalization error: {str(e)}")
        raise HTTPException(
            status_code=400,
            detail=str(e)
        )

    except Exception as e:
        logger.error(f"[personalization] Unexpected error: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Personalization failed: {str(e)}"
        )


@router.get("/personalize/health")
async def personalization_health():
    """Health check for personalization service"""
    cache_stats = get_cache_stats()
    return {
        "status": "ok",
        "service": "Content Personalization Agent",
        "model": settings.LITELLM_MODEL,
        "agent": "OpenAI Agents SDK + LiteLLM",
        "cache_entries": cache_stats["totalEntries"],
        "timestamp": datetime.utcnow().isoformat()
    }


@router.delete("/personalize/cache")
async def clear_personalization_cache():
    """Clear all cached personalizations"""
    count = clear_cache()
    return {
        "status": "ok",
        "cleared_entries": count,
        "timestamp": datetime.utcnow().isoformat()
    }


@router.get("/personalize/cache/stats")
async def get_personalization_cache_stats():
    """Get cache statistics for debugging"""
    stats = get_cache_stats()
    return {
        "status": "ok",
        **stats,
        "timestamp": datetime.utcnow().isoformat()
    }
