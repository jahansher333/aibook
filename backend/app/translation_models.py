"""
Pydantic models for translation API
"""
from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime


class TranslationRequest(BaseModel):
    """Request body for translation endpoint"""
    chapterId: str = Field(..., pattern=r'^ch\d{2}$', description="Chapter ID (ch01-ch13)")
    content: str = Field(..., min_length=100, max_length=50000, description="Chapter content to translate")
    skipCache: bool = Field(False, description="Bypass cache and fetch fresh translation")


class TranslationResponse(BaseModel):
    """Response body for successful translation"""
    chapterId: str
    urduContent: str
    duration: int  # milliseconds
    cached: bool
    cacheExpiryTime: datetime
    contentHash: str


class CacheStatusResponse(BaseModel):
    """Response for cache status check"""
    chapterId: str
    isCached: bool
    cacheExpiry: Optional[datetime] = None
    age: Optional[str] = None


class ErrorResponse(BaseModel):
    """Error response body"""
    error: str  # Error code (BAD_REQUEST, RATE_LIMITED, INTERNAL_ERROR, etc.)
    message: str
    timestamp: datetime
    retryAfter: Optional[int] = None  # For rate limit responses
