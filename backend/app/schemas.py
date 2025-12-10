"""
Pydantic schemas for request/response validation
"""
from pydantic import BaseModel, EmailStr, Field
from uuid import UUID
from typing import Optional
from datetime import datetime


# Authentication Schemas

class SignUpRequest(BaseModel):
    """Signup request with email, password, and optional profile"""
    email: EmailStr
    password: str = Field(min_length=8, max_length=100)
    name: Optional[str] = None
    background: Optional[dict] = None  # 7 questions as dict


class SignInRequest(BaseModel):
    """Login request"""
    email: EmailStr
    password: str


class TokenResponse(BaseModel):
    """JWT token response"""
    access_token: str
    token_type: str = "bearer"
    user_id: str
    profile_hash: Optional[str] = None


# Profile Schemas

class ProfileCreate(BaseModel):
    """Profile creation/update schema (7 background questions)"""
    rtx_gpu: bool = False
    rtx_model: Optional[str] = None
    jetson_board: str = Field(default="none", pattern="^(none|nano|nx|agx)$")
    ubuntu_experience: str = Field(default="beginner", pattern="^(beginner|intermediate|expert)$")
    ros2_knowledge: str = Field(default="none", pattern="^(none|basic|advanced)$")
    sim_preference: str = Field(default="cloud", pattern="^(cloud|local|both)$")
    learning_goal: str = Field(default="learn_basics", pattern="^(learn_basics|build_humanoid|research)$")
    preferred_language: str = Field(default="english", pattern="^(english|urdu)$")


class ProfileResponse(BaseModel):
    """Profile response"""
    rtx_gpu: bool
    rtx_model: Optional[str]
    jetson_board: str
    ubuntu_experience: str
    ros2_knowledge: str
    sim_preference: str
    learning_goal: str
    preferred_language: str
    completed_at: Optional[datetime]
    version: int
    profile_hash: str

    class Config:
        from_attributes = True


# User Schemas

class UserResponse(BaseModel):
    """User response (without sensitive data)"""
    id: str
    email: str
    provider: str
    created_at: datetime
    last_login: Optional[datetime]

    class Config:
        from_attributes = True


class UserWithProfileResponse(BaseModel):
    """Complete user data with profile"""
    user: UserResponse
    profile: Optional[ProfileResponse]
    profile_completed: bool


# Session Schema

class SessionResponse(BaseModel):
    """Session validation response"""
    user: UserResponse
    profile_completed: bool
    expires_at: datetime
