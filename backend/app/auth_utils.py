"""
Authentication utilities: password hashing, JWT tokens, profile hashing
"""
from passlib.context import CryptContext
from jose import JWTError, jwt
from datetime import datetime, timedelta
from fastapi import HTTPException, Security, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.orm import Session
import hashlib
import json
import os
from dotenv import load_dotenv

from .database import get_db
from .models import User

load_dotenv()

# JWT Configuration
JWT_SECRET_KEY = os.getenv("JWT_SECRET_KEY", "your-secret-key-change-in-production-min-32-characters")
JWT_ALGORITHM = os.getenv("JWT_ALGORITHM", "HS256")
JWT_EXPIRE_DAYS = int(os.getenv("JWT_EXPIRE_DAYS", "7"))

# Password hashing context (bcrypt with cost factor 12)
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# HTTP Bearer token security
security = HTTPBearer()


# Password Functions

def hash_password(password: str) -> str:
    """Hash password using bcrypt (cost factor 12)"""
    return pwd_context.hash(password)


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify password against hash"""
    return pwd_context.verify(plain_password, hashed_password)


# JWT Functions

def create_access_token(data: dict, expires_delta: timedelta = None) -> str:
    """
    Create JWT access token

    Args:
        data: Payload data (must include 'user_id')
        expires_delta: Optional expiration time (defaults to JWT_EXPIRE_DAYS)

    Returns:
        JWT token string
    """
    to_encode = data.copy()

    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(days=JWT_EXPIRE_DAYS)

    to_encode.update({"exp": expire, "iat": datetime.utcnow()})
    encoded_jwt = jwt.encode(to_encode, JWT_SECRET_KEY, algorithm=JWT_ALGORITHM)
    return encoded_jwt


def decode_access_token(token: str) -> dict:
    """
    Decode and validate JWT token

    Args:
        token: JWT token string

    Returns:
        Decoded payload dict

    Raises:
        HTTPException: If token is invalid or expired
    """
    try:
        payload = jwt.decode(token, JWT_SECRET_KEY, algorithms=[JWT_ALGORITHM])
        return payload
    except JWTError as e:
        raise HTTPException(status_code=401, detail=f"Invalid or expired token: {str(e)}")


# Profile Hash Function

def compute_profile_hash(profile: dict) -> str:
    """
    Compute deterministic SHA-256 hash of user profile
    Same profile → same hash (ensures cache hit for personalization)

    Args:
        profile: Dict with profile fields (rtx_gpu, jetson_board, etc.)

    Returns:
        SHA-256 hex string (64 characters)
    """
    # Sort keys for deterministic hashing (omit metadata like completed_at, version)
    sorted_profile = {
        "jetson_board": profile.get("jetson_board", "none"),
        "learning_goal": profile.get("learning_goal", "learn_basics"),
        "preferred_language": profile.get("preferred_language", "english"),
        "ros2_knowledge": profile.get("ros2_knowledge", "none"),
        "rtx_gpu": profile.get("rtx_gpu", False),
        "rtx_model": profile.get("rtx_model") or None,  # Normalize empty string to None
        "sim_preference": profile.get("sim_preference", "cloud"),
        "ubuntu_experience": profile.get("ubuntu_experience", "beginner"),
    }

    # JSON stringify with sorted keys
    profile_json = json.dumps(sorted_profile, sort_keys=True)

    # SHA-256 hash
    return hashlib.sha256(profile_json.encode('utf-8')).hexdigest()


# FastAPI Dependency: Get Current User

async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Security(security),
    db: Session = Depends(get_db)
) -> User:
    """
    FastAPI dependency to get current authenticated user from JWT token

    Usage:
        @router.get("/protected")
        async def protected_route(user: User = Depends(get_current_user)):
            return {"user_id": user.id}

    Args:
        credentials: HTTP Authorization header (Bearer token)
        db: Database session

    Returns:
        User model instance

    Raises:
        HTTPException: 401 if token invalid or user not found
    """
    token = credentials.credentials
    payload = decode_access_token(token)

    user_id: str = payload.get("user_id")
    if user_id is None:
        raise HTTPException(status_code=401, detail="Invalid token: missing user_id")

    from uuid import UUID
    # Convert string user_id to UUID for database query
    try:
        user_uuid = UUID(user_id)
        user = db.query(User).filter(User.id == user_uuid).first()
    except ValueError:
        raise HTTPException(status_code=401, detail="Invalid user ID format in token")
    if user is None:
        raise HTTPException(status_code=401, detail="User not found")

    return user
