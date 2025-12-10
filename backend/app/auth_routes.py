"""
FastAPI Authentication Routes
Endpoints: POST /sign-up, POST /sign-in, GET /session, GET /me
"""
from fastapi import APIRouter, HTTPException, Depends, status, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.orm import Session
from datetime import datetime
from typing import Optional
import os
import requests
from urllib.parse import urlencode

from .database import get_db
from .models import User, UserProfile
from .schemas import (
    SignUpRequest,
    SignInRequest,
    TokenResponse,
    SessionResponse,
    UserResponse,
    UserWithProfileResponse,
    ProfileResponse,
)
from .auth_utils import (
    hash_password,
    verify_password,
    create_access_token,
    compute_profile_hash,
    get_current_user,
    decode_access_token,
)

router = APIRouter(prefix="/api/auth", tags=["authentication"])


@router.post("/sign-up", response_model=TokenResponse, status_code=status.HTTP_201_CREATED)
async def sign_up(data: SignUpRequest, db: Session = Depends(get_db)):
    """
    Register new user with email/password + optional 7 background questions

    Request body:
        - email: Valid email address
        - password: Min 8 characters
        - name: Optional display name
        - background: Optional dict with 7 questions (rtx_gpu, jetson_board, etc.)

    Returns:
        - access_token: JWT token
        - token_type: "bearer"
        - user_id: UUID string
        - profile_hash: SHA-256 hash if profile provided

    Raises:
        - 400: Email already registered
        - 422: Invalid input data
    """
    # Check if email already exists
    existing_user = db.query(User).filter(User.email == data.email).first()
    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email already registered"
        )

    # Create user
    user = User(
        email=data.email,
        password_hash=hash_password(data.password),
        provider='email',
        last_login=datetime.utcnow()
    )
    db.add(user)
    db.flush()  # Get user.id without committing

    # Create profile if background provided
    profile_hash_value: Optional[str] = None
    if data.background:
        profile_hash_value = compute_profile_hash(data.background)

        profile = UserProfile(
            user_id=user.id,
            rtx_gpu=data.background.get("rtx_gpu", False),
            rtx_model=data.background.get("rtx_model"),
            jetson_board=data.background.get("jetson_board", "none"),
            ubuntu_experience=data.background.get("ubuntu_experience", "beginner"),
            ros2_knowledge=data.background.get("ros2_knowledge", "none"),
            sim_preference=data.background.get("sim_preference", "cloud"),
            learning_goal=data.background.get("learning_goal", "learn_basics"),
            preferred_language=data.background.get("preferred_language", "english"),
            profile_hash=profile_hash_value,
        )
        db.add(profile)

    db.commit()
    db.refresh(user)

    # Generate JWT token
    access_token = create_access_token(data={"user_id": str(user.id)})

    return TokenResponse(
        access_token=access_token,
        token_type="bearer",
        user_id=str(user.id),
        profile_hash=profile_hash_value
    )


@router.post("/sign-in", response_model=TokenResponse)
async def sign_in(data: SignInRequest, db: Session = Depends(get_db)):
    """
    Authenticate user with email/password

    Request body:
        - email: User's email
        - password: User's password

    Returns:
        - access_token: JWT token
        - token_type: "bearer"
        - user_id: UUID string
        - profile_hash: SHA-256 hash if profile exists

    Raises:
        - 401: Invalid credentials
        - 422: Invalid input data
    """
    # Find user by email
    user = db.query(User).filter(User.email == data.email).first()
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password"
        )

    # Verify password (only for email provider users)
    if user.provider != 'email' or not user.password_hash:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid authentication method for this user"
        )

    if not verify_password(data.password, user.password_hash):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password"
        )

    # Update last login
    user.last_login = datetime.utcnow()
    db.commit()

    # Get profile hash if exists
    profile_hash_value: Optional[str] = None
    if user.profile:
        profile_hash_value = user.profile.profile_hash

    # Generate JWT token
    access_token = create_access_token(data={"user_id": str(user.id)})

    return TokenResponse(
        access_token=access_token,
        token_type="bearer",
        user_id=str(user.id),
        profile_hash=profile_hash_value
    )


@router.get("/session", response_model=SessionResponse)
async def get_session(
    user: User = Depends(get_current_user),
    credentials: HTTPAuthorizationCredentials = Depends(HTTPBearer()),
    db: Session = Depends(get_db)
):
    """
    Validate JWT token and return session info

    Headers:
        - Authorization: Bearer {token}

    Returns:
        - user: User data (id, email, provider, created_at, last_login)
        - profile_completed: Boolean indicating if 7 questions answered
        - expires_at: Token expiration timestamp

    Raises:
        - 401: Invalid or expired token
    """
    # Check if profile completed
    profile_completed = user.profile is not None

    # Decode token to get expiration claim
    token = credentials.credentials
    payload = decode_access_token(token)
    expires_at = datetime.utcfromtimestamp(payload.get("exp", 0))

    # Convert user model to dict and ensure id is string
    user_dict = {
        "id": str(user.id),
        "email": user.email,
        "provider": user.provider,
        "created_at": user.created_at,
        "last_login": user.last_login
    }

    return SessionResponse(
        user=UserResponse.model_validate(user_dict),
        profile_completed=profile_completed,
        expires_at=expires_at
    )


@router.get("/me", response_model=UserWithProfileResponse)
async def get_me(user: User = Depends(get_current_user), db: Session = Depends(get_db)):
    """
    Get current user profile with 7 background questions

    Headers:
        - Authorization: Bearer {token}

    Returns:
        - user: User data (id, email, provider, created_at, last_login)
        - profile: Profile data with 7 questions (if completed) or null
        - profile_completed: Boolean indicating if 7 questions answered

    Raises:
        - 401: Invalid or expired token
    """
    profile_completed = user.profile is not None

    profile_data: Optional[ProfileResponse] = None
    if user.profile:
        profile_data = ProfileResponse.model_validate(user.profile)

    # Convert user model to dict and ensure id is string
    user_dict = {
        "id": str(user.id),
        "email": user.email,
        "provider": user.provider,
        "created_at": user.created_at,
        "last_login": user.last_login
    }

    return UserWithProfileResponse(
        user=UserResponse.model_validate(user_dict),
        profile=profile_data,
        profile_completed=profile_completed
    )


# Google OAuth Endpoints
@router.post("/google/initiate")
async def initiate_google_oauth(request: Request):
    """
    Initiate Google OAuth flow by generating consent URL

    Returns:
        - auth_url: URL to redirect user for Google consent
    """
    google_client_id = os.getenv("GOOGLE_CLIENT_ID")
    google_client_secret = os.getenv("GOOGLE_CLIENT_SECRET")

    if not google_client_id or not google_client_secret:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Google OAuth not configured"
        )

    redirect_uri = f"{request.base_url}api/auth/google/callback"
    scope = "openid email profile"

    params = {
        "client_id": google_client_id,
        "redirect_uri": redirect_uri,
        "response_type": "code",
        "scope": scope,
        "access_type": "offline",
        "prompt": "consent"
    }

    auth_url = f"https://accounts.google.com/o/oauth2/auth?{urlencode(params)}"

    return {"auth_url": auth_url}


@router.get("/google/callback")
async def google_oauth_callback(
    code: str,
    db: Session = Depends(get_db)
):
    """
    Handle Google OAuth callback and return user session

    Query params:
        - code: Authorization code from Google

    Returns:
        - access_token: JWT token
        - token_type: "bearer"
        - user_id: User ID
        - profile_completed: Whether profile is completed
    """
    google_client_id = os.getenv("GOOGLE_CLIENT_ID")
    google_client_secret = os.getenv("GOOGLE_CLIENT_SECRET")

    if not google_client_id or not google_client_secret:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Google OAuth not configured"
        )

    # Exchange authorization code for access token
    token_url = "https://oauth2.googleapis.com/token"
    redirect_uri = f"http://localhost:3000/api/auth/google/callback"  # This should match your frontend URL in production

    token_data = {
        "client_id": google_client_id,
        "client_secret": google_client_secret,
        "code": code,
        "grant_type": "authorization_code",
        "redirect_uri": redirect_uri,
    }

    token_response = requests.post(token_url, data=token_data)
    token_json = token_response.json()

    if "access_token" not in token_json:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Failed to get access token from Google"
        )

    # Get user info from Google
    user_info_response = requests.get(
        "https://www.googleapis.com/oauth2/v2/userinfo",
        headers={"Authorization": f"Bearer {token_json['access_token']}"}
    )
    user_info = user_info_response.json()

    if "email" not in user_info:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not retrieve user info from Google"
        )

    # Check if user already exists
    user = db.query(User).filter(User.google_id == user_info["id"]).first()

    if not user:
        # Check if email already exists (for account linking)
        existing_user = db.query(User).filter(User.email == user_info["email"]).first()
        if existing_user:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="Email already registered. Please sign in with your existing account."
            )

        # Create new user
        user = User(
            email=user_info["email"],
            provider='google',
            google_id=user_info["id"],
            last_login=datetime.utcnow()
        )
        db.add(user)
        db.commit()
        db.refresh(user)
    else:
        # Update last login
        user.last_login = datetime.utcnow()
        db.commit()

    # Check if user has profile
    profile_completed = user.profile is not None

    # Generate JWT token
    access_token = create_access_token(data={"user_id": str(user.id)})

    return {
        "access_token": access_token,
        "token_type": "bearer",
        "user_id": str(user.id),
        "profile_completed": profile_completed
    }
