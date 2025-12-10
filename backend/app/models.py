"""
SQLAlchemy models for authentication and user profiles
"""
from sqlalchemy import Column, String, DateTime, Integer, ForeignKey, Boolean, JSON
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from datetime import datetime
import uuid

from .database import Base


class User(Base):
    """User authentication model"""
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String(255), unique=True, nullable=False, index=True)
    password_hash = Column(String(255), nullable=True)  # Nullable for OAuth users
    provider = Column(String(50), nullable=False, default='email')  # 'email' or 'google'
    google_id = Column(String(255), unique=True, nullable=True, index=True)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    last_login = Column(DateTime, nullable=True)

    # Relationship to profile
    profile = relationship("UserProfile", back_populates="user", uselist=False, cascade="all, delete-orphan")

    def __repr__(self):
        return f"<User(id={self.id}, email={self.email}, provider={self.provider})>"


class UserProfile(Base):
    """User profile with 7 background questions"""
    __tablename__ = "user_profiles"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey('users.id', ondelete='CASCADE'), unique=True, nullable=False, index=True)

    # 7 background questions (using user's original questions from spec)
    rtx_gpu = Column(Boolean, nullable=False, default=False)
    rtx_model = Column(String(100), nullable=True)  # e.g., "RTX 4090", "RTX 3060"
    jetson_board = Column(String(20), nullable=False, default='none')  # 'none', 'nano', 'nx', 'agx'
    ubuntu_experience = Column(String(20), nullable=False, default='beginner')  # 'beginner', 'intermediate', 'expert'
    ros2_knowledge = Column(String(20), nullable=False, default='none')  # 'none', 'basic', 'advanced'
    sim_preference = Column(String(20), nullable=False, default='cloud')  # 'cloud', 'local', 'both'
    learning_goal = Column(String(30), nullable=False, default='learn_basics')  # 'learn_basics', 'build_humanoid', 'research'
    preferred_language = Column(String(20), nullable=False, default='english')  # 'english', 'urdu'

    # Metadata
    completed_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    version = Column(Integer, default=1, nullable=False)
    profile_hash = Column(String(64), nullable=False, index=True)  # SHA-256 hex (64 chars)

    # Relationship to user
    user = relationship("User", back_populates="profile")

    def to_dict(self):
        """Convert profile to dict for JSON responses"""
        return {
            "rtx_gpu": self.rtx_gpu,
            "rtx_model": self.rtx_model,
            "jetson_board": self.jetson_board,
            "ubuntu_experience": self.ubuntu_experience,
            "ros2_knowledge": self.ros2_knowledge,
            "sim_preference": self.sim_preference,
            "learning_goal": self.learning_goal,
            "preferred_language": self.preferred_language,
            "completed_at": self.completed_at.isoformat() if self.completed_at else None,
            "version": self.version,
            "profile_hash": self.profile_hash,
        }

    def __repr__(self):
        return f"<UserProfile(user_id={self.user_id}, rtx_gpu={self.rtx_gpu}, profile_hash={self.profile_hash})>"
