"""
Database connection and session management
"""
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, Session
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.pool import NullPool
import os
from dotenv import load_dotenv

load_dotenv()

# Get database URL from environment, fallback to local SQLite for development
NEON_DB_URL = os.getenv("NEON_DATABASE_URL", "")
DATABASE_URL = NEON_DB_URL if NEON_DB_URL and NEON_DB_URL.strip() else "sqlite:///./physical_ai_dev.db"

# Create Base instance for all models
Base = declarative_base()

# Create engine with connection pooling disabled for serverless (Neon best practice)
if DATABASE_URL.startswith("postgresql"):
    # Use NullPool for PostgreSQL/Neon as recommended
    engine = create_engine(
        DATABASE_URL,
        poolclass=NullPool,  # Recommended for Neon serverless
        echo=False,  # Set to True for SQL query logging during development
    )
else:
    # Use regular engine for SQLite
    engine = create_engine(
        DATABASE_URL,
        echo=False,  # Set to True for SQL query logging during development
    )

# Create session factory
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Initialize tables (wrapped in try-except to handle connection errors gracefully)
try:
    Base.metadata.create_all(bind=engine)
except Exception as e:
    # Log error but don't crash - database might be temporarily unavailable
    print(f"[DATABASE] Warning: Could not create tables on startup: {e}")
    print("[DATABASE] Tables will be created lazily when first needed")


def get_db() -> Session:
    """
    FastAPI dependency for database sessions
    Usage: def endpoint(db: Session = Depends(get_db))
    """
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
