"""
Database Migration Script for Authentication Tables
Creates users and user_profiles tables in Neon Postgres

Run with: python -m backend.migrations.001_create_auth_tables
"""
import os
import sys
from pathlib import Path

# Add backend to path
backend_dir = Path(__file__).parent.parent
sys.path.insert(0, str(backend_dir))

from app.database import engine
from app.models import Base, User, UserProfile


def create_tables():
    """Create all authentication tables"""
    print("Creating authentication tables...")

    try:
        # Create all tables defined in models
        Base.metadata.create_all(bind=engine)
        print("✅ Successfully created tables:")
        print("  - users")
        print("  - user_profiles")
        print("\nTable structure:")
        print("\nusers table:")
        print("  - id (UUID, primary key)")
        print("  - email (VARCHAR(255), unique, indexed)")
        print("  - password_hash (VARCHAR(255), nullable)")
        print("  - provider (VARCHAR(50), default='email')")
        print("  - google_id (VARCHAR(255), unique, indexed, nullable)")
        print("  - created_at (TIMESTAMP)")
        print("  - last_login (TIMESTAMP, nullable)")
        print("\nuser_profiles table:")
        print("  - id (UUID, primary key)")
        print("  - user_id (UUID, foreign key -> users.id, unique, indexed)")
        print("  - rtx_gpu (BOOLEAN, default=False)")
        print("  - rtx_model (VARCHAR(100), nullable)")
        print("  - jetson_board (VARCHAR(20), default='none')")
        print("  - ubuntu_experience (VARCHAR(20), default='beginner')")
        print("  - ros2_knowledge (VARCHAR(20), default='none')")
        print("  - sim_preference (VARCHAR(20), default='cloud')")
        print("  - learning_goal (VARCHAR(30), default='learn_basics')")
        print("  - preferred_language (VARCHAR(20), default='english')")
        print("  - completed_at (TIMESTAMP)")
        print("  - version (INTEGER, default=1)")
        print("  - profile_hash (VARCHAR(64), indexed)")

    except Exception as e:
        print(f"❌ Error creating tables: {e}")
        raise


def drop_tables():
    """Drop all authentication tables (use with caution!)"""
    print("⚠️  WARNING: Dropping all authentication tables...")

    try:
        Base.metadata.drop_all(bind=engine)
        print("✅ Successfully dropped tables")
    except Exception as e:
        print(f"❌ Error dropping tables: {e}")
        raise


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Manage authentication database tables")
    parser.add_argument(
        "--drop",
        action="store_true",
        help="Drop tables instead of creating them (DESTRUCTIVE)"
    )

    args = parser.parse_args()

    if args.drop:
        confirm = input("Are you sure you want to drop all auth tables? (yes/no): ")
        if confirm.lower() == "yes":
            drop_tables()
        else:
            print("Aborted")
    else:
        create_tables()
