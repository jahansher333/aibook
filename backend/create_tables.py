#!/usr/bin/env python3
"""
Script to create database tables
"""
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'app'))

from app.database import engine
from app.models import Base

def create_tables():
    """Create all database tables"""
    print("Creating database tables...")
    try:
        # Create all tables
        Base.metadata.create_all(bind=engine)
        print("[SUCCESS] Database tables created successfully!")

        # Verify tables were created
        from sqlalchemy import inspect
        inspector = inspect(engine)
        tables = inspector.get_table_names()
        print(f"Tables created: {tables}")

    except Exception as e:
        print(f"[ERROR] Error creating tables: {e}")
        return False

    return True

if __name__ == "__main__":
    success = create_tables()
    if not success:
        sys.exit(1)