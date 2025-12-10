from pydantic_settings import BaseSettings
from typing import List

class Settings(BaseSettings):
    # API Keys
    GROQ_API_KEY: str
    COHERE_API_KEY: str
    QDRANT_URL: str
    QDRANT_API_KEY: str
    NEON_DATABASE_URL: str

    # LiteLLM
    LITELLM_MODEL: str = "groq/llama-3.1-70b-instant"

    # Qdrant
    QDRANT_COLLECTION: str = "physical_ai_book"

    # Environment
    ENVIRONMENT: str = "development"
    LOG_LEVEL: str = "INFO"

    # CORS
    CORS_ORIGINS: List[str] = ["http://localhost:3000", "https://*.github.io"]

    class Config:
        env_file = ".env"
        case_sensitive = True

settings = Settings()
