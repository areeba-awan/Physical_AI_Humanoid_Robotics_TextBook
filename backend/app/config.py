"""
Application Configuration

Handles all environment variables and application settings.
Supports Neon Serverless Postgres and Qdrant Cloud.
"""

from pydantic_settings import BaseSettings
from functools import lru_cache
from typing import List
import os


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Application
    app_name: str = "PhysicalAI-Textbook"
    app_version: str = "1.0.0"
    debug: bool = False

    # Database - Neon Serverless Postgres
    # Format: postgresql+asyncpg://user:password@ep-xxx.region.aws.neon.tech/database
    database_url: str = "postgresql+asyncpg://user:password@localhost:5432/physicalai"
    database_pool_size: int = 5  # Lower for serverless
    database_max_overflow: int = 10

    # Authentication
    secret_key: str = "your-super-secret-key-change-in-production"
    algorithm: str = "HS256"
    access_token_expire_minutes: int = 15
    refresh_token_expire_days: int = 7

    # OpenAI Configuration
    openai_api_key: str = ""
    openai_model: str = "gpt-4o"
    openai_embedding_model: str = "text-embedding-3-large"
    openai_embedding_dimensions: int = 3072  # text-embedding-3-large default
    openai_base_url: str = ""  # For OpenRouter or custom endpoints

    # OpenRouter Configuration (fallback to be used if OPENAI_API_KEY is not set)
    openrouter_api_key: str = ""
    openrouter_base_url: str = "https://openrouter.ai/api/v1"
    openrouter_model: str = "anthropic/claude-3-sonnet"

    # Qwen Configuration
    qwen_api_key: str = ""
    qwen_embedding_model: str = "text-embedding-v1"
    qwen_embedding_dimensions: int = 1536  # Qwen embedding dimensions
    qwen_embedding_base_url: str = ""

    # Qdrant Cloud Configuration
    # URL format for cloud: https://xxx.region.aws.cloud.qdrant.io:6333
    # URL format for local: http://localhost:6333
    qdrant_url: str = "http://localhost:6333"
    qdrant_api_key: str = ""  # Required for Qdrant Cloud
    qdrant_collection: str = "physicalai_textbook"

    # CORS - Add your frontend domains
    cors_origins: List[str] = [
        "http://localhost:3000",
        "http://localhost:3001",
        "http://127.0.0.1:3000",
    ]

    # Rate Limiting
    rate_limit_requests: int = 100
    rate_limit_window: int = 60  # seconds

    # RAG Configuration
    rag_score_threshold: float = 0.35
    rag_max_results: int = 10
    rag_max_context_tokens: int = 4000

    # Chat Configuration
    chat_session_timeout_hours: int = 24
    chat_max_history_messages: int = 10

    class Config:
        env_file = ".env"
        case_sensitive = False
        extra = "ignore"  # Ignore extra env vars


@lru_cache()
def get_settings() -> Settings:
    """Get cached settings instance."""
    return Settings()


settings = get_settings()


def validate_settings() -> dict:
    """Validate critical settings and return status."""
    issues = []
    warnings = []

    # Check required settings - either OpenAI or OpenRouter API key
    if not settings.openai_api_key and not settings.openrouter_api_key:
        issues.append("No API key configured. Please set either OPENAI_API_KEY or OPENROUTER_API_KEY")

    if settings.secret_key == "your-super-secret-key-change-in-production":
        warnings.append("SECRET_KEY should be changed for production")

    if "localhost" in settings.database_url and not settings.debug:
        warnings.append("Using localhost database URL in non-debug mode")

    # Validate Qdrant settings for cloud
    if "cloud.qdrant.io" in settings.qdrant_url and not settings.qdrant_api_key:
        issues.append("QDRANT_API_KEY is required for Qdrant Cloud")

    return {
        "valid": len(issues) == 0,
        "issues": issues,
        "warnings": warnings,
        "settings": {
            "app_name": settings.app_name,
            "openai_model": settings.openai_model,
            "openrouter_model": settings.openrouter_model,
            "embedding_model": settings.openai_embedding_model,
            "embedding_dimensions": settings.openai_embedding_dimensions,
            "qdrant_collection": settings.qdrant_collection,
            "debug": settings.debug,
        }
    }
