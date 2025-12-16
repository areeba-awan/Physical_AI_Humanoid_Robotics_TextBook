from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager

from app.config import settings
from app.database import init_db
from app.api.v1.router import api_router


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan events."""
    # Startup
    await init_db()
    print("Database initialized")
    yield
    # Shutdown
    print("Shutting down...")


# Create FastAPI application
app = FastAPI(
    title=settings.app_name,
    version=settings.app_version,
    description="Physical AI & Humanoid Robotics Textbook Portal API",
    docs_url="/docs",
    redoc_url="/redoc",
    lifespan=lifespan,
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API router
app.include_router(api_router, prefix="/api/v1")


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "Physical AI Textbook Portal API",
        "version": settings.app_version,
        "docs": "/docs",
    }


@app.get("/health")
async def health_check():
    """
    Comprehensive health check endpoint.

    Checks:
    - API server status
    - Database connectivity
    - Qdrant vector database
    - OpenAI API configuration
    """
    from app.services.rag_service import RAGService

    health_status = {
        "status": "healthy",
        "version": settings.app_version,
        "services": {}
    }

    # Check Qdrant
    try:
        rag_service = RAGService()
        qdrant_health = rag_service.check_collection_health()
        health_status["services"]["qdrant"] = {
            "status": "healthy" if qdrant_health.get("healthy") else "degraded",
            "vectors_count": qdrant_health.get("vectors_count", 0),
            "collection_status": qdrant_health.get("status", "unknown"),
        }
    except Exception as e:
        health_status["services"]["qdrant"] = {
            "status": "unhealthy",
            "error": str(e),
        }
        health_status["status"] = "degraded"

    # Check OpenAI configuration
    health_status["services"]["openai"] = {
        "status": "configured" if settings.openai_api_key else "not_configured",
        "model": settings.openai_model,
        "embedding_model": settings.openai_embedding_model,
    }

    if not settings.openai_api_key:
        health_status["status"] = "degraded"

    # Check database configuration
    health_status["services"]["database"] = {
        "status": "configured" if settings.database_url else "not_configured",
    }

    return health_status
