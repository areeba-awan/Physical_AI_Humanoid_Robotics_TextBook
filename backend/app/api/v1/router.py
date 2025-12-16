from fastapi import APIRouter

from app.api.v1 import auth, profile, chat, content, agents, ingest

api_router = APIRouter()

api_router.include_router(auth.router, prefix="/auth", tags=["Authentication"])
api_router.include_router(profile.router, prefix="/profile", tags=["Profile"])
api_router.include_router(chat.router, prefix="/chat", tags=["Chat"])
api_router.include_router(content.router, prefix="/chapters", tags=["Content"])
api_router.include_router(agents.router, prefix="/agents", tags=["Agents"])
api_router.include_router(ingest.router, prefix="/ingest", tags=["Ingestion"])
