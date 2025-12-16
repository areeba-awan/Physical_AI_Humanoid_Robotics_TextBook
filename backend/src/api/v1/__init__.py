from fastapi import APIRouter

# Create a main router for v1 API
router = APIRouter()

# Include all v1 endpoints here
from . import query
from . import session

# Add them to the main v1 router
router.include_router(query.router, tags=["query"])
router.include_router(session.router, tags=["session"])