from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from backend.src.core.database import create_tables
from backend.src.api.v1 import router as api_router
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Application lifespan event handler.
    Runs setup when the application starts and teardown when it stops.
    """
    logger.info("Starting up the application...")
    
    # Create database tables
    await create_tables()
    logger.info("Database tables created successfully")
    
    yield  # Application runs during this period
    
    logger.info("Shutting down the application...")

# Create FastAPI app with lifespan
app = FastAPI(
    title="RAG Chatbot API",
    description="API for integrating RAG chatbot functionality with digital books",
    version="1.0.0",
    lifespan=lifespan
)

# Add rate limiting exception handler
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# Configure CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Expose headers if needed
    # expose_headers=["Access-Control-Allow-Origin"]
)

# Include API routes
app.include_router(api_router, prefix="/v1")

# Health check endpoint
@app.get("/health")
async def health_check():
    return {"status": "healthy"}

# Root endpoint
@app.get("/")
async def root():
    return {"message": "RAG Chatbot API - Ready to serve book-related queries"}