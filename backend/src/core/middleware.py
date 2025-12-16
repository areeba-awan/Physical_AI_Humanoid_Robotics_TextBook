from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response
import logging
import time
import traceback
from typing import Callable
import sys
import os

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        # In production, you might want to add a file handler
        # logging.FileHandler('app.log')
    ]
)

logger = logging.getLogger(__name__)

class LoggingMiddleware(BaseHTTPMiddleware):
    """
    Middleware to log requests and responses
    """
    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        start_time = time.time()
        
        # Log request
        logger.info(f"Request: {request.method} {request.url}")
        logger.debug(f"Headers: {request.headers}")
        
        try:
            response = await call_next(request)
        except Exception as e:
            # Log the exception
            logger.error(f"Exception in request {request.method} {request.url}: {str(e)}")
            logger.error(traceback.format_exc())
            raise e
        
        # Calculate process time
        process_time = time.time() - start_time
        
        # Add process time to response headers
        response.headers["X-Process-Time"] = str(process_time)
        
        # Log response
        logger.info(f"Response: {response.status_code} - Process time: {process_time:.4f}s")
        
        return response

def handle_validation_error(request: Request, exc: HTTPException):
    """
    Custom handler for validation errors
    """
    logger.error(f"Validation error: {exc.detail}")
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": "Validation Error",
            "message": exc.detail,
            "path": str(request.url)
        }
    )

def handle_internal_error(request: Request, exc: Exception):
    """
    Custom handler for internal server errors
    """
    logger.error(f"Internal server error: {str(exc)}")
    logger.error(traceback.format_exc())
    return JSONResponse(
        status_code=500,
        content={
            "error": "Internal Server Error",
            "message": "An unexpected error occurred",
            "path": str(request.url)
        }
    )

def add_exception_handlers(app):
    """
    Add exception handlers to the FastAPI app
    """
    @app.exception_handler(HTTPException)
    async def custom_http_exception_handler(request: Request, exc: HTTPException):
        return handle_validation_error(request, exc)
    
    @app.exception_handler(Exception)
    async def custom_exception_handler(request: Request, exc: Exception):
        return handle_internal_error(request, exc)

def setup_middlewares(app):
    """
    Setup all middlewares for the application
    """
    # Add logging middleware
    app.add_middleware(LoggingMiddleware)
    
    # Add trusted host middleware (for security)
    # In production, specify your actual allowed hosts
    app.add_middleware(
        TrustedHostMiddleware, 
        allowed_hosts=["*"]  # In production, replace with specific domains
    )
    
    # Add exception handlers
    add_exception_handlers(app)