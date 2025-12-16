from fastapi import APIRouter

# Create a main router for the API
router = APIRouter()

# Add common endpoints if needed
@router.get("/status")
def get_status():
    return {"status": "API is running", "version": "1.0.0"}