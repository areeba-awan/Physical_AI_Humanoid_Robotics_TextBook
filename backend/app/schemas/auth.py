from pydantic import BaseModel, EmailStr, Field
from typing import Optional
from datetime import datetime
from uuid import UUID

from app.schemas.user import UserResponse


class SignupRequest(BaseModel):
    email: EmailStr
    password: str = Field(..., min_length=8)
    name: str = Field(..., min_length=2, max_length=255)


class SigninRequest(BaseModel):
    email: EmailStr
    password: str


class TokenResponse(BaseModel):
    token: str
    expires_at: datetime


class SessionResponse(BaseModel):
    token: str
    expires_at: datetime
    refresh_token: Optional[str] = None


class AuthResponse(BaseModel):
    user: UserResponse
    session: SessionResponse
    requires_onboarding: bool = False

    class Config:
        from_attributes = True
