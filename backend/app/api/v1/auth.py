from fastapi import APIRouter, Depends, HTTPException, status, Request
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from datetime import datetime

from app.database import get_db
from app.models import User, UserProfile, Session
from app.schemas import (
    SignupRequest,
    SigninRequest,
    AuthResponse,
    UserResponse,
    SessionResponse,
)
from app.core.security import (
    get_password_hash,
    verify_password,
    create_access_token,
    create_refresh_token,
    get_token_expiry,
    verify_token,
)
from app.core.auth import get_current_user

router = APIRouter()


@router.post("/signup", response_model=AuthResponse, status_code=status.HTTP_201_CREATED)
async def signup(
    request: Request,
    data: SignupRequest,
    db: AsyncSession = Depends(get_db)
):
    """Create a new user account."""
    # Check if user exists
    result = await db.execute(select(User).where(User.email == data.email))
    existing_user = result.scalar_one_or_none()

    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email already registered"
        )

    # Create user
    user = User(
        email=data.email,
        name=data.name,
        password_hash=get_password_hash(data.password),
    )
    db.add(user)
    await db.flush()

    # Create empty profile
    profile = UserProfile(user_id=user.id)
    db.add(profile)

    # Create session
    access_token = create_access_token({"sub": str(user.id)})
    refresh_token = create_refresh_token({"sub": str(user.id)})

    session = Session(
        user_id=user.id,
        token=access_token,
        refresh_token=refresh_token,
        expires_at=get_token_expiry("access"),
        refresh_expires_at=get_token_expiry("refresh"),
        ip_address=request.client.host if request.client else None,
        user_agent=request.headers.get("user-agent"),
    )
    db.add(session)

    await db.commit()
    await db.refresh(user)

    return AuthResponse(
        user=UserResponse.model_validate(user),
        session=SessionResponse(
            token=access_token,
            expires_at=session.expires_at,
        ),
        requires_onboarding=True,
    )


@router.post("/signin", response_model=AuthResponse)
async def signin(
    request: Request,
    data: SigninRequest,
    db: AsyncSession = Depends(get_db)
):
    """Sign in to an existing account."""
    # Find user
    result = await db.execute(select(User).where(User.email == data.email))
    user = result.scalar_one_or_none()

    if not user or not verify_password(data.password, user.password_hash):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password"
        )

    # Update last login
    user.last_login_at = datetime.utcnow()

    # Create new session
    access_token = create_access_token({"sub": str(user.id)})
    refresh_token = create_refresh_token({"sub": str(user.id)})

    session = Session(
        user_id=user.id,
        token=access_token,
        refresh_token=refresh_token,
        expires_at=get_token_expiry("access"),
        refresh_expires_at=get_token_expiry("refresh"),
        ip_address=request.client.host if request.client else None,
        user_agent=request.headers.get("user-agent"),
    )
    db.add(session)

    await db.commit()
    await db.refresh(user)

    # Check if user needs onboarding
    result = await db.execute(select(UserProfile).where(UserProfile.user_id == user.id))
    profile = result.scalar_one_or_none()
    requires_onboarding = profile is None or profile.software_experience == "none"

    return AuthResponse(
        user=UserResponse.model_validate(user),
        session=SessionResponse(
            token=access_token,
            expires_at=session.expires_at,
        ),
        requires_onboarding=requires_onboarding,
    )


@router.post("/signout")
async def signout(
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """Sign out and invalidate the current session."""
    # Delete all sessions for this user (or just the current one)
    result = await db.execute(select(Session).where(Session.user_id == user.id))
    sessions = result.scalars().all()

    for session in sessions:
        await db.delete(session)

    await db.commit()

    return {"success": True, "message": "Successfully signed out"}


@router.get("/session", response_model=AuthResponse)
async def get_session(
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """Get the current session information."""
    # Get the latest session
    result = await db.execute(
        select(Session)
        .where(Session.user_id == user.id)
        .order_by(Session.created_at.desc())
    )
    session = result.scalar_one_or_none()

    if not session:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Session not found"
        )

    # Check if user needs onboarding
    result = await db.execute(select(UserProfile).where(UserProfile.user_id == user.id))
    profile = result.scalar_one_or_none()
    requires_onboarding = profile is None or profile.software_experience == "none"

    return AuthResponse(
        user=UserResponse.model_validate(user),
        session=SessionResponse(
            token=session.token,
            expires_at=session.expires_at,
        ),
        requires_onboarding=requires_onboarding,
    )


@router.post("/refresh", response_model=SessionResponse)
async def refresh_token(
    request: Request,
    db: AsyncSession = Depends(get_db)
):
    """Refresh the access token using the refresh token."""
    auth_header = request.headers.get("Authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Refresh token required"
        )

    refresh_token = auth_header[7:]
    payload = verify_token(refresh_token, token_type="refresh")

    if not payload:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired refresh token"
        )

    user_id = payload.get("sub")

    # Find the session with this refresh token
    result = await db.execute(
        select(Session).where(Session.refresh_token == refresh_token)
    )
    session = result.scalar_one_or_none()

    if not session or session.is_refresh_expired:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Session expired, please sign in again"
        )

    # Create new access token
    new_access_token = create_access_token({"sub": user_id})

    # Update session
    session.token = new_access_token
    session.expires_at = get_token_expiry("access")

    await db.commit()

    return SessionResponse(
        token=new_access_token,
        expires_at=session.expires_at,
    )
