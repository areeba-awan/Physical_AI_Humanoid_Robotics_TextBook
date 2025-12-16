from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select

from app.database import get_db
from app.models import User, UserProfile
from app.schemas import ProfileResponse, ProfileUpdate, BackgroundUpdate, ProgressUpdate
from app.core.auth import get_current_user

router = APIRouter()


@router.get("", response_model=ProfileResponse)
async def get_profile(
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """Get the current user's profile."""
    result = await db.execute(select(UserProfile).where(UserProfile.user_id == user.id))
    profile = result.scalar_one_or_none()

    if not profile:
        # Create default profile if it doesn't exist
        profile = UserProfile(user_id=user.id)
        db.add(profile)
        await db.commit()
        await db.refresh(profile)

    return ProfileResponse.model_validate(profile)


@router.put("", response_model=ProfileResponse)
async def update_profile(
    data: ProfileUpdate,
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """Update the current user's profile preferences."""
    result = await db.execute(select(UserProfile).where(UserProfile.user_id == user.id))
    profile = result.scalar_one_or_none()

    if not profile:
        profile = UserProfile(user_id=user.id)
        db.add(profile)

    # Update fields if provided
    update_data = data.model_dump(exclude_unset=True)
    for field, value in update_data.items():
        setattr(profile, field, value)

    await db.commit()
    await db.refresh(profile)

    return ProfileResponse.model_validate(profile)


@router.put("/background", response_model=dict)
async def update_background(
    data: BackgroundUpdate,
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """Update the user's background information (onboarding data)."""
    result = await db.execute(select(UserProfile).where(UserProfile.user_id == user.id))
    profile = result.scalar_one_or_none()

    if not profile:
        profile = UserProfile(user_id=user.id)
        db.add(profile)

    # Update all background fields
    profile.software_experience = data.software_experience
    profile.programming_languages = data.programming_languages
    profile.ros_experience = data.ros_experience
    profile.linux_experience = data.linux_experience
    profile.hardware_experience = data.hardware_experience
    profile.robotics_experience = data.robotics_experience
    profile.previous_projects = data.previous_projects
    profile.has_gpu_workstation = data.has_gpu_workstation
    profile.gpu_model = data.gpu_model
    profile.has_jetson_kit = data.has_jetson_kit
    profile.jetson_model = data.jetson_model
    profile.has_robot_hardware = data.has_robot_hardware
    profile.robot_description = data.robot_description

    await db.commit()
    await db.refresh(profile)

    return {
        "success": True,
        "profile": ProfileResponse.model_validate(profile)
    }


@router.get("/progress")
async def get_progress(
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """Get the user's learning progress."""
    result = await db.execute(select(UserProfile).where(UserProfile.user_id == user.id))
    profile = result.scalar_one_or_none()

    if not profile:
        return {
            "current_chapter": None,
            "completed_chapters": [],
            "total_completed": 0,
        }

    return {
        "current_chapter": profile.current_chapter,
        "completed_chapters": profile.completed_chapters or [],
        "total_completed": len(profile.completed_chapters or []),
    }


@router.put("/progress")
async def update_progress(
    data: ProgressUpdate,
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """Update the user's learning progress for a chapter."""
    result = await db.execute(select(UserProfile).where(UserProfile.user_id == user.id))
    profile = result.scalar_one_or_none()

    if not profile:
        profile = UserProfile(user_id=user.id)
        db.add(profile)

    # Update current chapter
    profile.current_chapter = data.chapter_id

    # Update completed chapters if status is completed
    if data.status == "completed":
        completed = profile.completed_chapters or []
        if data.chapter_id not in completed:
            completed.append(data.chapter_id)
            profile.completed_chapters = completed

    await db.commit()
    await db.refresh(profile)

    return {
        "success": True,
        "current_chapter": profile.current_chapter,
        "completed_chapters": profile.completed_chapters,
    }
