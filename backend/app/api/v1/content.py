from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select

from app.database import get_db
from app.models import User, UserProfile, ContentCache
from app.schemas import (
    PersonalizeRequest,
    PersonalizeResponse,
    TranslateRequest,
    TranslateResponse,
    SummaryResponse,
)
from app.core.auth import get_current_user
from app.services.content_service import ContentService

router = APIRouter()


@router.post("/{chapter_id}/personalize", response_model=PersonalizeResponse)
async def personalize_chapter(
    chapter_id: str,
    data: PersonalizeRequest,
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """Personalize chapter content based on user's background."""
    # Check cache first
    result = await db.execute(
        select(ContentCache).where(
            ContentCache.chapter_id == chapter_id,
            ContentCache.user_id == user.id,
            ContentCache.cache_type == "personalized",
        )
    )
    cached = result.scalar_one_or_none()

    if cached and not cached.is_expired:
        return PersonalizeResponse(
            personalized_content=cached.content,
            adaptations=["Loaded from cache"],
            cached=True,
        )

    # Get user profile
    result = await db.execute(select(UserProfile).where(UserProfile.user_id == user.id))
    profile = result.scalar_one_or_none()

    if not profile:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Please complete your profile first"
        )

    # Personalize content
    content_service = ContentService()
    response = await content_service.personalize_content(
        content=data.content,
        profile={
            "software_experience": profile.software_experience,
            "programming_languages": profile.programming_languages,
            "ros_experience": profile.ros_experience,
            "linux_experience": profile.linux_experience,
            "hardware_experience": profile.hardware_experience,
            "has_gpu_workstation": profile.has_gpu_workstation,
            "has_jetson_kit": profile.has_jetson_kit,
        }
    )

    # Cache the result
    if cached:
        cached.content = response["content"]
    else:
        cache_entry = ContentCache(
            chapter_id=chapter_id,
            user_id=user.id,
            cache_type="personalized",
            content=response["content"],
        )
        db.add(cache_entry)

    await db.commit()

    return PersonalizeResponse(
        personalized_content=response["content"],
        adaptations=response.get("adaptations", []),
        cached=False,
    )


@router.post("/{chapter_id}/translate", response_model=TranslateResponse)
async def translate_chapter(
    chapter_id: str,
    data: TranslateRequest,
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """Translate chapter content to Urdu."""
    # Check cache first
    result = await db.execute(
        select(ContentCache).where(
            ContentCache.chapter_id == chapter_id,
            ContentCache.user_id == user.id,
            ContentCache.cache_type == "translated",
            ContentCache.language == data.target_language,
        )
    )
    cached = result.scalar_one_or_none()

    if cached and not cached.is_expired:
        return TranslateResponse(
            translated_content=cached.content,
            preserved_terms=["Loaded from cache"],
            cached=True,
        )

    # Translate content
    content_service = ContentService()
    response = await content_service.translate_content(
        content=data.content,
        target_language=data.target_language,
    )

    # Cache the result
    if cached:
        cached.content = response["content"]
    else:
        cache_entry = ContentCache(
            chapter_id=chapter_id,
            user_id=user.id,
            cache_type="translated",
            content=response["content"],
            language=data.target_language,
        )
        db.add(cache_entry)

    await db.commit()

    return TranslateResponse(
        translated_content=response["content"],
        preserved_terms=response.get("preserved_terms", []),
        cached=False,
    )


@router.get("/{chapter_id}/summary", response_model=SummaryResponse)
async def get_chapter_summary(
    chapter_id: str,
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """Get a summary of a chapter."""
    content_service = ContentService()
    response = await content_service.summarize_content(chapter_id=chapter_id)

    return SummaryResponse(
        summary=response["summary"],
        key_concepts=response.get("key_concepts", []),
        learning_objectives=response.get("learning_objectives", []),
    )
