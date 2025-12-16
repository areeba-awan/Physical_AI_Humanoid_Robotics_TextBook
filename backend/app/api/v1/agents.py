from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select

from app.database import get_db
from app.models import User, UserProfile
from app.schemas import (
    SummarizeRequest,
    SummarizeResponse,
    TranslateAgentRequest,
    TranslateAgentResponse,
    PersonalizeAgentRequest,
    PersonalizeAgentResponse,
    GenerateRequest,
    GenerateResponse,
)
from app.core.auth import get_current_user
from app.services.agent_service import AgentService

router = APIRouter()


@router.post("/summarize", response_model=SummarizeResponse)
async def summarize_content(
    data: SummarizeRequest,
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """Summarize content using the ChapterSummarizerAgent."""
    agent_service = AgentService()
    response = await agent_service.summarize(
        content=data.content,
        summary_type=data.summary_type,
    )

    return SummarizeResponse(
        summary=response["summary"],
        key_points=response.get("key_points", []),
        agent_used="ChapterSummarizerAgent",
    )


@router.post("/translate", response_model=TranslateAgentResponse)
async def translate_content(
    data: TranslateAgentRequest,
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """Translate content using the UrduTranslatorAgent."""
    agent_service = AgentService()
    response = await agent_service.translate(
        content=data.content,
        target_language=data.target_language,
        preserve_terms=data.preserve_terms,
    )

    return TranslateAgentResponse(
        translated_content=response["translated_content"],
        preserved_terms=response.get("preserved_terms", []),
        agent_used="UrduTranslatorAgent",
    )


@router.post("/personalize", response_model=PersonalizeAgentResponse)
async def personalize_content(
    data: PersonalizeAgentRequest,
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """Personalize content using the PersonalizationAgent."""
    # Get user profile
    result = await db.execute(select(UserProfile).where(UserProfile.user_id == user.id))
    profile = result.scalar_one_or_none()

    if not profile:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Please complete your profile first"
        )

    agent_service = AgentService()
    response = await agent_service.personalize(
        content=data.content,
        chapter_id=data.chapter_id,
        user_profile={
            "software_experience": profile.software_experience,
            "programming_languages": profile.programming_languages,
            "ros_experience": profile.ros_experience,
            "linux_experience": profile.linux_experience,
            "hardware_experience": profile.hardware_experience,
            "has_gpu_workstation": profile.has_gpu_workstation,
            "has_jetson_kit": profile.has_jetson_kit,
        }
    )

    return PersonalizeAgentResponse(
        personalized_content=response["personalized_content"],
        adaptations=response.get("adaptations", []),
        agent_used="PersonalizationAgent",
    )


@router.post("/generate", response_model=GenerateResponse)
async def generate_content(
    data: GenerateRequest,
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """Generate educational content using the ContentGeneratorAgent."""
    agent_service = AgentService()
    response = await agent_service.generate(
        topic=data.topic,
        content_type=data.content_type,
        difficulty=data.difficulty,
        count=data.count,
    )

    return GenerateResponse(
        generated_content=response["generated_content"],
        content_type=data.content_type,
        agent_used="ContentGeneratorAgent",
    )
