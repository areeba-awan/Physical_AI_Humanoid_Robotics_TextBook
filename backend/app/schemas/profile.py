from pydantic import BaseModel, Field
from typing import Optional, List, Literal
from datetime import datetime
from uuid import UUID


class BackgroundUpdate(BaseModel):
    # Software Background
    software_experience: Literal["none", "beginner", "intermediate", "advanced"] = "none"
    programming_languages: List[str] = []
    ros_experience: bool = False
    linux_experience: Literal["none", "basic", "comfortable", "expert"] = "none"

    # Hardware Background
    hardware_experience: Literal["none", "hobbyist", "professional"] = "none"
    robotics_experience: bool = False
    previous_projects: Optional[str] = None

    # Equipment
    has_gpu_workstation: bool = False
    gpu_model: Optional[str] = None
    has_jetson_kit: bool = False
    jetson_model: Optional[str] = None
    has_robot_hardware: bool = False
    robot_description: Optional[str] = None


class ProfileUpdate(BaseModel):
    preferred_language: Optional[Literal["en", "ur"]] = None
    theme: Optional[Literal["light", "dark", "system"]] = None
    notifications_enabled: Optional[bool] = None
    current_chapter: Optional[str] = None


class ProgressUpdate(BaseModel):
    chapter_id: str
    status: Literal["not_started", "in_progress", "completed"] = "in_progress"
    progress_percent: int = Field(0, ge=0, le=100)
    time_spent: int = Field(0, ge=0)  # seconds
    quiz_score: Optional[int] = Field(None, ge=0, le=100)


class ProfileResponse(BaseModel):
    id: UUID
    user_id: UUID

    # Software Background
    software_experience: str
    programming_languages: List[str]
    ros_experience: bool
    linux_experience: str

    # Hardware Background
    hardware_experience: str
    robotics_experience: bool
    previous_projects: Optional[str]

    # Equipment
    has_gpu_workstation: bool
    gpu_model: Optional[str]
    has_jetson_kit: bool
    jetson_model: Optional[str]
    has_robot_hardware: bool
    robot_description: Optional[str]

    # Preferences
    preferred_language: str
    theme: str
    notifications_enabled: bool

    # Progress
    current_chapter: Optional[str]
    completed_chapters: List[str]

    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
