from sqlalchemy import Column, String, Boolean, DateTime, Text, ForeignKey
from sqlalchemy.dialects.postgresql import UUID, ARRAY
from sqlalchemy.orm import relationship
from datetime import datetime
import uuid

from app.database import Base


class UserProfile(Base):
    __tablename__ = "user_profiles"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), unique=True, nullable=False)

    # Software Background
    software_experience = Column(String(20), default="none")  # none, beginner, intermediate, advanced
    programming_languages = Column(ARRAY(String), default=[])
    ros_experience = Column(Boolean, default=False)
    linux_experience = Column(String(20), default="none")  # none, basic, comfortable, expert

    # Hardware Background
    hardware_experience = Column(String(20), default="none")  # none, hobbyist, professional
    robotics_experience = Column(Boolean, default=False)
    previous_projects = Column(Text, nullable=True)

    # Equipment
    has_gpu_workstation = Column(Boolean, default=False)
    gpu_model = Column(String(100), nullable=True)
    has_jetson_kit = Column(Boolean, default=False)
    jetson_model = Column(String(100), nullable=True)
    has_robot_hardware = Column(Boolean, default=False)
    robot_description = Column(Text, nullable=True)

    # Preferences
    preferred_language = Column(String(5), default="en")  # en, ur
    theme = Column(String(10), default="system")  # light, dark, system
    notifications_enabled = Column(Boolean, default=True)

    # Progress
    current_chapter = Column(String(100), nullable=True)
    completed_chapters = Column(ARRAY(String), default=[])

    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    # Relationships
    user = relationship("User", back_populates="profile")

    def __repr__(self):
        return f"<UserProfile {self.user_id}>"
