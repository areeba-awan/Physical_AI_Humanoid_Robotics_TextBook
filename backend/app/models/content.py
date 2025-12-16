from sqlalchemy import Column, String, Integer, DateTime, ForeignKey, Text
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from datetime import datetime
import uuid

from app.database import Base


class ContentCache(Base):
    __tablename__ = "content_cache"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    chapter_id = Column(String(100), nullable=False, index=True)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), nullable=False)
    cache_type = Column(String(20), nullable=False)  # 'personalized', 'translated'
    content = Column(Text, nullable=False)
    language = Column(String(5), nullable=True)  # for translations
    created_at = Column(DateTime, default=datetime.utcnow)
    expires_at = Column(DateTime, nullable=True)

    __table_args__ = (
        # Unique constraint on chapter_id, user_id, cache_type, language
        {"sqlite_autoincrement": True},
    )

    def __repr__(self):
        return f"<ContentCache {self.chapter_id} - {self.cache_type}>"

    @property
    def is_expired(self) -> bool:
        if self.expires_at is None:
            return False
        return datetime.utcnow() > self.expires_at


class LearningProgress(Base):
    __tablename__ = "learning_progress"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), nullable=False, index=True)
    chapter_id = Column(String(100), nullable=False)
    status = Column(String(20), default="not_started")  # 'not_started', 'in_progress', 'completed'
    progress_percent = Column(Integer, default=0)
    time_spent = Column(Integer, default=0)  # seconds
    quiz_score = Column(Integer, nullable=True)
    completed_at = Column(DateTime, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    # Relationships
    user = relationship("User", back_populates="learning_progress")

    __table_args__ = (
        # Unique constraint on user_id, chapter_id
        {"sqlite_autoincrement": True},
    )

    def __repr__(self):
        return f"<LearningProgress {self.user_id} - {self.chapter_id}>"
