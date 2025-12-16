from sqlalchemy import Column, Integer, String, DateTime, Text, ForeignKey, UUID, Float
from sqlalchemy.dialects.postgresql import UUID as PG_UUID
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column, relationship
from datetime import datetime
import uuid
from typing import Optional, List


class Base(DeclarativeBase):
    pass


class BookContent(Base):
    """
    Model for storing book content chunks with their embeddings
    """
    __tablename__ = "book_content"

    id: Mapped[UUID] = mapped_column(
        PG_UUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4
    )
    book_id: Mapped[UUID] = mapped_column(
        PG_UUID(as_uuid=True),
        nullable=False
    )
    chunk_text: Mapped[str] = mapped_column(
        Text,
        nullable=False
    )
    chunk_metadata: Mapped[dict] = mapped_column(
        String,  # Will store JSON as string
        nullable=True
    )
    # Note: embedding_vector is not stored in Postgres but in Qdrant
    created_at: Mapped[datetime] = mapped_column(
        DateTime,
        default=datetime.utcnow,
        nullable=False
    )
    updated_at: Mapped[datetime] = mapped_column(
        DateTime,
        default=datetime.utcnow,
        onupdate=datetime.utcnow,
        nullable=False
    )

    # Relationship with GeneratedResponse
    generated_responses: Mapped[List["GeneratedResponse"]] = relationship(
        "GeneratedResponse",
        secondary="response_content_association",
        back_populates="book_content"
    )


class UserQuery(Base):
    """
    Model for storing user queries
    """
    __tablename__ = "user_query"

    id: Mapped[UUID] = mapped_column(
        PG_UUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4
    )
    session_id: Mapped[UUID] = mapped_column(
        PG_UUID(as_uuid=True),
        ForeignKey("session.id"),
        nullable=False
    )
    query_text: Mapped[str] = mapped_column(
        Text,
        nullable=False
    )
    query_metadata: Mapped[dict] = mapped_column(
        String,  # Will store JSON as string
        nullable=True
    )
    created_at: Mapped[datetime] = mapped_column(
        DateTime,
        default=datetime.utcnow,
        nullable=False
    )

    # Relationship
    session: Mapped["Session"] = relationship("Session", back_populates="user_queries")
    generated_response: Mapped["GeneratedResponse"] = relationship("GeneratedResponse", back_populates="user_query", uselist=False)


class GeneratedResponse(Base):
    """
    Model for storing AI-generated responses
    """
    __tablename__ = "generated_response"

    id: Mapped[UUID] = mapped_column(
        PG_UUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4
    )
    query_id: Mapped[UUID] = mapped_column(
        PG_UUID(as_uuid=True),
        ForeignKey("user_query.id"),
        nullable=False
    )
    response_text: Mapped[str] = mapped_column(
        Text,
        nullable=False
    )
    referenced_content: Mapped[dict] = mapped_column(
        String,  # Will store JSON as string
        nullable=True
    )
    confidence_score: Mapped[float] = mapped_column(
        Float,
        nullable=True
    )
    created_at: Mapped[datetime] = mapped_column(
        DateTime,
        default=datetime.utcnow,
        nullable=False
    )

    # Relationships
    user_query: Mapped["UserQuery"] = relationship("UserQuery", back_populates="generated_response")
    book_content: Mapped[List["BookContent"]] = relationship(
        "BookContent",
        secondary="response_content_association",
        back_populates="generated_responses"
    )


class Session(Base):
    """
    Model for storing chat sessions
    """
    __tablename__ = "session"

    id: Mapped[UUID] = mapped_column(
        PG_UUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4
    )
    user_id: Mapped[Optional[UUID]] = mapped_column(
        PG_UUID(as_uuid=True),
        nullable=True
    )
    book_id: Mapped[UUID] = mapped_column(
        PG_UUID(as_uuid=True),
        nullable=False
    )
    session_metadata: Mapped[dict] = mapped_column(
        String,  # Will store JSON as string
        nullable=True
    )
    created_at: Mapped[datetime] = mapped_column(
        DateTime,
        default=datetime.utcnow,
        nullable=False
    )
    updated_at: Mapped[datetime] = mapped_column(
        DateTime,
        default=datetime.utcnow,
        onupdate=datetime.utcnow,
        nullable=False
    )
    expires_at: Mapped[datetime] = mapped_column(
        DateTime,
        nullable=False
    )

    # Relationships
    user_queries: Mapped[List["UserQuery"]] = relationship("UserQuery", back_populates="session")
    chat_messages: Mapped[List["ChatMessage"]] = relationship("ChatMessage", back_populates="session")


class ChatMessage(Base):
    """
    Model for storing individual chat messages in a session
    """
    __tablename__ = "chat_message"

    id: Mapped[UUID] = mapped_column(
        PG_UUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4
    )
    session_id: Mapped[UUID] = mapped_column(
        PG_UUID(as_uuid=True),
        ForeignKey("session.id"),
        nullable=False
    )
    message_type: Mapped[str] = mapped_column(
        String(20),  # 'user', 'assistant', 'system'
        nullable=False
    )
    content: Mapped[str] = mapped_column(
        Text,
        nullable=False
    )
    timestamp: Mapped[datetime] = mapped_column(
        DateTime,
        default=datetime.utcnow,
        nullable=False
    )
    metadata: Mapped[dict] = mapped_column(
        String,  # Will store JSON as string
        nullable=True
    )

    # Relationship
    session: Mapped["Session"] = relationship("Session", back_populates="chat_messages")


# Association table for many-to-many relationship between GeneratedResponse and BookContent
class ResponseContentAssociation(Base):
    """
    Association table for linking GeneratedResponse to BookContent
    """
    __tablename__ = "response_content_association"
    
    response_id: Mapped[UUID] = mapped_column(
        PG_UUID(as_uuid=True),
        ForeignKey("generated_response.id"),
        primary_key=True
    )
    content_id: Mapped[UUID] = mapped_column(
        PG_UUID(as_uuid=True),
        ForeignKey("book_content.id"),
        primary_key=True
    )