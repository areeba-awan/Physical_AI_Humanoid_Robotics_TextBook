from sqlalchemy import Column, Integer, String, DateTime, Text, ForeignKey, UUID, Float
from sqlalchemy.dialects.postgresql import UUID as PG_UUID
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from backend.src.core.database import Base
from datetime import datetime
import uuid


class BookContent(Base):
    """
    Model for storing book content chunks with their embeddings
    """
    __tablename__ = "book_content"

    id = Column(
        PG_UUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4
    )
    book_id = Column(
        PG_UUID(as_uuid=True),
        nullable=False
    )
    chunk_text = Column(
        Text,
        nullable=False
    )
    chunk_metadata = Column(
        String,  # Will store JSON as string
        nullable=True
    )
    # Note: embedding_vector is not stored in Postgres but in Qdrant
    created_at = Column(
        DateTime,
        default=datetime.utcnow,
        nullable=False
    )
    updated_at = Column(
        DateTime,
        default=datetime.utcnow,
        onupdate=datetime.utcnow,
        nullable=False
    )


class UserQuery(Base):
    """
    Model for storing user queries
    """
    __tablename__ = "user_query"

    id = Column(
        PG_UUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4
    )
    session_id = Column(
        PG_UUID(as_uuid=True),
        ForeignKey("session.id"),
        nullable=False
    )
    query_text = Column(
        Text,
        nullable=False
    )
    query_metadata = Column(
        String,  # Will store JSON as string
        nullable=True
    )
    created_at = Column(
        DateTime,
        default=datetime.utcnow,
        nullable=False
    )

    # Relationship
    session = relationship("Session", back_populates="user_queries")
    generated_response = relationship("GeneratedResponse", back_populates="user_query", uselist=False)


class GeneratedResponse(Base):
    """
    Model for storing AI-generated responses
    """
    __tablename__ = "generated_response"

    id = Column(
        PG_UUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4
    )
    query_id = Column(
        PG_UUID(as_uuid=True),
        ForeignKey("user_query.id"),
        nullable=False
    )
    response_text = Column(
        Text,
        nullable=False
    )
    referenced_content = Column(
        String,  # Will store JSON as string
        nullable=True
    )
    confidence_score = Column(
        Float,
        nullable=True
    )
    created_at = Column(
        DateTime,
        default=datetime.utcnow,
        nullable=False
    )

    # Relationships
    user_query = relationship("UserQuery", back_populates="generated_response")
    content_references = relationship("ContentReference", back_populates="generated_response", cascade="all, delete-orphan")


class Session(Base):
    """
    Model for storing chat sessions
    """
    __tablename__ = "session"

    id = Column(
        PG_UUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4
    )
    user_id = Column(
        PG_UUID(as_uuid=True),
        nullable=True
    )
    book_id = Column(
        PG_UUID(as_uuid=True),
        nullable=False
    )
    session_metadata = Column(
        String,  # Will store JSON as string
        nullable=True
    )
    created_at = Column(
        DateTime,
        default=datetime.utcnow,
        nullable=False
    )
    updated_at = Column(
        DateTime,
        default=datetime.utcnow,
        onupdate=datetime.utcnow,
        nullable=False
    )
    expires_at = Column(
        DateTime,
        nullable=False
    )

    # Additional reading context fields
    current_page = Column(Integer, nullable=True)  # Current page the user is reading
    current_section = Column(String(255), nullable=True)  # Current section/chapter name
    reading_position = Column(Text, nullable=True)  # More detailed reading position

    # Relationships
    user_queries = relationship("UserQuery", back_populates="session")
    chat_messages = relationship("ChatMessage", back_populates="session")


class ChatMessage(Base):
    """
    Model for storing individual chat messages in a session
    """
    __tablename__ = "chat_message"

    id = Column(
        PG_UUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4
    )
    session_id = Column(
        PG_UUID(as_uuid=True),
        ForeignKey("session.id"),
        nullable=False
    )
    message_type = Column(
        String(20),  # 'user', 'assistant', 'system'
        nullable=False
    )
    content = Column(
        Text,
        nullable=False
    )
    timestamp = Column(
        DateTime,
        default=datetime.utcnow,
        nullable=False
    )
    metadata = Column(
        String,  # Will store JSON as string
        nullable=True
    )

    # Relationship
    session = relationship("Session", back_populates="chat_messages")


class ContentReference(Base):
    """
    Model for storing references to book content within generated responses
    """
    __tablename__ = "content_reference"

    id = Column(
        PG_UUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4
    )
    response_id = Column(
        PG_UUID(as_uuid=True),
        ForeignKey("generated_response.id"),
        nullable=False
    )
    content_id = Column(
        PG_UUID(as_uuid=True),
        nullable=False  # References the book content chunk
    )
    text_snippet = Column(
        Text,
        nullable=True  # The referenced text snippet
    )
    page_number = Column(
        Integer,
        nullable=True
    )
    section_title = Column(
        String(255),
        nullable=True
    )
    created_at = Column(
        DateTime,
        default=datetime.utcnow,
        nullable=False
    )

    # Relationship
    generated_response = relationship("GeneratedResponse", back_populates="content_references")