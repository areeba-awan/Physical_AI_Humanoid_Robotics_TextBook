from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import DeclarativeBase
from contextlib import asynccontextmanager
import os
from dotenv import load_dotenv

load_dotenv()

# Database URL from environment variable
DATABASE_URL = os.getenv("NEON_DATABASE_URL", "postgresql+asyncpg://user:password@localhost/dbname")

# Create async engine
engine = create_async_engine(
    DATABASE_URL,
    echo=True,  # Set to True for debugging SQL queries
    pool_pre_ping=True,  # Verify connections before use
)

# Create async session maker
async_session_local = sessionmaker(
    engine, 
    class_=AsyncSession, 
    expire_on_commit=False
)

class Base(DeclarativeBase):
    pass

# Dependency to get DB session
async def get_db():
    async with async_session_local() as session:
        yield session

# Function to create tables (for initial setup/migration)
async def create_tables():
    async with engine.begin() as conn:
        # Create all tables defined in models
        await conn.run_sync(Base.metadata.create_all)

# Alternative migration function (for more complex migration scenarios)
async def run_migrations():
    """
    Run database migrations.
    This is a simplified version - in production, you'd use Alembic for more advanced migrations.
    """
    async with engine.begin() as conn:
        # Drop and recreate all tables (for development only)
        # In production, use Alembic for proper migration management
        await conn.run_sync(Base.metadata.drop_all)
        await conn.run_sync(Base.metadata.create_all)