"""
Database models and connection for Neon Serverless Postgres
"""
import os
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlalchemy.orm import declarative_base
from sqlalchemy import Column, String, Text, DateTime, Integer, JSON
from datetime import datetime

Base = declarative_base()

# Neon Postgres connection string
DATABASE_URL = os.getenv(
    "DATABASE_URL",
    "postgresql+asyncpg://user:password@localhost/dbname"
)

engine = create_async_engine(
    DATABASE_URL,
    echo=False,
    pool_pre_ping=True,
    pool_size=5,
    max_overflow=10
)

AsyncSessionLocal = async_sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False
)


class Conversation(Base):
    """Store conversation sessions"""
    __tablename__ = "conversations"
    
    id = Column(String, primary_key=True)
    user_id = Column(String, index=True)  # Optional: for multi-user support
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    metadata = Column(JSON, default={})  # Store additional context


class Message(Base):
    """Store individual messages in conversations"""
    __tablename__ = "messages"
    
    id = Column(Integer, primary_key=True, autoincrement=True)
    conversation_id = Column(String, index=True)
    role = Column(String)  # 'user', 'assistant', 'system'
    content = Column(Text)
    selected_text = Column(Text, nullable=True)  # Store selected text context
    metadata = Column(JSON, default={})  # Store embeddings, sources, etc.
    created_at = Column(DateTime, default=datetime.utcnow)


async def init_db():
    """Initialize database tables"""
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
    print("Database tables initialized")


async def get_db():
    """Dependency for getting database session"""
    async with AsyncSessionLocal() as session:
        try:
            yield session
        finally:
            await session.close()
