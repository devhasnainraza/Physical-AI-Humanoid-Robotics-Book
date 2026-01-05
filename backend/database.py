"""
Database models and connection for Neon Serverless Postgres
"""
import os
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlalchemy.orm import declarative_base
from sqlalchemy import Column, String, Text, DateTime, Integer, JSON
from datetime import datetime
from dotenv import load_dotenv

# Load .env from backend directory
env_path = os.path.join(os.path.dirname(__file__), '.env')
load_dotenv(env_path)

Base = declarative_base()

# Neon Postgres connection string
DATABASE_URL = os.getenv("DATABASE_URL")
if not DATABASE_URL:
    raise ValueError("DATABASE_URL environment variable is required. Please set it in backend/.env file")

# Ensure DATABASE_URL has +asyncpg driver and remove sslmode (asyncpg uses ssl parameter)
if DATABASE_URL.startswith("postgresql://") and "+asyncpg" not in DATABASE_URL:
    DATABASE_URL = DATABASE_URL.replace("postgresql://", "postgresql+asyncpg://", 1)
elif not DATABASE_URL.startswith("postgresql"):
    raise ValueError(f"Invalid DATABASE_URL format: {DATABASE_URL}. Must start with postgresql:// or postgresql+asyncpg://")

# Remove sslmode from URL (asyncpg doesn't support it as a query parameter)
# Instead, we'll use connect_args for SSL
from urllib.parse import urlparse, parse_qs, urlencode, urlunparse

parsed = urlparse(DATABASE_URL)
query_params = parse_qs(parsed.query)
ssl_required = query_params.pop('sslmode', None) or query_params.pop('ssl', None)

# Rebuild URL without sslmode
new_query = urlencode(query_params, doseq=True)
clean_url = urlunparse(parsed._replace(query=new_query))

# Configure SSL for asyncpg
connect_args = {}
if ssl_required or 'require' in str(ssl_required):
    connect_args['ssl'] = 'require'

engine = create_async_engine(
    clean_url,
    echo=False,
    pool_pre_ping=True,
    pool_size=5,
    max_overflow=10,
    connect_args=connect_args
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
    extra_data = Column(JSON, default={})  # Store additional context (renamed from metadata to avoid SQLAlchemy conflict)


class Message(Base):
    """Store individual messages in conversations"""
    __tablename__ = "messages"
    
    id = Column(Integer, primary_key=True, autoincrement=True)
    conversation_id = Column(String, index=True)
    role = Column(String)  # 'user', 'assistant', 'system'
    content = Column(Text)
    selected_text = Column(Text, nullable=True)  # Store selected text context
    extra_data = Column(JSON, default={})  # Store embeddings, sources, etc. (renamed from metadata to avoid SQLAlchemy conflict)
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
