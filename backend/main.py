"""
FastAPI Backend for RAG Chatbot
Uses OpenAI ChatKit/Agents SDK, Neon Postgres, and Qdrant Cloud
"""
import os
import uuid
from datetime import datetime
from typing import Optional
from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select

from database import init_db, get_db, Conversation, Message

load_dotenv()

app = FastAPI(title="Physical AI RAG Chatbot API")

# Enable CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, restrict to your domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Configuration
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL", "https://your-cluster.qdrant.io")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

if not OPENAI_API_KEY:
    print("Warning: OPENAI_API_KEY not set")

# Initialize OpenAI client
openai_client = OpenAI(api_key=OPENAI_API_KEY)

# Connect to Qdrant Cloud
try:
    if QDRANT_URL and QDRANT_API_KEY:
        qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
        print(f"Connected to Qdrant Cloud: {QDRANT_URL}")
    else:
        print("Warning: QDRANT_URL or QDRANT_API_KEY not set, using in-memory")
        qdrant = QdrantClient(":memory:")
except Exception as e:
    print(f"Qdrant Connection Error: {e}")
    qdrant = QdrantClient(":memory:")

COLLECTION_NAME = "textbook_knowledge"

# Ensure collection exists
try:
    qdrant.get_collection(COLLECTION_NAME)
except:
    # Create collection with OpenAI embedding size (1536 for text-embedding-3-small)
    qdrant.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=models.VectorParams(
            size=1536,
            distance=models.Distance.COSINE
        ),
    )
    print(f"Created Qdrant collection: {COLLECTION_NAME}")


# Initialize database on startup
@app.on_event("startup")
async def startup_event():
    await init_db()
    print("Database initialized")


# Request/Response Models
class ChatRequest(BaseModel):
    message: str
    selected_text: Optional[str] = ""
    conversation_id: Optional[str] = None
    user_id: Optional[str] = None


class ChatResponse(BaseModel):
    response: str
    conversation_id: str
    sources: Optional[list] = None


class IngestRequest(BaseModel):
    text: str
    metadata: dict


# Routes
@app.get("/")
def read_root():
    return {
        "status": "Physical AI RAG Chatbot Backend",
        "version": "1.0.0",
        "services": {
            "openai": "ChatKit/Agents SDK",
            "qdrant": "Vector Database",
            "neon": "Postgres Database"
        }
    }


@app.post("/ingest", response_model=dict)
async def ingest_text(request: IngestRequest):
    """Ingest text chunks into Qdrant vector database"""
    try:
        # Generate embedding using OpenAI
        embedding_response = openai_client.embeddings.create(
            model="text-embedding-3-small",
            input=request.text
        )
        vector = embedding_response.data[0].embedding

        # Upload to Qdrant
        point_id = abs(hash(request.text)) % (10**16)
        qdrant.upsert(
            collection_name=COLLECTION_NAME,
            points=[
                models.PointStruct(
                    id=point_id,
                    vector=vector,
                    payload={
                        "text": request.text,
                        **request.metadata
                    }
                )
            ]
        )
        return {"status": "ingested", "point_id": point_id}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/chat", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    db: AsyncSession = Depends(get_db)
):
    """
    Main chat endpoint with RAG
    Uses OpenAI ChatKit/Agents SDK with conversation history from Neon Postgres
    """
    # Get or create conversation
    conversation_id = request.conversation_id or str(uuid.uuid4())
    
    # Retrieve conversation history from Neon Postgres
    conversation_query = select(Conversation).where(
        Conversation.id == conversation_id
    )
    result = await db.execute(conversation_query)
    conversation = result.scalar_one_or_none()
    
    if not conversation:
        # Create new conversation
        conversation = Conversation(
            id=conversation_id,
            user_id=request.user_id or "anonymous",
            metadata={}
        )
        db.add(conversation)
        await db.commit()
    
    # Retrieve message history
    messages_query = select(Message).where(
        Message.conversation_id == conversation_id
    ).order_by(Message.created_at)
    result = await db.execute(messages_query)
    history_messages = result.scalars().all()
    
    # Build conversation context for OpenAI
    openai_messages = []
    
    # Add system message with RAG context
    context = ""
    sources = []
    
    # 1. Retrieve relevant context from Qdrant
    try:
        # Generate query embedding
        query_embedding_response = openai_client.embeddings.create(
            model="text-embedding-3-small",
            input=request.message
        )
        query_embedding = query_embedding_response.data[0].embedding

        # Search Qdrant
        search_results = qdrant.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            limit=5  # Get top 5 relevant chunks
        )
        
        context_chunks = []
        for hit in search_results:
            if 'text' in hit.payload:
                context_chunks.append(hit.payload['text'])
                if 'source' in hit.payload:
                    sources.append(hit.payload['source'])
        
        context = "\n\n".join(context_chunks)
    except Exception as e:
        print(f"Retrieval failed: {e}")
        context = "Knowledge base search unavailable."
    
    # Build system prompt with RAG context
    system_prompt = """You are Cortex, an expert AI tutor for Physical AI & Humanoid Robotics.
You help students understand concepts from the textbook using Retrieval-Augmented Generation (RAG).

When answering questions:
1. Use the provided knowledge base context to give accurate, detailed answers
2. If the user has selected specific text, prioritize that context
3. Cite sources when possible
4. Be conversational and educational
5. If information isn't in the context, say so clearly

KNOWLEDGE BASE CONTEXT:
{context}
"""
    
    # If user selected text, add it as high-priority context
    if request.selected_text:
        system_prompt += f"\n\nUSER SELECTED TEXT (HIGH PRIORITY):\n{request.selected_text}\n"
    
    openai_messages.append({
        "role": "system",
        "content": system_prompt.format(context=context)
    })
    
    # Add conversation history
    for msg in history_messages[-10:]:  # Last 10 messages for context
        openai_messages.append({
            "role": msg.role,
            "content": msg.content
        })
    
    # Add current user message
    openai_messages.append({
        "role": "user",
        "content": request.message
    })
    
    # Save user message to database
    user_message = Message(
        conversation_id=conversation_id,
        role="user",
        content=request.message,
        selected_text=request.selected_text,
        metadata={"sources": sources}
    )
    db.add(user_message)
    await db.commit()
    
    # Generate response using OpenAI ChatKit
    try:
        completion = openai_client.chat.completions.create(
            model="gpt-4o-mini",  # Using gpt-4o-mini for cost efficiency
            messages=openai_messages,
            temperature=0.7,
            max_tokens=1000
        )
        
        assistant_response = completion.choices[0].message.content
        
        # Save assistant message to database
        assistant_message = Message(
            conversation_id=conversation_id,
            role="assistant",
            content=assistant_response,
            metadata={"sources": sources}
        )
        db.add(assistant_message)
        await db.commit()
        
        # Update conversation timestamp
        conversation.updated_at = datetime.utcnow()
        await db.commit()
        
        return ChatResponse(
            response=assistant_response,
            conversation_id=conversation_id,
            sources=list(set(sources)) if sources else None
        )
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"OpenAI API error: {str(e)}")


@app.get("/conversations/{conversation_id}")
async def get_conversation(
    conversation_id: str,
    db: AsyncSession = Depends(get_db)
):
    """Retrieve conversation history"""
    messages_query = select(Message).where(
        Message.conversation_id == conversation_id
    ).order_by(Message.created_at)
    result = await db.execute(messages_query)
    messages = result.scalars().all()
    
    return {
        "conversation_id": conversation_id,
        "messages": [
            {
                "role": msg.role,
                "content": msg.content,
                "selected_text": msg.selected_text,
                "created_at": msg.created_at.isoformat()
            }
            for msg in messages
        ]
    }


@app.delete("/conversations/{conversation_id}")
async def delete_conversation(
    conversation_id: str,
    db: AsyncSession = Depends(get_db)
):
    """Delete a conversation"""
    # Delete messages
    messages_query = select(Message).where(
        Message.conversation_id == conversation_id
    )
    result = await db.execute(messages_query)
    messages = result.scalars().all()
    for msg in messages:
        await db.delete(msg)
    
    # Delete conversation
    conversation_query = select(Conversation).where(
        Conversation.id == conversation_id
    )
    result = await db.execute(conversation_query)
    conversation = result.scalar_one_or_none()
    if conversation:
        await db.delete(conversation)
    
    await db.commit()
    return {"status": "deleted", "conversation_id": conversation_id}
