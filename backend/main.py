import os
import google.generativeai as genai
from fastapi import FastAPI, HTTPException, Depends
from pydantic import BaseModel
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
from fastapi.middleware.cors import CORSMiddleware
from typing import List, Dict
import uuid

load_dotenv()

app = FastAPI()

# Enable CORS for GitHub Pages
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], # In production, restrict to your domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Config
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

genai.configure(api_key=GEMINI_API_KEY)
model = genai.GenerativeModel('gemini-1.5-flash')

# Database (In-Memory for Demo)
chat_db: Dict[str, List[Dict[str, str]]] = {}

# Qdrant Setup
try:
    if QDRANT_URL:
        qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    else:
        qdrant = QdrantClient(":memory:")
except:
    qdrant = QdrantClient(":memory:")

COLLECTION_NAME = "textbook_knowledge"

class ChatRequest(BaseModel):
    session_id: str
    message: str
    selected_text: str = ""

@app.get("/")
def read_root():
    return {"status": "Physical AI Backend is Running"}

@app.post("/chat")
async def chat(request: ChatRequest):
    # 1. Init Session
    if request.session_id not in chat_db:
        chat_db[request.session_id] = []
    
    # 2. RAG Retrieval
    context = ""
    try:
        embedding = genai.embed_content(
            model="models/embedding-001",
            content=request.message,
            task_type="retrieval_query"
        )['embedding']
        
        hits = qdrant.search(
            collection_name=COLLECTION_NAME,
            query_vector=embedding,
            limit=3
        )
        context = "\n".join([h.payload['text'] for h in hits])
    except:
        pass

    # 3. Construct Prompt
    history = chat_db[request.session_id][-5:] # Last 5 messages
    
    prompt = f"""
    You are Cortex-H1, an advanced AI Robotics Professor.
    
    CONTEXT FROM TEXTBOOK:
    {context}
    
    USER SELECTION: {request.selected_text}
    
    HISTORY:
    {history}
    
    USER: {request.message}
    
    Answer deeply and technically.
    """

    # 4. Generate
    response = model.generate_content(prompt)
    answer = response.text

    # 5. Save History
    chat_db[request.session_id].append({"role": "user", "content": request.message})
    chat_db[request.session_id].append({"role": "model", "content": answer})

    return {"response": answer, "history": chat_db[request.session_id]}

@app.get("/history/{session_id}")
def get_history(session_id: str):
    return chat_db.get(session_id, [])