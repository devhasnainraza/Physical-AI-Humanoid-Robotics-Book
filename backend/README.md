# RAG Chatbot Backend

FastAPI backend for the Integrated RAG Chatbot feature, using OpenAI ChatKit/Agents SDK, Neon Serverless Postgres, and Qdrant Cloud.

## Features

- **RAG (Retrieval-Augmented Generation)**: Answers questions using textbook content from Qdrant vector database
- **Conversation Management**: Stores chat history in Neon Postgres
- **Selected Text Support**: Answers questions based on user-selected text from the book
- **OpenAI Integration**: Uses OpenAI ChatKit/Agents SDK for intelligent responses

## Prerequisites

- Python 3.10+
- Neon Serverless Postgres database
- Qdrant Cloud account (Free Tier)
- OpenAI API key

## Setup

### 1. Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### 2. Environment Variables

Create a `.env` file in the `backend/` directory:

```env
# OpenAI API Key (required)
OPENAI_API_KEY=sk-...

# Qdrant Cloud Configuration (required)
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Neon Postgres Database (required)
DATABASE_URL=postgresql+asyncpg://user:password@ep-xxx.us-east-2.aws.neon.tech/dbname?sslmode=require

# Optional: Gemini API (if you want to keep Gemini as fallback)
GEMINI_API_KEY=your-gemini-key
```

### 3. Initialize Database

The database tables will be created automatically on first startup. Alternatively, you can run:

```python
from database import init_db
import asyncio

asyncio.run(init_db())
```

### 4. Run the Server

```bash
uvicorn main:app --reload --port 8000
```

The API will be available at `http://localhost:8000`

## API Endpoints

### `GET /`
Health check endpoint.

### `POST /chat`
Main chat endpoint with RAG.

**Request:**
```json
{
  "message": "What is ROS 2?",
  "selected_text": "Optional: user-selected text from the book",
  "conversation_id": "Optional: existing conversation ID",
  "user_id": "Optional: user identifier"
}
```

**Response:**
```json
{
  "response": "ROS 2 is...",
  "conversation_id": "uuid-here",
  "sources": ["docs/module-1-ros2/01-intro.md"]
}
```

### `POST /ingest`
Ingest text chunks into Qdrant vector database.

**Request:**
```json
{
  "text": "Text content to embed",
  "metadata": {
    "source": "docs/module-1/intro.md",
    "title": "Introduction"
  }
}
```

### `GET /conversations/{conversation_id}`
Retrieve conversation history.

### `DELETE /conversations/{conversation_id}`
Delete a conversation.

## Database Schema

### `conversations` table
- `id` (String, Primary Key): Conversation UUID
- `user_id` (String): User identifier
- `created_at` (DateTime): Creation timestamp
- `updated_at` (DateTime): Last update timestamp
- `metadata` (JSON): Additional context

### `messages` table
- `id` (Integer, Primary Key): Auto-increment ID
- `conversation_id` (String, Indexed): Foreign key to conversations
- `role` (String): 'user', 'assistant', or 'system'
- `content` (Text): Message content
- `selected_text` (Text, Nullable): Selected text context
- `metadata` (JSON): Sources, embeddings, etc.
- `created_at` (DateTime): Message timestamp

## RAG Workflow

1. **Query Embedding**: User question is embedded using OpenAI `text-embedding-3-small`
2. **Vector Search**: Qdrant searches for top 5 relevant chunks
3. **Context Building**: Retrieved chunks + selected text (if any) are combined
4. **LLM Generation**: OpenAI GPT-4o-mini generates response with context
5. **Storage**: Conversation and messages are saved to Neon Postgres

## Testing

Test the API with curl:

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is Physical AI?",
    "selected_text": ""
  }'
```

## Deployment

For production deployment:

1. Set environment variables in your hosting platform
2. Ensure Neon Postgres is accessible from your server
3. Configure CORS origins in `main.py` (currently allows all)
4. Use a production ASGI server like Gunicorn with Uvicorn workers:

```bash
gunicorn main:app -w 4 -k uvicorn.workers.UvicornWorker
```

## Troubleshooting

### Qdrant Connection Issues
- Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct
- Check that your Qdrant cluster is running
- Ensure the collection `textbook_knowledge` exists

### Database Connection Issues
- Verify `DATABASE_URL` format (must include `+asyncpg`)
- Check SSL mode is set to `require`
- Ensure Neon database is not paused

### OpenAI API Issues
- Verify `OPENAI_API_KEY` is valid
- Check API rate limits
- Ensure you have credits in your OpenAI account
