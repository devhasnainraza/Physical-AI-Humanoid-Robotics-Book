# Quickstart: Physical AI Textbook

## Prerequisites
- Node.js 18+
- Python 3.10+ (for RAG ingestion)
- Docker (for Qdrant)
- OpenAI API Key
- Google Gemini API Key
- PostgreSQL Database URL

## Installation

1. **Install Dependencies**
   ```bash
   npm install
   ```

2. **Environment Setup**
   Copy `.env.example` to `.env.local`:
   ```env
   BETTER_AUTH_SECRET=...
   BETTER_AUTH_URL=http://localhost:3000
   DATABASE_URL=postgresql://...
   OPENAI_API_KEY=...
   GEMINI_API_KEY=...
   QDRANT_URL=http://localhost:6333
   ```

3. **Start Database Services**
   ```bash
   docker run -p 6333:6333 qdrant/qdrant
   ```

4. **Run Development Server**
   ```bash
   npm run start
   ```

## RAG Ingestion (Manual)

To index the current docs:
```bash
python scripts/rag_ingest.py --docs ./docs --collection textbook_knowledge
```
