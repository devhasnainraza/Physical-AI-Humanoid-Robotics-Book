# Physical AI & Humanoid Robotics Textbook (Cortex-H1)

Official textbook platform for the "Certified Agentic AI & Robotics Engineer" course.

## Features
- **4-Module Syllabus**: ROS 2, Digital Twin, AI-Robot Brain, VLA.
- **Hardware Profiling**: Content adapts for Unitree Go2, Jetson Orin, or Cloud Simulation.
- **RAG Chatbot**: Ask questions about the content (Context-Aware).
- **Agentic Translation**: Translate pages to Urdu/Hindi on-the-fly.

## Prerequisites
- Node.js 18+
- Docker (for Qdrant/Postgres)
- OpenAI & Gemini API Keys

## Setup

1. **Install Dependencies**
   ```bash
   npm install
   ```

2. **Environment Variables**
   Copy `.env.local` and fill in your keys.

3. **Start Infrastructure**
   ```bash
   docker-compose up -d
   ```

4. **Initialize Knowledge Base**
   ```bash
   npm run qdrant:setup
   # or
   node scripts/setup_qdrant.js
   ```

5. **Start Development Server**
   ```bash
   npm start
   ```

## RAG Ingestion
To index new content:
```bash
python scripts/rag_ingest.py
```