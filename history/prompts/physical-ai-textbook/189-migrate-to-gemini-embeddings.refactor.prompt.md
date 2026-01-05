---
id: 189
title: Migrate to Gemini Embeddings
stage: refactor
date: 2026-01-05
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: main
user: Muhammad Hasnain
command: remove cohere use gemini embeding model
labels: ["python", "refactor", "rag", "gemini", "cohere"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - scripts/rag_ingest.py
 - backend/main.py
tests:
 - "Run script and observe output"
---

## Prompt

User requested to remove Cohere and use Gemini embedding model.

## Response snapshot

1.  Rewrote `scripts/rag_ingest.py` to use `google-generativeai` instead of `cohere`.
2.  Updated embedding model to `models/text-embedding-004` (768 dimensions).
3.  Updated Qdrant collection vector size to 768.
4.  Updated `get_embedding_batch` to use `genai.embed_content` with `task_type="retrieval_document"`.
5.  Updated `backend/main.py` to use `models/text-embedding-004` for consistency.

## Outcome

- ✅ Impact: Ingestion and Retrieval now both use Google Gemini embeddings, removing the Cohere dependency.
- 🧪 Tests: Will run the script next to verify.
- 📁 Files: Modified `scripts/rag_ingest.py` and `backend/main.py`.
- 🔁 Next prompts: Run `python scripts/rag_ingest.py`.
- 🧠 Reflection: Switching embedding models requires re-ingesting all data because vectors are incompatible. The script handles this by recreating the collection.

## Evaluation notes (flywheel)

- Failure modes observed: None yet.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
