# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `001-physical-ai-textbook` | **Date**: 2025-12-13 | **Spec**: [specs/001-physical-ai-textbook/spec.md](../spec.md)
**Input**: Feature specification from `specs/001-physical-ai-textbook/spec.md`

## Summary

Create a Docusaurus-based educational platform ("Cortex-H1") featuring a 4-module syllabus, hardware-aware content personalization ("Unitree Go2" support), a context-aware RAG chatbot using OpenAI ChatKit/Qdrant, and on-the-fly Agentic Translation via Gemini 2.0 Flash.

## Technical Context

**Language/Version**: TypeScript 5.x, Python 3.10+ (Ingestion)
**Primary Dependencies**: 
- Framework: Docusaurus 3.x
- Auth: Better Auth (PostgreSQL)
- AI: OpenAI ChatKit, LangChain, Google Generative AI SDK
- Vector DB: Qdrant Client
**Storage**: PostgreSQL (Users), Qdrant (Vectors)
**Testing**: Jest, React Testing Library
**Target Platform**: Web (Responsive)
**Project Type**: Web Application (Static Site Generator + Dynamic API routes)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Article I (Co-Learning)**: Syllabus structure matches requirements.
- **Article II (Dynamic Personalization)**: Implemented via `HardwareOnly` component and Better Auth profile.
- **Article III (Tech Stack)**: Docusaurus, Better Auth, ChatKit/Qdrant/LangChain all selected.

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-textbook/
├── plan.md              # This file
├── research.md          # Technology confirmation
├── data-model.md        # User and RAG schemas
├── quickstart.md        # Setup guide
├── contracts/           # API definitions
└── tasks.md             # (To be created)
```

### Source Code (repository root)

```text
# Docusaurus Project Structure
/
├── docs/                      # Syllabus Content (MDX)
│   ├── module-1-ros2/
│   ├── module-2-digital-twin/
│   ├── module-3-ai-brain/
│   └── module-4-vla/
├── src/
│   ├── components/
│   │   ├── HardwareOnly/      # Personalization Component
│   │   ├── ChatWidget/        # RAG Chatbot
│   │   └── Translator/        # Translation Button
│   ├── pages/
│   └── theme/                 # Custom Layout wrappers
├── api/                       # Serverless Functions (or Next.js/Express proxy)
│   ├── auth/                  # Better Auth routes
│   ├── chat/                  # RAG endpoint
│   └── translate/             # Translation endpoint
├── scripts/
│   └── rag_ingest.py          # Python Ingestion Script
└── docusaurus.config.ts
```

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Custom Auth in Static Site | Dynamic Personalization | Static site cannot handle per-user content hiding securely/effectively |
| Python Script + Node App | RAG Ecosystem | Python has better library support (LangChain/LlamaIndex) for RAG ingestion |
