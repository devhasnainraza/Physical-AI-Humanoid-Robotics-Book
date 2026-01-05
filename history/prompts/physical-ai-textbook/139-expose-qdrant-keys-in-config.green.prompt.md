---
id: 139
title: Expose Qdrant Keys in Config
stage: green
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["config","qdrant","env"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - docusaurus.config.ts
tests:
 - npm run start (Manual)
---

## Prompt

applying full code for qdrant vector db...

## Response snapshot

Updated `docusaurus.config.ts` to expose `QDRANT_URL` and `QDRANT_API_KEY` to the client-side bundle via `customFields`. This allows the `ChatWidget` to make direct REST API calls to the Qdrant Cloud instance for RAG search.

## Outcome

- ✅ Impact: Frontend can now authenticate with Qdrant.
- 🧪 Tests: Configuration updated.
- 📁 Files: `docusaurus.config.ts` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Client-side RAG requires exposing read-only keys or careful CORS management.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
