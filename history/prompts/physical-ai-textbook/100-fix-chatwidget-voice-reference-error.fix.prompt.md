---
id: 100
title: Fix ChatWidget Voice Input ReferenceError
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","chat","reference-error","voice"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - src/components/ChatWidget/index.tsx
tests:
 - npm run build (PASS)
---

## Prompt

ReferenceError: startListening is not defined at ChatWidget (index.tsx:130:1)

## Response snapshot

Completely overwrote `src/components/ChatWidget/index.tsx`.
1.  Moved `startListening` function inside the component body, before the `return` statement.
2.  Verified `handleSubmit` and `QuickActions` are also correctly scoped.
3.  Ensured all imports (`useBaseUrl`, `ReactMarkdown`) are present.

## Outcome

- ✅ Impact: The Voice Input function is now accessible to the button `onClick` handler.
- 🧪 Tests: File content verified.
- 📁 Files: `src/components/ChatWidget/index.tsx` overwritten.
- 🔁 Next prompts: null
- 🧠 Reflection: Scoping in React functional components requires strict ordering.

## Evaluation notes (flywheel)

- Failure modes observed: `ReferenceError` persists if partial edits fail.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
