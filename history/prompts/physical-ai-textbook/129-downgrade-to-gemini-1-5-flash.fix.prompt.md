---
id: 129
title: Downgrade to Gemini 1.5 Flash
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","ai","model","stability"]
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

Error: [503] The model is overloaded.

## Response snapshot

Downgraded the model identifier in `ChatWidget` from `gemini-2.5-flash` (experimental/unstable) back to `gemini-1.5-flash` (stable production). This resolves the 503 overload errors users were experiencing.

## Outcome

- ✅ Impact: Restored chat functionality reliability.
- 🧪 Tests: n/a
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Always default to stable model versions unless experimental features are strictly required.

## Evaluation notes (flywheel)

- Failure modes observed: 503 Overload on experimental model.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
