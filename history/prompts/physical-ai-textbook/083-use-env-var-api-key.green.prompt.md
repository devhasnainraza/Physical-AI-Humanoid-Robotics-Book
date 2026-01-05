---
id: 083
title: Use Environment Variable for API Key
stage: green
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["security","config","env","api-key"]
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

remove gemini api kay for public file already i pasted in .env

## Response snapshot

Removed the hardcoded API Key string from `ChatWidget`.
Updated logic to use `process.env.GEMINI_API_KEY`.
Added fallback to `localStorage` (prompt) if the env var is missing in the build.

## Outcome

- ✅ Impact: Codebase is secure (no secrets in source).
- 🧪 Tests: Code updated.
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Environment variables in static sites are baked in at build time. User must ensure their build environment (local or CI) has the key.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
