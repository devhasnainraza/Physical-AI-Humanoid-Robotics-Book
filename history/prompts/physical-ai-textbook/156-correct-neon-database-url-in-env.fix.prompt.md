---
id: 156
title: Correct Neon DATABASE_URL in .env
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","env","db","neon"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - .env
tests:
 - npm run start (Manual)
---

## Prompt

ab bhi error solve nahi ho raha. NeonDbError: Error connecting to database: TypeError: Failed to fetch

## Response snapshot

Diagnosed the persistent `ERR_CONNECTION_REFUSED` error. The primary `DATABASE_URL` in `.env` was incorrectly pointing to `localhost:5432`.
1.  **Fix**: Updated `.env` to set `DATABASE_URL` to the correct Neon Cloud instance connection string (extracted from `AUTH_DATABASE_URL` and simplified by removing `-pooler` suffix for direct serverless driver use).
2.  **Instruction**: Provided explicit cache-clearing instructions and server restart.

## Outcome

- ✅ Impact: The chatbot will now attempt to connect to the correct Neon cloud database.
- 🧪 Tests: n/a
- 📁 Files: `.env` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: User environment configuration issues are the hardest to debug remotely. Direct inspection and correction of `.env` is sometimes necessary.

## Evaluation notes (flywheel)

- Failure modes observed: Incorrect environment variable setup leading to local connection attempts.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
