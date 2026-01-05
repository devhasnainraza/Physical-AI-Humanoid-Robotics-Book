---
id: 064
title: Fix Auth Crash
stage: green
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","crash","auth"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - src/components/Auth/AuthUI.tsx
tests:
 - npm run build (PASS)
---

## Prompt

This page crashed... Cannot read properties of undefined (reading 'image')

## Response snapshot

Diagnosed that `session` might be truthy but `session.user` undefined in some edge cases with Better Auth client. Added `session && session.user` check to `AuthUI.tsx` to prevent the crash.

## Outcome

- ✅ Impact: Site should load without crashing even if auth state is partial.
- 🧪 Tests: Component updated.
- 📁 Files: `src/components/Auth/AuthUI.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Always optional chain (`?.`) nested properties from external APIs.

## Evaluation notes (flywheel)

- Failure modes observed: `session.user` undefined access.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
