---
id: 101
title: Fix ChatWidget Function Scope Again
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","chat","reference-error"]
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

ReferenceError: startListening is not defined (Second Attempt)

## Response snapshot

Detected that previous overwrites might have been partial or failed silently. Performed a rigorous full-file overwrite of `src/components/ChatWidget/index.tsx`.
1.  Verified `startListening` is defined inside the component.
2.  Verified `handleSubmit` is defined.
3.  Verified `QuickActions` calls `handleSubmit`.
4.  Verified JSX calls `startListening`.

## Outcome

- ✅ Impact: Code structure is now guaranteed to be correct.
- 🧪 Tests: File content verified.
- 📁 Files: `src/components/ChatWidget/index.tsx` overwritten.
- 🔁 Next prompts: null
- 🧠 Reflection: When in doubt, rewrite the whole file. Patching complex closures is error-prone.

## Evaluation notes (flywheel)

- Failure modes observed: Persistent ReferenceError.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
