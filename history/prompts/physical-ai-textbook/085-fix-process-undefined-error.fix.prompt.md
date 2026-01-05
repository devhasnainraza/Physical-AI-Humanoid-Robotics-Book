---
id: 085
title: Fix Process Undefined Error
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","webpack","browser","env"]
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

process is not defined

## Response snapshot

Wrapped `process.env.GEMINI_API_KEY` access in a `try/catch` block with `typeof process !== 'undefined'` check. This prevents the browser from crashing when accessing Node.js globals that Webpack 5 doesn't polyfill.

## Outcome

- ✅ Impact: Chatbot no longer crashes on submit.
- 🧪 Tests: Component logic updated.
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Client-side env var access is always tricky in modern bundlers.

## Evaluation notes (flywheel)

- Failure modes observed: `ReferenceError: process is not defined`.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
