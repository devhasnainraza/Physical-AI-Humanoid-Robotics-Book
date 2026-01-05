---
id: 118
title: Remove API Key Prompt
stage: green
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["config","security","chat","api-key"]
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

yar yeh feature remove karo ke user chat ke liye apni apikey dale...

## Response snapshot

Removed the interactive `prompt()` for the API key in `ChatWidget`.
1.  **Logic Update**: Now strictly checks `process.env.GEMINI_API_KEY` (and `localStorage` as a silent fallback).
2.  **Error Handling**: If the key is missing, it displays a system error message in the chat bubble instead of blocking the UI with a browser prompt.

## Outcome

- ✅ Impact: User experience is seamless; they don't know an API key is involved unless it fails.
- 🧪 Tests: Component logic updated.
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Hardcoding keys or relying on prompts is bad UX for production apps. Env injection is the way.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
