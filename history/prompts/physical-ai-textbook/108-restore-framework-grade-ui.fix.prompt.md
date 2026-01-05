---
id: 108
title: Restore Framework Grade UI with Fixes
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","chat","ui","features"]
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

improve my bot very very build as a advanced functionality

## Response snapshot

Restored the "Framework-Grade" UI (Turn 103) which was the most advanced visual iteration (Syntax Highlighting, Full Screen, Clean Dark Mode) but merged in the fixes for `startListening` and `handleSubmit` that were lost in the "Sober" redesign. Added TTS (Text-to-Speech) logic inside this robust UI frame.

## Outcome

- ✅ Impact: Best of both worlds: Professional UI + Advanced Functionality (Voice/TTS).
- 🧪 Tests: Component visual structure updated.
- 📁 Files: `src/components/ChatWidget/index.tsx` overwritten.
- 🔁 Next prompts: null
- 🧠 Reflection: Iterative design sometimes regresses functionality. Merging known-good states is key.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
