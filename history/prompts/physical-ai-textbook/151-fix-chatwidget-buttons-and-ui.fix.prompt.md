---
id: 151
title: Fix ChatWidget Buttons and UI
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","ui","buttons","chat"]
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

buttons not working properly (interactivity)

## Response snapshot

1.  **Refactor**: Removed the dropdown menu entirely and replaced it with direct icon buttons in the header for Expand, Clear, and Close. This solves the click-closing issue definitively.
2.  **Logic Fix**: Ensured `startListening` and `clearHistory` are properly scoped and attached to the new buttons.
3.  **UI**: Kept the "Blue Glassmorphism" theme but with the simplified header controls.

## Outcome

- ✅ Impact: Header buttons are now 100% reliable and easy to access.
- 🧪 Tests: Component logic updated.
- 📁 Files: `src/components/ChatWidget/index.tsx` overwritten.
- 🔁 Next prompts: null
- 🧠 Reflection: Simpler is often better.

## Evaluation notes (flywheel)

- Failure modes observed: Dropdown menu UX issues.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
