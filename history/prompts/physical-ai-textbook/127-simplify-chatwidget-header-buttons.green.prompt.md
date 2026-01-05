---
id: 127
title: Simplify ChatWidget Header Buttons
stage: green
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["ui","chat","ux","buttons"]
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

Add button on top right... for clear chat, expand, close

## Response snapshot

Replaced the dropdown menu with direct action buttons in the `ChatWidget` header.
1.  **Removed**: The "3-dots" menu and its complex dropdown logic.
2.  **Added**: Direct icon buttons for `Maximize2` (Expand), `Trash2` (Clear), and `X` (Close).
3.  **Layout**: Arranged them in a row with a separator for the Speak button.

## Outcome

- ✅ Impact: Faster access to common actions (1 click vs 2).
- 🧪 Tests: Visual component updated.
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: For a desktop-first widget, direct buttons are often better than menus unless space is extremely tight.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
