---
id: 111
title: Add 3-Dots Menu to Chatbot
stage: green
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["ui","chat","menu","ux"]
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

mujhe chat ki navbar esi banake dou top right side pe 3 dots hon

## Response snapshot

Implemented a "Kebab Menu" (3-dots) in the Chat Widget header.
1.  **Menu Dropdown**: Replaced standalone icons with a clean dropdown containing "Minimize", "Clear History", and "Close".
2.  **UI Polish**: Matched the menu style to the rest of the minimalist theme (white/dark mode, border radius).
3.  **Click Outside**: Added logic to close the menu when clicking away.

## Outcome

- ✅ Impact: Header is less cluttered and more professional.
- 🧪 Tests: Component logic updated.
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Hiding secondary actions behind a menu is a standard pattern for clean UIs.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
