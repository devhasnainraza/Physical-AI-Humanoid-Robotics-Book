---
id: 103
title: Upgrade Chat UI to Framework Grade
stage: green
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["ui","chat","syntax-highlighting","polish"]
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

high upgrade chat ui... chatbot ui frameworks

## Response snapshot

Completely refactored `ChatWidget` to mimic high-end AI interfaces (like Vercel AI SDK/ChatGPT).
1.  **Framework-Grade UI**: Switched to a cleaner, less "glassy", more structural design (border-gray-200, solid backgrounds, precise spacing).
2.  **Syntax Highlighting**: Integrated `react-syntax-highlighter` (Prism) for professional code rendering.
3.  **Expansion**: Added a "Full Screen" toggle button.
4.  **User Avatars**: Uses Clerk user initials for user messages.

## Outcome

- ✅ Impact: The Chatbot now looks like a serious developer tool.
- 🧪 Tests: Component visual structure updated.
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Code highlighting is essential for a technical textbook chatbot.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null