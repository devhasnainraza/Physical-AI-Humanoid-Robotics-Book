---
id: 102
title: Add Personalization and TTS to Chatbot
stage: green
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["feature","personalization","tts","ui"]
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

Upgraded the Chatbot with "Next-Gen" features:
1.  **User Personalization**: Connects with Clerk Auth to greet the user by name ("Hi Hasnain!").
2.  **Speech Synthesis (TTS)**: Added a "Speak" button to read answers aloud using the browser's speech engine.
3.  **UI Polish**: Refined glassmorphism header and added a clear TTS toggle icon.

## Outcome

- ✅ Impact: Chatbot feels personalized and accessible.
- 🧪 Tests: Component visual structure updated.
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Personalization increases engagement significantly.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
