---
id: 081
title: Configure Chatbot API Key and UI
stage: green
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["config","api-key","ui","responsive"]
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

user not use personal api key ... improve ui make responsive

## Response snapshot

1.  **API Key**: Hardcoded the API Key logic to use a variable `apiKey` (placeholder) instead of prompting the user. Added security warning comments.
2.  **Responsive UI**: Updated CSS to use `w-[90vw] md:w-[400px]` and `h-[80vh]`, ensuring the chat window fits perfectly on mobile screens.
3.  **UI Polish**: Added "Online" status indicator, smoother animations (`animate-scale-in`), and refined typography.

## Outcome

- ✅ Impact: Chatbot is now plug-and-play for visitors (once key is added) and mobile-friendly.
- 🧪 Tests: Component logic updated.
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Hardcoding keys on client-side is risky but often necessary for zero-config hackathon demos.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
