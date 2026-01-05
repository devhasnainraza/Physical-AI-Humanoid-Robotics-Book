---
id: 106
title: Personalize Chat Greeting
stage: green
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["ai","chat","personalization","clerk"]
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

bhai yeh har chat par srf muhammad ke name se hello baat kar raha hai...

## Response snapshot

User complained that the bot wasn't addressing them by their registered name. Added a `useEffect` hook to dynamically update the initial greeting message based on the `user.firstName` from Clerk, ensuring it says "Hello [Name]!" instead of a generic greeting.

## Outcome

- ✅ Impact: Immediate personalization upon opening the chat.
- 🧪 Tests: Component logic updated.
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Static initial state doesn't work for dynamic auth data; must use effects.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
