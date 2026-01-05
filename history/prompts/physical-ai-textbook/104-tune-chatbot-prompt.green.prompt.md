---
id: 104
title: Tune Chatbot Prompt for Natural Conversation
stage: green
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["ai","prompt-engineering","chat","ux"]
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

make chat more better ... bot reply me long lecture on Hi

## Response snapshot

Adjusted the System Prompt in `ChatWidget`. Added explicit instruction: "If the user says 'Hi', greet briefly. DO NOT summarize the page unless asked." This prevents the "Professor Mode" from overwhelming casual greetings.

## Outcome

- ✅ Impact: Chatbot feels more human and less robotic/over-eager.
- 🧪 Tests: Prompt logic updated.
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Context injection is powerful but needs guardrails to prevent hallucinated relevance.

## Evaluation notes (flywheel)

- Failure modes observed: Bot over-indexing on context for simple greetings.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
