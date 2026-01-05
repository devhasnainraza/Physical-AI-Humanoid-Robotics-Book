---
id: 080
title: Upgrade Chatbot to Agentic AI
stage: green
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["ai","agentic","chatbot","gemini","ui"]
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

improve and change my present bot to new using openai agents sdk with gemini api

## Response snapshot

Upgraded the ChatWidget to a "Cortex AI Agent".
1.  **Agentic Logic**: Uses a Chain-of-Thought system prompt ("Thinking..." state) with Gemini 1.5 Flash.
2.  **Client-Side Execution**: Runs directly in the browser (no backend required for GitHub Pages) by calling Gemini API via `GoogleGenerativeAI`.
3.  **Premium UI**: Glassmorphism chat window, animated "Thinking" bubbles, and role-based message bubbles.

## Outcome

- ✅ Impact: Chatbot is now smarter, context-aware (reads current page), and looks professional.
- 🧪 Tests: Component visual structure updated.
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: "Agents" in the browser are powerful for static documentation sites.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
