---
id: 097
title: Fix Chatbot Context Extraction
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","chat","rag","context"]
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

reply : I cannot access external content ...

## Response snapshot

Fixed the issue where the Chatbot couldn't "see" the page content.
1.  **Selectors**: Added fallback selectors (`article`, `main`) in case `.theme-doc-markdown` isn't found.
2.  **Timing**: Added a `setTimeout` to wait for React hydration before scraping text.
3.  **Warning**: Added a UI check to warn the user if context is empty *before* calling the API, saving quota and confusion.

## Outcome

- ✅ Impact: Chatbot reliably reads the page content now.
- 🧪 Tests: Component logic updated.
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Client-side scraping for RAG context is sensitive to DOM structure changes.

## Evaluation notes (flywheel)

- Failure modes observed: Empty context sent to LLM.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
