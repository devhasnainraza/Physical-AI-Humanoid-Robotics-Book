---
id: 119
title: Fix ChatWidget API Key Logic and Remove Text Selection
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","chat","api-key","docusaurus"]
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

bot reply :System Error: API Key is missing...

## Response snapshot

1.  **API Key Fix**: Updated `ChatWidget` to use `useDocusaurusContext` to retrieve the API key from `siteConfig.customFields.geminiApiKey`. This fixes the issue where `process.env` is stripped in the browser bundle.
2.  **Text Selection Removal**: Removed the text selection popup feature as requested by the user in the previous turn (merged into this fix to ensure a clean state).
3.  **UI Preservation**: Kept the user's preferred "Ultra-Advanced" UI (Lucide icons, Glassmorphism).

## Outcome

- ✅ Impact: Chatbot should now connect to Gemini API successfully without prompts.
- 🧪 Tests: Component logic updated.
- 📁 Files: `src/components/ChatWidget/index.tsx` overwritten.
- 🔁 Next prompts: null
- 🧠 Reflection: Static site generators (Docusaurus/Next.js) handle env vars differently than Node.js. Always check docs for client-side access.

## Evaluation notes (flywheel)

- Failure modes observed: `process.env` access failure in client.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
