---
id: 090
title: Replace Chat Logo
stage: green
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["ui","branding","logo","chat"]
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

change my chat logo add new logo cortex-logo.svg

## Response snapshot

Replaced the generic robot emoji in the Chat Widget with the custom `cortex-logo.svg` using `useBaseUrl`. Updated both the floating button and the chat window header.

## Outcome

- ✅ Impact: Consistent branding across Navbar and Chatbot.
- 🧪 Tests: Component visual structure updated.
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: SVGs need `useBaseUrl` in Docusaurus to work correctly in production builds with subpaths.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
