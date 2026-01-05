---
id: 091
title: Fix useBaseUrl Undefined Error
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","import","docusaurus","react"]
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

This page crashed. useBaseUrl is not defined

## Response snapshot

Fixed the `ReferenceError` in `ChatWidget`.
1.  Added `import useBaseUrl from '@docusaurus/useBaseUrl';` at the top of the file.
2.  Ensured `useBaseUrl` is called inside the component function body.

## Outcome

- ✅ Impact: ChatWidget now correctly resolves the logo path without crashing.
- 🧪 Tests: Component updated.
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Importing Docusaurus helpers is required before use.

## Evaluation notes (flywheel)

- Failure modes observed: Missing import.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
