---
id: 162
title: Fix inputRef is not defined Error
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","chat","ref","reference-error"]
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

ReferenceError: inputRef is not defined

## Response snapshot

1.  **Diagnosis**: The `inputRef` used to focus the input field after voice-to-text was accidentally deleted during a previous refactor.
2.  **Fix**: Re-declared the `inputRef` using `useRef<HTMLTextAreaElement>(null)` at the top of the `ChatWidget` component.

## Outcome

- ✅ Impact: The `startListening` function will no longer crash when trying to focus the input field.
- 🧪 Tests: n/a
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Aggressive refactors can sometimes delete necessary one-line declarations. Always double check ref dependencies.

## Evaluation notes (flywheel)

- Failure modes observed: `ReferenceError`.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
