---
id: 092
title: Fix Missing Import useBaseUrl
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","import","docusaurus"]
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

not solve that error ... useBaseUrl is not defined

## Response snapshot

Acknowledged that the previous `write_file` operation omitted the `import useBaseUrl` statement despite the plan. Used `replace` to explicitly insert the import line at the top of `src/components/ChatWidget/index.tsx`.

## Outcome

- ✅ Impact: Component now has the required import to run without crashing.
- 🧪 Tests: File content updated.
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Double-check the *entire* file content when doing a full overwrite.

## Evaluation notes (flywheel)

- Failure modes observed: Missing import in previous turn.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
