---
id: 131
title: Fix Gemini Model Version 404
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","ai","model","api"]
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

Error: [404] models/gemini-1.5-flash is not found

## Response snapshot

Updated the Gemini model identifier in `ChatWidget`.
1.  **Issue**: The alias `gemini-1.5-flash` was returning 404 on the `v1beta` API endpoint.
2.  **Fix**: Switched to the specific version tag `gemini-1.5-flash-001`, which is the canonical ID for the stable release.

## Outcome

- ✅ Impact: API calls should now resolve correctly.
- 🧪 Tests: n/a
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Model aliases often change or are region-specific. Pinning versions (`-001`) is safer for production.

## Evaluation notes (flywheel)

- Failure modes observed: Model alias 404.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
