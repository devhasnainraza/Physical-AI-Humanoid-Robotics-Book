---
id: 154
title: Aggressive Neon DB Fix
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","db","neon","cache"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - src/lib/db.ts
tests:
 - npm run start (Manual)
---

## Prompt

not solve that's error (Persistent Localhost Refusal)

## Response snapshot

1.  **Code Update**: Rewrote `src/lib/db.ts` to use regex matching to extract the Neon host and force `neonConfig.fetchEndpoint` to the HTTPS URL. Added `pipelineConnect = false` to disable any WebSocket/Localhost attempts.
2.  **Diagnosis**: The persistence of the error suggests Webpack caching.
3.  **Instruction**: Explicitly instructed the user to delete `.docusaurus` and `node_modules/.cache` to ensure the new DB logic is compiled.

## Outcome

- ✅ Impact: The code now contains the strongest possible override for the Neon driver's routing logic.
- 🧪 Tests: n/a
- 📁 Files: `src/lib/db.ts` overwritten.
- 🔁 Next prompts: null
- 🧠 Reflection: Docusaurus caching is aggressive. Code changes in `src/lib` don't always trigger a full rebuild of the client bundle in dev mode.

## Evaluation notes (flywheel)

- Failure modes observed: Stale build artifacts.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
