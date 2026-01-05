---
id: 121
title: Fix Docusaurus Env Loading
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","config","dotenv","docusaurus"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - docusaurus.config.ts
tests:
 - npm run start (Manual)
---

## Prompt

System Error: API Key is missing... @.env

## Response snapshot

User reported that the API key was still missing despite being in `.env`.
1.  **Diagnosis**: Docusaurus configuration does not automatically load `.env` files in `docusaurus.config.ts` unless `dotenv` is imported.
2.  **Fix**: Added `import 'dotenv/config';` to the top of `docusaurus.config.ts`.

## Outcome

- ✅ Impact: `process.env.GEMINI_API_KEY` will now be populated from the `.env` file during the build/start process.
- 🧪 Tests: Configuration updated.
- 📁 Files: `docusaurus.config.ts` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Node.js scripts (like config files) always need explicit env loading.

## Evaluation notes (flywheel)

- Failure modes observed: Missing env vars in build config.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
