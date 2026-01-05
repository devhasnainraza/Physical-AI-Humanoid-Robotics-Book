---
id: 150
title: Disable Neon Pipeline Connect
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","db","neon","localhost-error"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - src/lib/db.ts
tests:
 - npm run build (PASS)
---

## Prompt

POST https://localhost/sql net::ERR_CONNECTION_REFUSED (Persistence)

## Response snapshot

The user provided another screenshot showing the `localhost/sql` error persisting.
1.  **Root Cause**: The `@neondatabase/serverless` driver has a `pipelineConnect` feature that tries to open a WebSocket or special connection, which defaults to localhost in dev environments if not explicitly disabled or configured.
2.  **Fix**: Updated `src/lib/db.ts` to set `neonConfig.pipelineConnect = false`. This forces the driver to use standard HTTP fetch requests to the endpoint defined in `fetchEndpoint`, preventing it from trying any "smart" local connection logic.

## Outcome

- ✅ Impact: Should force the driver to use the Cloud HTTP API strictly.
- 🧪 Tests: n/a
- 📁 Files: `src/lib/db.ts` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Serverless drivers in browser environments are tricky. "Pipeline" modes often fail on client-side due to WebSocket restrictions or env detection. Standard HTTP is safer.

## Evaluation notes (flywheel)

- Failure modes observed: `localhost` fetch attempts.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
