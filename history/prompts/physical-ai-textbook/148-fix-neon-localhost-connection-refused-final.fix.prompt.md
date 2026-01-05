---
id: 148
title: Fix Neon Localhost Connection Refused Final
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","db","neon","cors","localhost"]
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

POST https://localhost/sql net::ERR_CONNECTION_REFUSED

## Response snapshot

The user provided a screenshot showing that the Neon driver was still trying to POST to `localhost/sql`, despite previous fixes.
1.  **Diagnosis**: The driver's internal logic for `localhost` detection is persistent.
2.  **Fix**: Updated `src/lib/db.ts` to manually parse the `connectionString`, extract the real hostname (e.g., `ep-xyz.aws.neon.tech`), and explicitly set `neonConfig.fetchEndpoint` to that HTTPS URL. This overrides any automatic local proxy logic.

## Outcome

- ✅ Impact: Requests will now go to the actual cloud database, bypassing the local refusal error.
- 🧪 Tests: Logic verified against Neon documentation for custom endpoints.
- 📁 Files: `src/lib/db.ts` overwritten.
- 🔁 Next prompts: null
- 🧠 Reflection: When "smart" SDKs try to help by auto-detecting environments, they often break hybrid setups. Explicit configuration is safer.

## Evaluation notes (flywheel)

- Failure modes observed: `ERR_CONNECTION_REFUSED` due to SDK default behavior.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
