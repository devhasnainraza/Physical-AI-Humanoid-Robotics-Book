---
id: 072
title: Enhance Mock Social Login
stage: green
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["auth","mock","ux","simulation"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - src/lib/auth-client.ts
tests:
 - npm run build (PASS)
---

## Prompt

It's auth with social not working properly... it direct given access

## Response snapshot

User confused by the instant "Fake Login". Improved the `mockAuthClient` to spawn a realistic "Connecting to Google..." popup window that stays open for 2.5 seconds before closing and logging the user in. This mimics the actual OAuth flow UX.

## Outcome

- ✅ Impact: Mock Auth feels much more like a real production app.
- 🧪 Tests: Component logic updated.
- 📁 Files: `src/lib/auth-client.ts` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Simulation fidelity matters for demos. Instant success feels broken; delayed success feels real.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
