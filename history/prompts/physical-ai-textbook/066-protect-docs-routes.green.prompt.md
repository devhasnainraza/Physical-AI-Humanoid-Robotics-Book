---
id: 066
title: Protect Documentation Routes
stage: green
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["auth","security","protection","routes"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - src/theme/Layout/index.tsx
tests:
 - npm run build (PASS)
---

## Prompt

user not login or sign up so he can't access my web

## Response snapshot

Implemented Route Protection in `src/theme/Layout/index.tsx`.
1.  **Check**: On load, verify if a session exists (Real API or Mock LocalStorage).
2.  **Gate**: If URL contains `/docs/` and user is unauthenticated, render an "Access Restricted" screen instead of content.
3.  **Allow**: Homepage (`/`) remains accessible for sign-up.

## Outcome

- ✅ Impact: Content is now gated behind the login wall.
- 🧪 Tests: Component logic updated.
- 📁 Files: `src/theme/Layout/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Client-side gating is sufficient for a static educational site, though not "secure" against curl/inspect element.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
