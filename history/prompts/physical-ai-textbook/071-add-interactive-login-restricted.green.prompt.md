---
id: 071
title: Add Interactive Login to Restricted Page
stage: green
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["auth","ux","access-control","polish"]
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

change restricted page to add get started button same here

## Response snapshot

Upgraded the "Access Restricted" page in `src/theme/Layout/index.tsx`.
1.  **Interactive Button**: Replaced the passive "Click top right" text with a large, primary "Login / Sign Up" button that opens the `AuthModal` directly.
2.  **Premium UI**: Added a large lock icon, gradient headline, and responsive layout.
3.  **Back Home**: Added a secondary button to return to the homepage.

## Outcome

- ✅ Impact: Users stuck on the restricted page can now log in immediately without hunting for the navbar button.
- 🧪 Tests: Component visual structure updated.
- 📁 Files: `src/theme/Layout/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Reducing friction at "gate" pages improves conversion.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
