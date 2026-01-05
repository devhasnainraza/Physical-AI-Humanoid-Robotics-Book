---
id: 079
title: Fix Clerk Build Errors
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","build","clerk","cleanup"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - src/pages/login.tsx (deleted)
 - src/pages/signup.tsx (deleted)
 - src/pages/forgot-password.tsx (deleted)
 - src/components/HardwareOnly/index.tsx
tests:
 - npm run build (PASS)
---

## Prompt

ERROR in ./src/components/HardwareOnly... Module not found: Error: Can't resolve '../../lib/auth-client'

## Response snapshot

Fixed build errors caused by deleting `auth-client.ts`.
1.  **Deleted** obsolete `src/pages/` auth files that were referencing the old client.
2.  **Refactored** `HardwareOnly` component to use Clerk's `useUser()` hook and `unsafeMetadata` instead of the old client.

## Outcome

- ✅ Impact: Project builds cleanly with Clerk integration.
- 🧪 Tests: File cleanup + Code refactor.
- 📁 Files: 3 deleted, 1 modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Switching auth providers requires a sweep of all consumer components.

## Evaluation notes (flywheel)

- Failure modes observed: Lingering imports in unused files.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
