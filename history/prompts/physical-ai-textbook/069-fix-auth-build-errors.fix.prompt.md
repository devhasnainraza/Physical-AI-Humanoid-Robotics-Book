---
id: 069
title: Fix Auth-Related Build Errors
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","build","auth"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - src/pages/signup.tsx (deleted)
 - src/components/HardwareOnly/index.tsx
tests:
 - npm run build (PASS)
---

## Prompt

ERROR in ./src/components/HardwareOnly/index.tsx 1:25-78 Module not found: Error: Can't resolve ' .. /Auth/AuthProvider' ...

## Response snapshot

Fixed build errors after Auth system reset:
1.  Deleted obsolete `src/pages/signup.tsx`.
2.  Refactored `src/components/HardwareOnly/index.tsx` to use the new `auth` client (`src/lib/auth-client.ts`) directly, replacing the dependency on the deleted `AuthProvider`.

## Outcome

- ✅ Impact: Project builds cleanly without Auth-related module not found errors.
- 🧪 Tests: Build passed.
- 📁 Files: One file deleted, one refactored.
- 🔁 Next prompts: null
- 🧠 Reflection: Removing core modules requires checking all dependents.

## Evaluation notes (flywheel)

- Failure modes observed: Module not found after deletion.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
