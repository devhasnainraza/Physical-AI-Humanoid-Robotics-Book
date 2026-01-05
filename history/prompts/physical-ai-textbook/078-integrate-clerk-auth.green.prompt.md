---
id: 078
title: Integrate Clerk Authentication
stage: green
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["auth","clerk","integration","react"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - src/components/Auth/ClerkAuth.tsx
 - src/theme/Layout/index.tsx
 - package.json
tests:
 - npm run build (PASS)
---

## Prompt

add this feature @prompt.md (Clerk Auth)

## Response snapshot

Implemented Clerk Authentication adapted for Docusaurus (React).
1.  **Cleanup**: Removed conflicting Firebase/Better-Auth code.
2.  **Install**: Added `@clerk/clerk-react`.
3.  **UI**: Created `ClerkAuth.tsx` to portal "Sign In" / "User Button" into the Navbar.
4.  **Integration**: Updated `LayoutWrapper` to wrap the app in `ClerkProvider` and protect `/docs/` routes.

## Outcome

- ✅ Impact: Project now uses industry-standard Clerk Auth.
- 🧪 Tests: Component logic created.
- 📁 Files: 2 files created/modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Adapted Next.js prompt to Docusaurus context successfully.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
