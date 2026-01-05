---
id: 076
title: Troubleshoot Firebase 400 Error
stage: explainer
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["troubleshoot","firebase","auth","config"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - null
tests:
 - null
---

## Prompt

error in console : {"error":{"code":400,"message":"CONFIGURATION_NOT_FOUND"...}}

## Response snapshot

Identified Firebase error `CONFIGURATION_NOT_FOUND` as a missing "Sign-in method" configuration in the Firebase Console. Advised user to enable "Google" provider and add their GitHub Pages domain to Authorized Domains.

## Outcome

- ✅ Impact: User guided to fix external service configuration.
- 🧪 Tests: N/A
- 📁 Files: N/A
- 🔁 Next prompts: null
- 🧠 Reflection: Code is correct, but SaaS integrations always require dashboard config.

## Evaluation notes (flywheel)

- Failure modes observed: Missing Firebase Console setup.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
