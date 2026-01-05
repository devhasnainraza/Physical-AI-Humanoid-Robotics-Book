---
id: 132
title: Switch to Gemini Pro
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","ai","model","gemini-pro"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - src/components/ChatWidget/index.tsx
tests:
 - npm run build (PASS)
---

## Prompt

Error: [404] models/gemini-1.5-flash-001 is not found

## Response snapshot

Switched the model identifier to `gemini-pro`.
1.  **Reason**: Repeated 404s on `gemini-1.5-flash` variants suggest region/account incompatibility or API version mismatch for that specific model.
2.  **Fix**: `gemini-pro` is the GA (General Availability) model and has the widest support.

## Outcome

- ✅ Impact: Should resolve the 404 Model Not Found error.
- 🧪 Tests: n/a
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: When cutting-edge models fail, fallback to the GA version.

## Evaluation notes (flywheel)

- Failure modes observed: Persistent 404s on 1.5 Flash.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
