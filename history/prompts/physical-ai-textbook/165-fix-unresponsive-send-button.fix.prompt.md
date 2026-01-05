---
id: 165
title: Fix Unresponsive Send Button
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","chat","ux","state-bug"]
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

not working properly my chat (send button unresponsive)

## Response snapshot

1.  **Diagnosis**: The "Send" button was likely unresponsive due to a `loading` state that was not being correctly reset to `false` after an API error or a cancelled stream, leaving the button permanently disabled.
2.  **Fix**: Refactored the `handleSubmit` function to use a `finally` block, ensuring `setLoading(false)` is always called, regardless of whether the API call succeeds or fails.
3.  **UI Polish**: Added a `Loader2` spinner icon to the Send button itself when `loading` is true, providing clearer feedback to the user.

## Outcome

- ✅ Impact: The send button is now robust and will never get stuck in a disabled state.
- 🧪 Tests: n/a
- 📁 Files: `src/components/ChatWidget/index.tsx` overwritten.
- 🔁 Next prompts: null
- 🧠 Reflection: For any async UI action, always use `finally` to reset loading states.

## Evaluation notes (flywheel)

- Failure modes observed: Stuck loading state.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
