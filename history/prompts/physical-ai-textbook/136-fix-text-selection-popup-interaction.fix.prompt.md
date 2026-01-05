---
id: 136
title: Fix Text Selection Popup Interaction
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","ui","event-handling","chat"]
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

why not working explain and summarize button...

## Response snapshot

Fixed the issue where the text selection popup would close before the button click registered.
1.  **Event Handling**: Switched from `onClick` to `onMouseDown` for the popup buttons. `mousedown` fires before the text selection is cleared by the browser, ensuring the action triggers.
2.  **Logic Update**: Updated the `handleSelection` listener to ignore clicks that originate *inside* the popup (`target.closest('.selection-popup')`).

## Outcome

- ✅ Impact: The "Explain" and "Summarize" buttons now work reliably when text is highlighted.
- 🧪 Tests: Component logic updated.
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Race conditions between `blur`/`mouseup` and `click` are common in floating UIs. `onMouseDown` + `preventDefault` is the standard fix.

## Evaluation notes (flywheel)

- Failure modes observed: Popup closing prematurely.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
