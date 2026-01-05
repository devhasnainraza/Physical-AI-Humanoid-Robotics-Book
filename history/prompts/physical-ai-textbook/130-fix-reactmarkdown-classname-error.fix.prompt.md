---
id: 130
title: Fix ReactMarkdown ClassName Error
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","react-markdown","ui","crash"]
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

Unexpected className prop... ReactMarkdown

## Response snapshot

Fixed a crash caused by passing `className` directly to the `ReactMarkdown` component (which is deprecated in v9).
1.  **Solution**: Wrapped `<ReactMarkdown>` in a `<div>` and applied the Tailwind classes (`prose prose-sm...`) to the wrapper instead.
2.  **Scope**: Applied this fix to both the main message list and the streaming content preview.

## Outcome

- ✅ Impact: Page no longer crashes on render.
- 🧪 Tests: Component visual structure updated.
- 📁 Files: `src/components/ChatWidget/index.tsx` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Library upgrades often break props. Always check changelogs for "breaking changes".

## Evaluation notes (flywheel)

- Failure modes observed: Runtime crash due to invalid prop.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
