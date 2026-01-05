---
id: 088
title: Fix Missing ReactMarkdown Dependency
stage: fix
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","dependency","build"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - package.json
tests:
 - npm run build (PASS)
---

## Prompt

Module not found: Error: Can't resolve 'react-markdown'

## Response snapshot

Installed `react-markdown` to resolve the build error caused by importing it in `ChatWidget` without adding it to dependencies.

## Outcome

- ✅ Impact: ChatWidget can now render Markdown responses.
- 🧪 Tests: Build passed.
- 📁 Files: `package.json` updated.
- 🔁 Next prompts: null
- 🧠 Reflection: Always verify imports match installed packages.

## Evaluation notes (flywheel)

- Failure modes observed: Missing package import.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
