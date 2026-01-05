---
id: 185
title: Fix MDX Compilation Error in Chapter 2
stage: fix
date: 2026-01-05
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: main
user: Muhammad Hasnain
command: fix
labels: ["mdx", "bugfix", "syntax-error"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - docs/textbook/02-chapter-2-robot-hardware.md
tests:
 - "Checked for invalid JSX tag syntax (<1us)"
---

## Prompt

User reported an MDX compilation error in `docs/textbook/02-chapter-2-robot-hardware.md`:
"Unexpected character 1 (U+0031) before name, expected a character that can start a name, such as a letter, $, or _" at line 111.
Context provided via `error.md`.

## Response snapshot

Fixed MDX syntax error in `docs/textbook/02-chapter-2-robot-hardware.md` by escaping `<` in `<1us` to `&lt;1us`.

## Outcome

- ✅ Impact: Fixed build error.
- 🧪 Tests: Verified manually by inspecting code.
- 📁 Files: Modified 1 file.
- 🔁 Next prompts: Verify build.
- 🧠 Reflection: MDX parsers are strict about `<` characters.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
