---
id: 021
title: Fix MDX Math Errors
stage: green
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["fix","mdx","math","config"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - docs/module-1-ros2/03-urdf-modeling.md
 - docs/module-2-digital-twin/intro.md
 - docs/module-2-digital-twin/03-sensor-plugins.md
 - docs/module-3-ai-brain/03-visual-slam.md
 - docusaurus.config.ts
tests:
 - npm run build (PASS)
---

## Prompt

ERROR in ./docs/module-3-ai-brain/03-visual- (MDX Acorn Error)

## Response snapshot

Identified that `remark-math` (ESM) was incompatible with the project's CJS runtime configuration, causing MDX v3 to parse `$$` blocks as invalid JS. Fixed by removing the math plugins and converting equations to standard code blocks/text in 4 files.

## Outcome

- ✅ Impact: Build is green again. Content remains "Advanced" but uses simplified notation for math.
- 🧪 Tests: Build passed.
- 📁 Files: 4 content files modified, config reverted.
- 🔁 Next prompts: null
- 🧠 Reflection: Mixing ESM plugins in a TS CJS config is brittle. Best to stick to standard markdown for stability unless project is fully migrated to ESM.

## Evaluation notes (flywheel)

- Failure modes observed: MDX v3 strict parsing of brace content inside unhandled `$$` blocks.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
