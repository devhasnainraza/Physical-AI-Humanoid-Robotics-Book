---
id: "030"
title: "restore-textbook-structure"
stage: "green"
date: "2025-12-30"
surface: "agent"
model: "gemini-2.0-flash"
feature: "physical-ai-textbook"
branch: "main"
user: "user"
command: "create textbook"
labels:
  - "docusaurus"
  - "sidebars"
  - "fix"
links:
  spec: "specs/001-physical-ai-textbook/spec.md"
  tasks: "specs/001-physical-ai-textbook/tasks.md"
files:
  - "sidebars.ts"
tests: []
---

# Prompt

hey plz you create a Physical AI & Humanoid Robotics Textbook for teaching using docusaurus

# Response

Analyzed the existing project structure and found that the Docusaurus project was already initialized but `sidebars.ts` was misconfigured, pointing to non-existent folders.

1.  Verified content in `docs/textbook` and `docs/module-1-ros2`.
2.  Updated `sidebars.ts` to correctly map the existing chapters and modules, enabling navigation.
3.  Identified a configuration mismatch in Authentication:
    *   `src/theme/Root.tsx` uses Clerk.
    *   `auth-server.ts` and `tasks.md` specify Better Auth.
    *   `package.json` includes both.
    *   Port conflict: Auth server (3000) vs Docusaurus (3000).

Restored the textbook structure so the content is visible. Deferred Auth migration to next steps.
