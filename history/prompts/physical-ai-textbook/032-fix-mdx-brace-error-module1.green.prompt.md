---
id: "032"
title: "fix-mdx-brace-error-module1"
stage: "green"
date: "2025-12-30"
surface: "agent"
model: "gemini-2.0-flash"
feature: "physical-ai-textbook"
branch: "main"
user: "user"
command: "fix crash module 1"
labels:
  - "mdx"
  - "bugfix"
links:
  spec: "specs/001-physical-ai-textbook/spec.md"
files:
  - "docs/module-1-ros2/01-foundations-physical-ai.md"
tests: []
---

# Prompt

User reported a crash on `docs/module-1-ros2/foundations-physical-ai` with error "B is not defined".

# Response

Identified the cause: Same issue as previous task. MDX interprets `{B}` and `{A}` as JavaScript expressions (variables) instead of literal text.
Fixed the issue by escaping the braces `\{B\}` and `\{A\}` in `docs/module-1-ros2/01-foundations-physical-ai.md`.

1.  Located the problematic line:
    *   `A point $P$ in frame {B} can be expressed in frame {A}...`
2.  Applied replacements to escape the braces.
3.  Verified by searching codebase for other occurrences (none found in active docs).
