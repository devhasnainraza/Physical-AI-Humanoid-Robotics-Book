---
id: "031"
title: "fix-mdx-brace-error"
stage: "green"
date: "2025-12-30"
surface: "agent"
model: "gemini-2.0-flash"
feature: "physical-ai-textbook"
branch: "main"
user: "user"
command: "fix crash"
labels:
  - "mdx"
  - "bugfix"
links:
  spec: "specs/001-physical-ai-textbook/spec.md"
files:
  - "docs/textbook/01-chapter-1-foundations.md"
tests: []
---

# Prompt

User reported a crash on `docs/textbook/chapter-1-foundations` with error "B is not defined".

# Response

Identified the cause: MDX interprets `{B}` and `{A}` as JavaScript expressions (variables) instead of literal text.
Fixed the issue by escaping the braces `\{B\}` and `\{A\}` in `docs/textbook/01-chapter-1-foundations.md`.

1.  Located the problematic lines:
    *   `If frame {B} is rotated relative to frame {A}...`
    *   `...transform a point $v$ from frame {B} to frame {A}...`
2.  Applied replacements to escape the braces.
3.  Verified the fix.
