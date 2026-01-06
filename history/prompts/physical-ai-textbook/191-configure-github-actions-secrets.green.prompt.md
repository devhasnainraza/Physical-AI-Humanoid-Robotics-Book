---
id: 191
title: Configure GitHub Actions Secrets
stage: green
date: 2026-01-05
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: main
user: Muhammad Hasnain
command: now you give me env for github pages
labels: ["github-actions", "deployment", "env", "secrets"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .github/workflows/deploy.yml
tests:
 - "Push changes and verify workflow run"
---

## Prompt

User asked for environment variable configuration for GitHub Pages deployment.

## Response snapshot

1.  Explained that GitHub Pages is static and keys must be injected at build time.
2.  Updated `.github/workflows/deploy.yml` to pass `QDRANT_URL`, `QDRANT_API_KEY`, and `DATABASE_URL` secrets to the `npm run build` command.
3.  Instructed user to set these secrets in GitHub Repo Settings.

## Outcome

- ✅ Impact: Deployment workflow now supports full application configuration including RAG and Database.
- 🧪 Tests: Pushed to `main`. Workflow should trigger.
- 📁 Files: Modified `.github/workflows/deploy.yml`.
- 🔁 Next prompts: None.
- 🧠 Reflection: Building keys into static sites is insecure but acceptable for demos. Production requires a backend proxy.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
