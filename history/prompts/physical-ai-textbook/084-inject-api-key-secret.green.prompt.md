---
id: 084
title: Inject API Key Secret in CI
stage: green
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: Muhammad Hasnain
command: /sp.implement
labels: ["ci","github-actions","secrets","env"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - .github/workflows/deploy.yml
tests:
 - null
---

## Prompt

yes update (deploy.yml)

## Response snapshot

Updated GitHub Actions workflow to inject `GEMINI_API_KEY` from repository secrets during the build process. This allows the Docusaurus build to inline the key into the client-side bundle.

## Outcome

- ✅ Impact: Deployment will now succeed with the API key intact (if secret is set in repo settings).
- 🧪 Tests: Workflow updated.
- 📁 Files: `.github/workflows/deploy.yml` modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Crucial step for JAMstack deployments relying on build-time env vars.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
