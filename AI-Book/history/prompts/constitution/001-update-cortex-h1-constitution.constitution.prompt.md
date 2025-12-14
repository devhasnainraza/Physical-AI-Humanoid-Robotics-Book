---
id: 001
title: Update Cortex-H1 Constitution
stage: constitution
date: 2025-12-13
surface: agent
model: gemini-cli
feature: none
branch: docs/update-constitution
user: user
command: /sp.constitution
labels: ["governance", "setup", "cortex-h1"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - null
---

## Prompt

$ARGUMENTS
/sp.constitution
**Project Constitution: The "Cortex-H1" Physical AI Platform**

**Article I: The Co-Learning Mandate**
The platform is not a static textbook; it is a "Co-Learning Agent." It must bridge the gap between "Code" and "Physical Reality." We strictly adhere to the CAARE Level IV Curriculum (ROS 2, Digital Twins, Isaac Sim, VLA).

**Article II: Dynamic Personalization (The "Hardware Reality" Rule)**
The content must adapt to the user's available hardware. A user with a physical "Unitree Go2" robot should see different deployment steps than a user running "Isaac Sim" on the cloud.

**Article III: Tech Stack Sovereignty**
We utilize the mandated stack:
1.  **Framework:** Docusaurus 3.x (React).
2.  **Identity:** Better Auth (PostgreSQL Adapter).
3.  **Intelligence:** OpenAI ChatKit + Qdrant (Vector DB) + LangChain.

## Response snapshot

Updated constitution to v1.0.0 with 3 key articles: Co-Learning Mandate, Dynamic Personalization, and Tech Stack Sovereignty.

## Outcome

- ✅ Impact: Established project governance and core technical principles.
- 🧪 Tests: N/A
- 📁 Files: Updated constitution.md
- 🔁 Next prompts: /sp.plan
- 🧠 Reflection: Initial constitution setup.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None
