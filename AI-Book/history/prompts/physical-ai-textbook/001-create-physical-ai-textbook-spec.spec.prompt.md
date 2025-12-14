---
id: 001
title: Create Physical AI Textbook Spec
stage: spec
date: 2025-12-13
surface: agent
model: gemini-cli
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: user
command: /sp.specify
labels: ["spec", "setup", "docusaurus", "ros2", "vla"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-physical-ai-textbook/spec.md
 - specs/001-physical-ai-textbook/checklists/requirements.md
tests:
 - null
---

## Prompt

$ARGUMENTS
**Specification: The "Physical AI & Humanoid Robotics" Textbook** **1. Core Architecture** Create a Docusaurus-based educational platform that serves as the official textbook for the "Certified Agentic AI & Robotics Engineer" course. **2. The Syllabus Structure (Content Schema)** Generate the sidebar and folder structure to match the Official Syllabus exactly: * **Module 1: The Robotic Nervous System (ROS 2)** * Concepts: Nodes, Topics, Services, Custom Interfaces. * *Lab:* Creating a URDF for a humanoid robot. * **Module 2: The Digital Twin** * Concepts: Gazebo Harmony Physics, Unity Rendering, Sensor Simulation. * **Module 3: The AI-Robot Brain** * Concepts: NVIDIA Isaac Sim, Nav2, SlamToolbox. * **Module 4: Vision-Language-Action (VLA)** * Concepts: Whisper API (Voice), GR00T-mimicry, Text-to-Action. **3. Advanced Feature Requirements (The "Winning" Features)** * **A. Identity & Profiling (Better Auth):** * Implement a custom Signup Form using Better Auth. * **Crucial:** Add a `hardware_profile` selection: `['Cloud_Simulation_Only', 'Jetson_Orin_Kit', 'Unitree_Go2_Robot']`. * Store this profile in the User Session. * **B. Context-Aware RAG Chatbot (ChatKit):** * Integrate OpenAI ChatKit as a floating widget. * **"Smart Context" Feature:** Implement a text-selection listener. If a user highlights text on the page (e.g., "Inverse Kinematics"), the Chatbot input must automatically fill with: *"Explain [Selected Text] specifically for a humanoid robot."* * **Backend:** Define a Python script (`rag_ingest.py`) to parse the Docusaurus `.md` files and index them into Qdrant. * **C. Agentic "Translator" Skill:** * Instead of static translations, add a "Translate Page" button. * This triggers a Server-Side Agent (Gemini 2.0 Flash) to translate the current Markdown content into Urdu/Hindi on the fly, preserving code blocks. **4. User Experience (UX) Rules** * Use `<HardwareOnly profile="Unitree_Go2">` tags in the markdown. Content inside these tags should only render if the logged-in user has that specific hardware profile.

## Response snapshot

Created feature spec for "Physical AI & Humanoid Robotics Textbook".
- Branch: 001-physical-ai-textbook
- Defined 4 key user stories: Core Syllabus, Identity/Profiling, RAG Chatbot, Agentic Translation.
- Validated against constitution (tech stack constraints accepted).
- Created requirements checklist.

## Outcome

- ✅ Impact: Formalized feature requirements into a structured spec.
- 🧪 Tests: N/A
- 📁 Files: specs/001-physical-ai-textbook/spec.md, specs/001-physical-ai-textbook/checklists/requirements.md
- 🔁 Next prompts: /sp.plan
- 🧠 Reflection: Strong alignment with constitution principles.

## Evaluation notes (flywheel)

- Failure modes observed: PowerShell script string parsing failed; fell back to manual file creation.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): Improve PowerShell script argument handling for large JSON blobs.
