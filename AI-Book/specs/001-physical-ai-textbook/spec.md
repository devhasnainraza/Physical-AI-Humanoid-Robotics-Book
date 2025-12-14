# Feature Specification: Physical AI & Humanoid Robotics Textbook

**Feature Branch**: `001-physical-ai-textbook`
**Created**: 2025-12-13
**Status**: Draft
**Input**: User description provided in prompt.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Core Syllabus & Navigation (Priority: P1)

A student can browse the core "Certified Agentic AI & Robotics Engineer" curriculum, navigating through the four defined modules (ROS 2, Digital Twin, AI-Robot Brain, VLA).

**Why this priority**: Defines the fundamental content structure and value proposition of the textbook.
**Independent Test**: Verify navigation sidebar matches the official syllabus and pages load for each module.

**Acceptance Scenarios**:
1. **Given** a visitor on the landing page, **When** they view the sidebar, **Then** they see "Module 1: The Robotic Nervous System", "Module 2: The Digital Twin", "Module 3: The AI-Robot Brain", and "Module 4: Vision-Language-Action".
2. **Given** a student viewing Module 1, **When** they click "Lab: Creating a URDF", **Then** the specific lab content page loads.

### User Story 2 - Identity & Hardware Profiling (Priority: P1)

A user signs up and selects their specific hardware availability (e.g., "Unitree Go2"), which persists in their session.

**Why this priority**: Required for the "Dynamic Personalization" core principle (Article II of Constitution).
**Independent Test**: Create an account, select "Unitree Go2", logout, login, and verify profile persists.

**Acceptance Scenarios**:
1. **Given** a new user on the signup form, **When** they register, **Then** they are prompted to select a `hardware_profile` from: `Cloud_Simulation_Only`, `Jetson_Orin_Kit`, `Unitree_Go2_Robot`.
2. **Given** a logged-in user with "Unitree Go2" profile, **When** they view a page with `<HardwareOnly profile="Unitree_Go2">` content, **Then** that content is visible.
3. **Given** a user with "Cloud_Simulation_Only", **When** they view the same page, **Then** the "Unitree Go2" specific content is hidden.

### User Story 3 - Context-Aware RAG Chatbot (Priority: P2)

A student reading complex topics (e.g., "Inverse Kinematics") can highlight text to instantly ask a context-aware question via the floating AI widget.

**Why this priority**: enhances the learning experience by reducing friction in asking questions.
**Independent Test**: Highlight text, verify chatbot input population, submit query, verify response relevance to "humanoid robot" context.

**Acceptance Scenarios**:
1. **Given** a user reading a markdown page, **When** they highlight the phrase "Inverse Kinematics", **Then** the floating Chatbot widget opens/activates.
2. **Given** the widget opens, **Then** the input field is pre-filled with: "Explain Inverse Kinematics specifically for a humanoid robot."
3. **Given** the user sends the message, **Then** the bot replies using indexed knowledge from the textbook.

### User Story 4 - Agentic Translation (Priority: P3)

A student fluent in Urdu or Hindi can translate the current page on-the-fly without losing code block formatting.

**Why this priority**: Accessibility and inclusivity for the target demographic.
**Independent Test**: Click "Translate Page", verify text changes to Urdu/Hindi, verify code blocks remain in English/original format.

**Acceptance Scenarios**:
1. **Given** a user on a content page, **When** they click "Translate Page" (and select Urdu), **Then** the prose content is replaced with Urdu text.
2. **Given** the page has Python/C++ code blocks, **When** translation completes, **Then** the code blocks remain unchanged and correctly formatted.

### Edge Cases

- **Hardware Profile Missing**: If a user has no profile set (legacy/error), default to `Cloud_Simulation_Only`.
- **RAG Indexing Failure**: If the ingestion script fails on a malformed MD file, it should log the error and continue indexing others.
- **Translation Latency**: If translation takes >5s, show a loading state/spinner on the button.
- **Chatbot Offline**: If ChatKit/API is unreachable, the widget should display a friendly "Offline" message.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a web-based platform (Docusaurus-based) structured into 4 specific Modules: ROS 2, Digital Twin, AI-Robot Brain, VLA.
- **FR-002**: System MUST allow users to register and login using an authentication system (Better Auth) that captures a mandatory `hardware_profile`.
- **FR-003**: System MUST enforce valid `hardware_profile` values: `Cloud_Simulation_Only`, `Jetson_Orin_Kit`, `Unitree_Go2_Robot`.
- **FR-004**: System MUST render content conditionally based on the user's `hardware_profile` using a custom markdown component/tag (e.g., `<HardwareOnly>`).
- **FR-005**: System MUST include a floating chatbot widget (ChatKit) available on all content pages.
- **FR-006**: System MUST detect text selection events on content pages and inject the formatted prompt ("Explain [Selection] specifically for a humanoid robot") into the chatbot input.
- **FR-007**: System MUST support RAG (Retrieval-Augmented Generation) by indexing all content markdown files into a vector database (Qdrant).
- **FR-008**: System MUST provide a "Translate Page" feature that utilizes an LLM (Gemini 2.0 Flash) to convert prose to Urdu/Hindi.
- **FR-009**: The translation feature MUST strictly preserve all markdown code blocks (content between triple backticks) in their original state.

### Key Entities

- **User**: Represents a learner. Attributes: `id`, `email`, `hardware_profile`.
- **HardwareProfile**: Enum/Type. Values: `Cloud_Simulation_Only`, `Jetson_Orin_Kit`, `Unitree_Go2_Robot`.
- **KnowledgeBase**: The collection of indexed textbook content vectors.
- **ContentPage**: A single unit of learning material (Markdown file).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can successfully register and select a hardware profile in < 2 minutes.
- **SC-002**: Hardware-specific content is 100% hidden for incompatible profiles during page render.
- **SC-003**: Highlighting text triggers the chatbot input update in < 200ms.
- **SC-004**: RAG Chatbot answers questions using textbook content with > 80% relevance (measured by test set).
- **SC-005**: "Translate Page" returns a translated version in < 10 seconds for an average length page.
- **SC-006**: Code blocks in translated pages remain identical to the source in 100% of test cases.
