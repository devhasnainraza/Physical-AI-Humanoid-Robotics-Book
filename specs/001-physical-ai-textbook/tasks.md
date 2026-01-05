---
description: "Task list for Physical AI & Humanoid Robotics Textbook implementation"
---

# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `specs/001-physical-ai-textbook/`
**Prerequisites**: plan.md (required), spec.md (required), data-model.md, contracts/

**Tests**: NOT explicitly requested in spec, but basic validation tasks included for key logic.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Initialize Docusaurus 3.x project with TypeScript in root directory
- [ ] T002 [P] Install dependencies: `better-auth`, `openai-chatkit`, `@langchain/core`, `@qdrant/js-client-rest`, `openai`
- [ ] T003 [P] Configure Tailwind CSS in `docusaurus.config.ts` and `src/css/custom.css`
- [ ] T004 [P] Setup PostgreSQL database and Qdrant instance (Docker/Cloud) (Created docker-compose.yml)
- [ ] T005 [P] Create `.env.local` with keys for Better Auth, OpenAI, Gemini, Postgres, Qdrant

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 Refine/Verify Better Auth configuration in `auth-server.ts` (Hybrid Mode)
- [ ] T007 [P] Create `HardwareProfile` enum and extend User schema in `api/auth/schema.ts` (if applicable) or database migration
- [ ] T008 [P] Setup API route handler structure in `api/` to support custom endpoints (Chat, Translation)
- [ ] T009 [P] Create Qdrant collection `textbook_knowledge` using setup script or curl (Created scripts/setup_qdrant.js)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Core Syllabus & Navigation (Priority: P1)

**Goal**: Student can browse the 4-module curriculum and view content.

**Independent Test**: Verify navigation sidebar matches official syllabus and module pages load.

### Implementation for User Story 1

- [ ] T010 [US1] Create folder structure for `docs/module-1-ros2`, `docs/module-2-digital-twin`, `docs/module-3-ai-brain`, `docs/module-4-vla`
- [ ] T011 [P] [US1] Create content placeholder `docs/module-1-ros2/lab-urdf.md`
- [ ] T012 [P] [US1] Configure sidebars in `sidebars.ts` to explicitly list the 4 modules and their hierarchy
- [ ] T013 [US1] Verify Docusaurus build renders the sidebar correctly

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Identity & Hardware Profiling (Priority: P1)

**Goal**: User can sign up with hardware profile; content hides/shows based on profile.

**Independent Test**: Create account with "Unitree Go2", verify `<HardwareOnly>` content renders.

### Implementation for User Story 2

- [ ] T014 [US2] Create Custom Signup Form component in `src/components/Auth/SignupForm.tsx` with `hardware_profile` selector
- [ ] T015 [US2] Update Auth Provider to expose `user.hardware_profile` to client-side context
- [ ] T016 [US2] Create `HardwareOnly` component in `src/components/HardwareOnly/index.tsx` that checks auth context
- [ ] T017 [US2] Add test page `docs/test-hardware.md` using `<HardwareOnly profile="Unitree_Go2">` to verify behavior

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Context-Aware RAG Chatbot (Priority: P2)

**Goal**: Student can highlight text to ask RAG-powered questions via floating widget.

**Independent Test**: Highlight text, widget opens with pre-filled query, returns relevant answer.

### Implementation for User Story 3

- [ ] T018 [P] [US3] Create Python ingestion script `scripts/rag_ingest.py` to parse MD files and upsert to Qdrant
- [ ] T019 [US3] Refine/Verify RAG API endpoint `api/chat/route.ts` using ChatKit/LangChain to query Qdrant (Ensure Vercel Function compatibility)
- [ ] T020 [P] [US3] Create `ChatWidget` component in `src/components/ChatWidget/index.tsx` using ChatKit UI
- [ ] T021 [US3] Implement global `mouseup` listener in `src/theme/Layout/index.tsx` (swizzle Layout) to detect selection
- [ ] T022 [US3] Connect selection event to `ChatWidget` state to pre-fill input box

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: User Story 4 - Agentic Translation (Priority: P3)

**Goal**: Student can translate page to Urdu/Hindi on-the-fly, preserving code blocks.

**Independent Test**: Click translate, prose changes language, code blocks remain identical.

### Implementation for User Story 4

- [ ] T023 [P] [US4] Refine/Verify Translation API endpoint `api/translate/route.ts` using Gemini 2.0 Flash SDK (Ensure Vercel Function compatibility)
- [ ] T024 [P] [US4] Create `Translator` button component in `src/components/Translator/index.tsx`
- [ ] T025 [US4] Implement client-side logic to replace DOM text nodes with translated content (use React Portal or targeted ID replacement to avoid hydration errors)
- [ ] T026 [US4] Verify code block preservation on a page with Python snippets

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T027 [P] Clean up temporary test pages
- [ ] T028 Update `README.md` with setup instructions
- [ ] T029 Perform full regression test of all 4 modules and profiles
- [ ] T030 [US3] Create RAG Evaluation Test Set (questions + expected answers) (SC-004)
- [ ] T031 [US3] Run Relevance Evaluation script/manual test and verify > 80% relevance (SC-004)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies
- **Foundational (Phase 2)**: Depends on Setup
- **User Stories (Phase 3+)**: All depend on Foundational
  - US1 (Syllabus) can run parallel to others
  - US2 (Auth) blocks US3/US4 personalization aspects (but implementation can be parallel)

### User Story Dependencies

- **US1 (Syllabus)**: Independent
- **US2 (Identity)**: Independent foundation, but US3/4 might use auth context eventually
- **US3 (RAG)**: Independent (can use mock auth or no auth for dev)
- **US4 (Translation)**: Independent

### Parallel Opportunities

- T002, T003, T004, T005 (Setup)
- T010, T011, T012 (US1 Content)
- T018, T020 (US3 Backend vs Frontend)
- T023, T024 (US4 API vs UI)

---

## Implementation Strategy

### MVP First (US1 + US2)

1. Complete Setup + Foundational
2. Implement US1 (Structure)
3. Implement US2 (Auth + Hardware Profiles)
4. **Deploy MVP**: Working Textbook with Auth

### Incremental Delivery

1. Add US3 (Chatbot) → Release 1.1
2. Add US4 (Translation) → Release 1.2

### Parallel Team Strategy

- **Dev A**: US1 (Content) + US4 (Translation)
- **Dev B**: US2 (Auth) + US3 (RAG/Chatbot)