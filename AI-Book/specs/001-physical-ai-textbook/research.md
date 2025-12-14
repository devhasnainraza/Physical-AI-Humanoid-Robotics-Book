# Research: Physical AI & Humanoid Robotics Textbook

## Technology Stack Confirmation

### Framework: Docusaurus 3.x
- **Decision**: Use Docusaurus 3.x with TypeScript.
- **Rationale**: Mandated by Constitution Article III. Ideal for documentation/textbook content with React extensibility.

### Identity: Better Auth
- **Decision**: Better Auth with PostgreSQL adapter.
- **Rationale**: Mandated by Constitution Article III. Provides robust authentication and session management needed for hardware profiling.

### Intelligence: ChatKit + Qdrant
- **Decision**: OpenAI ChatKit for UI/State + Qdrant for Vector Storage.
- **Rationale**: Mandated by Constitution Article III. Qdrant handles the RAG index; ChatKit manages the conversational state.

### Translation: Gemini 2.0 Flash
- **Decision**: Google Gemini 2.0 Flash via Generative AI SDK.
- **Rationale**: Specified in feature requirements for high-speed, low-latency translation of markdown content.

## Integration Patterns

### Hardware Profiling
- **Pattern**: Store `hardware_profile` in User Session (Better Auth additional fields).
- **Implementation**: Custom React component `<HardwareOnly profile="...">` checks session context. If match/cloud, render children; else return null.

### Text Selection Trigger
- **Pattern**: Global `mouseup` listener in the Layout wrapper.
- **Logic**: 
  - Check if `window.getSelection().toString()` is not empty.
  - If valid, dispatch event or update ChatKit context state.
  - Pre-fill input box with template: "Explain [Selection] specifically for a humanoid robot."

### RAG Ingestion
- **Pattern**: Python script (`rag_ingest.py`) running in CI/CD or manual trigger.
- **Logic**:
  - Parse `docs/**/*.md`.
  - Chunk text (keeping code blocks intact).
  - Generate embeddings (OpenAI/Gemini).
  - Upsert to Qdrant collection `textbook_knowledge`.
