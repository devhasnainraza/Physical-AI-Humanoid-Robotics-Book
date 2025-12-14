# Data Model: Physical AI & Humanoid Robotics Textbook

## Entities

### User
Represents a student/learner accessing the platform.
- **Source**: Better Auth (PostgreSQL)
- **Fields**:
  - `id`: String (UUID)
  - `email`: String (Unique)
  - `name`: String
  - `hardware_profile`: Enum (See HardwareProfile) - **CRITICAL**
  - `created_at`: Timestamp

### HardwareProfile (Enum)
Defines the available hardware context for personalization.
- **Values**:
  - `Cloud_Simulation_Only` (Default)
  - `Jetson_Orin_Kit`
  - `Unitree_Go2_Robot`

### KnowledgeChunk
Represents a segment of the textbook indexed for RAG.
- **Source**: Qdrant (Vector DB)
- **Fields**:
  - `id`: String (UUID or Hash)
  - `content`: String (The text chunk)
  - `embedding`: Vector (Float32 array)
  - `source_file`: String (Path to MD file)
  - `module`: String (e.g., "Module 1")

## Storage Schema

### PostgreSQL (Auth)
Standard Better Auth schema with added column:
```sql
ALTER TABLE "user" ADD COLUMN "hardware_profile" TEXT DEFAULT 'Cloud_Simulation_Only';
```

### Qdrant (Vectors)
Collection: `textbook_knowledge`
- Payload: `{"content": "...", "source": "...", "module": "..."}`
- Vector Size: Depends on embedding model (e.g., 1536 for OpenAI text-embedding-3-small)
