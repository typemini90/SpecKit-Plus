# SpecKit-Plus Backend

## Directory Structure

```
backend/
├── main.py                 # Main FastAPI application entry point
├── configs/
│   └── config.py          # Configuration and model settings
├── services/
│   ├── rag.py            # RAG (Retrieval Augmented Generation) service
│   ├── indexer.py        # Document indexing for Qdrant vector database
│   └── chatkit_service.py # ChatKit server with RAG capabilities
├── data/
│   ├── vector_store.py   # Qdrant vector database operations
│   └── embeddings.py     # Embedding service using FastEmbed
├── simple_agents/
│   └── aagents.py        # Triage Agent definition
├── agents/
│   └── Runner.py         # Agent runner
├── tests/
│   ├── test_keys.py      # Configuration tests
│   └── test_rag.py       # RAG functionality tests
└── history/              # History of prompts and decisions
    ├── prompts/          # Prompt History Records (PHRs)
    └── adr/             # Architecture Decision Records (ADRs)
```

## What Has Been Done

### 1. Code Organization & Structure
- Organized backend files into logical directories (configs, services, data, simple_agents, agents, tests)
- Fixed import paths across all files to work with new directory structure
- Centralized model configuration in `configs/config.py`

### 2. Qdrant Integration & RAG System
- Implemented document indexing system in `services/indexer.py` to upload documents from `frontend/docs` to Qdrant
- Fixed Qdrant ID validation issues by changing from file paths to UUIDs
- Created RAG service in `services/rag.py` for contextual query processing
- Connected the main Agent in `main.py` to use Qdrant context before responding
- Updated default collection name to "Humanoids" across all components

### 3. Agent Integration
- Connected the Triage Agent to use RAG context from Qdrant for more informed responses
- Updated API endpoints (`/api/query` and `/api/selection`) to incorporate RAG context

### 4. Configuration & Security
- Centralized configuration management with proper environment variable handling
- Fixed undefined variable issues in services

## What Is Left

### 1. Testing & Verification
- [ ] Test complete RAG flow to verify Agent uses Qdrant context properly
- [ ] Verify responses include context from uploaded documents
- [ ] Confirm sources field is populated with document references

### 2. Documentation & History
- [ ] Create PHR for RAG integration work
- [ ] Finalize commit with all changes

## API Endpoints

- `GET /` - Health check endpoint
- `POST /api/query` - General chat queries with RAG context
- `POST /api/selection` - Queries based on selected text with RAG context

## Setup

1. Install dependencies: `uv pip install -r requirements.txt`
2. Set up environment variables in `.env`:
   ```
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   EXTERNAL_API_KEY=your_external_api_key
   ```
3. Index documents: `python services/indexer.py`
4. Start server: `uv run --no-dev uvicorn main:app --reload --port=8000`