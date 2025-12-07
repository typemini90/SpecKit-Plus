---
id: 2
title: Backend RAG Implementation Plan
stage: plan
date: 2025-12-07
surface: agent
model: claude-3-5-sonnet-20241022
feature: 001-backend-rag-pipeline
branch: main
user: giaic
command: /sp.plan
labels: ["backend", "rag", "planning", "architecture", "design"]
links:
  spec: /home/giaic/code/dockathon/backend/specs/001-backend-rag-pipeline/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - /home/giaic/code/dockathon/backend/specs/001-backend-rag-pipeline/plan.md
 - /home/giaic/code/dockathon/backend/specs/001-backend-rag-pipeline/tasks.md
 - /home/giaic/code/dockathon/backend/specs/001-backend-rag-pipeline/data-model.md
 - /home/giaic/code/dockathon/backend/specs/001-backend-rag-pipeline/research.md
 - /home/giaic/code/dockathon/backend/specs/001-backend-rag-pipeline/quickstart.md
 - /home/giaic/code/dockathon/backend/specs/001-backend-rag-pipeline/contracts/api-spec.yaml
 - /home/giaic/code/dockathon/backend/.specify/memory/constitution.md
 - /home/giaic/code/dockathon/backend/.specify/memory/qwen-context.md
tests:
 - none
---

## Prompt

Use my existing specification to create a full implementation plan.

Include:
1. Architecture overview
2. Component breakdown
3. Implementation phases
4. Dependencies and sequencing
5. Design decisions that require ADRs
6. Validation and quality gates (based on Constitution + Spec)

Output to: specs/[feature]/plan.md

## Response snapshot

# Implementation Plan: Backend RAG Pipeline

## Technical Context

**Feature Spec**: `/home/giaic/code/dockathon/backend/specs/001-backend-rag-pipeline/spec.md`
**Constitution**: `/home/giaic/code/dockathon/backend/.specify/memory/constitution.md`

**Component Architecture**:
- FastAPI application with health, query, and selection endpoints
- Document indexer for .md and .mdx files
- Qdrant vector database for storing embeddings
- FastEmbed MiniLM-L6-v2 for generating 384-dimensional embeddings
- OpenAI API for answer generation

**Technology Stack**:
- Python 3.13+
- FastAPI for web framework
- FastEmbed for local embeddings
- Qdrant Client for vector storage
- OpenAI SDK for answer generation
- Pydantic for request/response validation
- Loguru for logging

**Dependencies**:
- Qdrant Cloud (with URL and API key)
- OpenAI API key
- ../docs directory with .md/.mdx files

**Integration Points**:
- File system access to ../docs directory
- Qdrant Cloud service
- OpenAI API

**Unknowns**: 
- None (all requirements clearly specified in feature spec)

## Constitution Check

Verifying alignment with project constitution principles:

1. ✅ **Reliability and deterministic behavior**: All operations will be implemented with consistent, predictable behavior using proper error handling
2. ✅ **Clean, reproducible indexing and embeddings**: Using fixed model (MiniLM-L6-v2) with consistent parameters
3. ✅ **Clear separation between indexing and runtime serving**: Separate modules for indexing and API serving
4. ✅ **No hardcoded secrets**: All sensitive values will be loaded from environment variables
5. ✅ **Strict error handling and structured JSON responses**: All endpoints will use proper error handling with structured responses
6. ✅ **Predictable, testable RAG pipeline**: Design will allow for easy testing of individual components

All constitution principles are satisfied by this implementation plan.

## Gates

- ✅ **Constitution compliance**: All principles addressed and satisfied
- ✅ **Specification feasibility**: All requirements are technically achievable with selected technologies
- ✅ **Dependency resolution**: All required technologies are available and compatible

## Phase 0: Outline & Research

This phase has no NEEDS CLARIFICATION items as the feature spec is complete and clear. However, I'll conduct research on best practices for the chosen technologies:

### Research Summary

**FastEmbed Best Practices**:
- FastEmbed MiniLM-L6-v2 is efficient for sentence similarity tasks
- Provides 384-dimensional embeddings as required
- Local processing with no external API calls needed
- Supports batch processing for improved performance

**Qdrant Integration Best Practices**:
- Use cosine similarity for semantic search
- Properly configure collection for 384 dimensions
- Handle vector search with appropriate similarity thresholds
- Implement proper error handling for connection issues

**FastAPI Best Practices**:
- Use Pydantic models for request/response validation
- Implement proper middleware for logging and error handling
- Use dependency injection for shared services
- Follow RESTful API design principles

## Phase 1: Design & Contracts

### Data Model

#### DocumentChunk Entity
- chunk_id: str (primary key)
- content: str (the text content)
- doc_path: str (path to original document)
- embedding: List[float] (384-dimensional vector)
- metadata: Dict[str, Any] (additional document metadata)

#### QueryRequest Entity
- query: str (user query text)

#### QueryResponse Entity
- answer: str (generated answer)
- sources: List[str] (list of source documents)

#### SelectionRequest Entity
- selected_text: str (user-selected text)
- question: str (question about selected text)

#### SelectionResponse Entity
- answer: str (generated answer)

### API Contracts

#### Health Check Endpoint
```
GET /api/health
Response: 200 {"status": "ok"}
```

#### Query Endpoint
```
POST /api/query
Request: {"query": "text to search for"}
Response: 200 {"answer": "generated answer", "sources": ["source1", "source2"]}
Error: 400/500 with structured JSON error
```

#### Selection Endpoint
```
POST /api/selection
Request: {"selected_text": "text selected by user", "question": "question about text"}
Response: 200 {"answer": "generated answer"}
Error: 400/500 with structured JSON error
```

### Quickstart Guide

1. Install dependencies: `uv sync` (or `pip install -r requirements.txt`)
2. Set up environment variables (see `.env.example`)
3. Run the indexer to process documents: `python indexer.py`
4. Start the server: `uv run python main.py`
5. API documentation available at `http://localhost:8000/docs`

### Agent Context Update

The following technology will be added to the agent context:
- FastEmbed for embedding generation
- Qdrant for vector storage
- OpenAI API for answer generation
- FastAPI for web framework

### Phase 1 Completion Status

✅ **data-model.md** created with all required entities
✅ **research.md** created with all technology decisions and best practices
✅ **contracts/api-spec.yaml** created with OpenAPI specification
✅ **quickstart.md** created with setup and usage instructions
✅ **qwen-context.md** updated with relevant technology information

## Phase 2: Implementation Plan

### Component Breakdown

1. **Embeddings Module (`embeddings.py`)**:
   - Initialize FastEmbed model
   - Generate embeddings for text chunks
   - Handle model loading errors

2. **Vector Store Module (`vector_store.py`)**:
   - Connect to Qdrant Cloud
   - Create "book" collection with 384-dim vectors and cosine similarity
   - Implement document storage and retrieval methods
   - Handle connection errors gracefully

3. **Document Indexer (`indexer.py`)**:
   - Recursively scan ../docs for .md and .mdx files
   - Chunk documents appropriately
   - Generate embeddings for chunks
   - Store in Qdrant with metadata

4. **RAG Service (`rag.py`)**:
   - Query handling with context retrieval from Qdrant
   - Answer generation using OpenAI API
   - Selection-based answering using provided text
   - "I don't know" responses when context is insufficient
   - Error handling for external API calls

5. **API Application (`main.py`)**:
   - FastAPI application with logging
   - Health endpoint
   - Query endpoint with Pydantic models
   - Selection endpoint with Pydantic models
   - Error handling middleware
   - Structured JSON responses

6. **Environment Configuration (`.env.example`)**:
   - QDRANT_URL
   - QDRANT_API_KEY
   - GEMINI_API_KEY

## Dependencies and Sequencing

### Implementation Sequence

1. Create the `embeddings.py` module (no external dependencies)
2. Create the `vector_store.py` module (depends on embeddings and Qdrant)
3. Create the `rag.py` module (depends on embeddings, vector_store, and OpenAI)
4. Create the `indexer.py` script (depends on embeddings and vector_store)
5. Update `pyproject.toml` with all required dependencies
6. Create the `main.py` FastAPI application (depends on rag service)
7. Write tests for all components

### Critical Dependencies

- FastEmbed model loading requires proper initialization
- Qdrant connection needs valid credentials
- OpenAI API calls require API key and network connectivity

## Design Decisions Requiring ADRs

1. **Embedding Model Selection**: Using FastEmbed MiniLM-L6-v2 was chosen for its efficiency and 384-dimension output. This decision impacts performance and accuracy.
2. **Vector Database Choice**: Qdrant was selected for its semantic search capabilities and ease of integration with our stack.
3. **API Architecture**: Using FastAPI with Pydantic models provides automatic validation and documentation, which impacts development speed and reliability.

## Validation and Quality Gates

### Unit Tests
- Test embedding generation accuracy
- Test vector storage and retrieval
- Test RAG query and selection logic
- Test API endpoint responses

### Integration Tests
- Test end-to-end flow from document indexing to query response
- Verify "I don't know" responses for insufficient context
- Validate error handling under various failure conditions
- Test with different document types (.md, .mdx)

### Quality Checks
- All API responses must use structured JSON
- All external API calls must have try/except blocks
- All endpoints must have proper status codes
- All operations must be reproducible across environments
- Logging must be implemented for indexing and inference

### Performance Criteria
- Query responses under 5 seconds
- Proper handling of large documents
- Efficient memory usage during indexing
- Proper rate limiting for external API calls

## Outcome

- ✅ Impact: Created comprehensive implementation plan with all required artifacts
- ✅ Phase 0 research completed with technology decisions documented
- ✅ Phase 1 design completed with data models, contracts, quickstart guide, and agent context
- 🧪 Tests: none yet; will be created during implementation
- 📁 Files: plan.md, tasks.md, data-model.md, research.md, quickstart.md, contracts/api-spec.yaml, qwen-context.md
- 🔁 Next prompts: Implementation of the backend RAG system according to plan
- 🧠 Reflection: The plan follows spec-driven development principles and addresses all requirements from the feature specification

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementation following the created tasks