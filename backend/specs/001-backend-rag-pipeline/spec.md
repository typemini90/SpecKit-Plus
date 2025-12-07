# Feature Specification: Backend RAG Pipeline

**Feature Branch**: `001-backend-rag-pipeline`
**Created**: 2025-12-07
**Status**: Draft
**Input**: User description: "Backend-Only RAG System for Physical AI & Humanoid Robotics Textbook Intent: Build a backend-only RAG pipeline that indexes the entire Docusaurus textbook and serves two APIs: 1) /api/query — answer using retrieved context 2) /api/selection — answer ONLY from selected text The backend runs independently from the frontend. Target User: Students and readers of the Physical AI & Humanoid Robotics textbook who ask questions and receive answers sourced strictly from the book. Success Criteria (SMART): - Embedding model: FastEmbed MiniLM-L6-v2 (384-dim) is used consistently across indexing and querying. - Indexing reads all .md and .mdx files under ../docs recursively. - Qdrant collection "book" is created with 384-dim vectors and cosine similarity. - /api/health returns 200 with JSON {status:"ok"}. - /api/query returns contextual answers using ONLY retrieved text; answers unrelated to context must reply "I don't know." - /api/selection uses ONLY user-selected text and never hits Qdrant. - All responses use structured JSON and proper HTTP status codes. - All external API calls include try/except + error logging. - Backend starts locally on http://localhost:8000 with uvicorn and works end-to-end in one run. - Code must be deployable to Render/Fly.io without modifications. Constraints: - Python 3.10+, uv for dependencies - Folders: /backend/, with indexer + FastAPI app - Qdrant Cloud required (URL + API key via env vars) - GEMINI_API_KEY via env vars - No frontend changes; agent owns ONLY backend folder - Must not hardcode secrets - Must be reproducible on any machine Non-Goals: - No UI or Docusaurus integration - No chatbot bubble or frontend code - No file uploads or PDF ingestion - No streaming responses - No extra features beyond: embeddings, indexing, RAG, selection-answering, APIs Deliverables: - index.py that chunks, embeds, and indexes docs - app.py that exposes /api/health, /api/query, /api/selection - env template - logging setup - clear instructions to run locally Timeline: Complete in the current development session."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Query with Context (Priority: P1)

Student asks a question about Physical AI concepts and receives an answer based strictly on the textbook content.

**Why this priority**: Core functionality that addresses the primary use case of the RAG system - allowing students to get answers from the textbook.

**Independent Test**: Student can submit a question via API and receive an answer sourced strictly from the textbook content. System should return "I don't know" when the question is unrelated to the context.

**Acceptance Scenarios**:

1. **Given** a student submits a relevant question about textbook content, **When** they call /api/query, **Then** they receive an answer based only on retrieved context
2. **Given** a student submits an irrelevant question, **When** they call /api/query, **Then** they receive a response stating "I don't know"
3. **Given** a student submits a query, **When** the system encounters an error during processing, **Then** they receive an appropriate error response with proper HTTP status code

---

### User Story 2 - Answer from Selected Text (Priority: P2)

Student selects specific text from the textbook and asks a question about that selection, receiving an answer based only on the selected text.

**Why this priority**: Provides an alternative way to interact with the system, allowing more targeted questions based on selected content.

**Independent Test**: Student can submit selected text and a question, and receive an answer based only on that text without consulting the full knowledge base.

**Acceptance Scenarios**:

1. **Given** a student selects specific text and submits a question, **When** they call /api/selection, **Then** they receive an answer based only on the selected text
2. **Given** a student submits selected text and an unrelated question, **When** they call /api/selection, **Then** they receive a response stating "I don't know"
3. **Given** a student submits malformed selection, **When** they call /api/selection, **Then** they receive an appropriate error response

---

### User Story 3 - Health Check and System Status (Priority: P3)

System administrator or frontend client can verify the backend service is running and operational.

**Why this priority**: Essential for monitoring system availability and for frontend applications to determine if the service is accessible.

**Independent Test**: Health check endpoint is accessible and returns operational status.

**Acceptance Scenarios**:

1. **Given** the system is running, **When** a client calls /api/health, **Then** they receive a 200 status with JSON {status: "ok"}
2. **Given** the system is down, **When** a client calls /api/health, **Then** they receive an appropriate error status

---

### Edge Cases

- What happens when the Qdrant service is unavailable?
- How does the system handle very long documents for selection-based queries?
- What if the embedding model service is temporarily unavailable?
- How does the system handle concurrent requests during indexing?
- What if the user submits a query that matches multiple sections of the textbook?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST index all .md and .mdx files under ../docs recursively using FastEmbed MiniLM-L6-v2 (384-dim)
- **FR-002**: System MUST create a Qdrant collection named "book" with 384-dim vectors and cosine similarity
- **FR-003**: Users MUST be able to submit questions to /api/query and receive answers based only on retrieved context
- **FR-004**: System MUST return "I don't know" when query context is insufficient or unrelated to textbook content
- **FR-005**: Users MUST be able to submit selected text and questions to /api/selection and receive answers based only on that text
- **FR-006**: System MUST return structured JSON responses with appropriate HTTP status codes for all endpoints
- **FR-007**: System MUST implement try/except error handling with logging for all external API calls
- **FR-008**: System MUST expose a health check endpoint at /api/health returning 200 with JSON {status: "ok"}
- **FR-009**: System MUST start locally on http://localhost:8000 and work end-to-end in one run
- **FR-010**: System MUST be deployable to Render/Fly.io without modifications

### Key Entities *(include if feature involves data)*

- **Document Chunk**: Represents a segment of text extracted from .md/.mdx files, with associated embedding vector
- **Query**: A question submitted by the user for contextual answer generation
- **Selection**: User-selected text content provided for targeted answer generation
- **Response**: Structured answer from the system with appropriate status information

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Embedding model FastEmbed MiniLM-L6-v2 (384-dim) is used consistently across indexing and querying operations
- **SC-002**: Indexing process successfully reads and processes all .md and .mdx files under ../docs recursively
- **SC-003**: Qdrant collection "book" is created with 384-dim vectors and cosine similarity configuration
- **SC-004**: /api/health endpoint returns 200 status with JSON {status: "ok"} when the system is operational
- **SC-005**: /api/query returns contextual answers using ONLY retrieved text with 95% accuracy in context-relevance
- **SC-006**: /api/query returns "I don't know" response for queries unrelated to context with 95% accuracy
- **SC-007**: /api/selection uses ONLY user-selected text and never consults Qdrant, with 100% compliance
- **SC-008**: 100% of all responses use structured JSON format with proper HTTP status codes
- **SC-009**: 100% of external API calls include try/except error handling with proper logging
- **SC-010**: Backend starts successfully on http://localhost:8000 and works end-to-end in one run with 99% uptime during testing
- **SC-011**: Code deploys successfully to Render/Fly.io without modifications and passes basic functionality tests
- **SC-012**: All operations complete reproducibly across different machines without configuration adjustments