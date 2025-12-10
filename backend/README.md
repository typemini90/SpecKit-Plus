# Backend RAG System

This backend system implements a Retrieval Augmented Generation (RAG) solution for the Physical AI & Humanoid Robotics project.

## Core Principles

This project adheres to the following core principles as defined in our [Constitution](../.specify/memory/constitution.md):

- **Reliability and deterministic behavior**: Backend operations are predictable and consistent across different environments and runs
- **Clean, reproducible indexing and embeddings**: The indexing process is reproducible with clear inputs and verifiable outputs
- **Clear separation between indexing and runtime serving**: Indexing operations are functionally separate from runtime query operations
- **No hardcoded secrets**: All sensitive information is stored in environment variables
- **Strict error handling and structured JSON responses**: Comprehensive error handling with descriptive messages
- **Predictable, testable RAG pipeline**: Designed for testability with clearly defined inputs, outputs, and transformation steps

## Quality Standards

All components adhere to these quality standards:
- All endpoints use Pydantic schemas for request/response validation
- All errors use structured JSON and proper status codes
- Every external API call is wrapped in try/except blocks
- Logging required for indexing, search, and inference operations
- All operations are reproducible across machines
- Vector dimensions strictly match model + Qdrant configuration
- System responds with "I don't know" when context is insufficient
- No silent failures

## Backend Constraints

- FastAPI server framework
- uv for dependency management
- Running locally on port 8000
- Indexer reads all .md and .mdx files under ../docs
- Supports re-indexing operations
- Supports both query-based and selection-based RAG
- Embedding model: FastEmbed BGE small (384 dimensions)

## Environment Variables

Before starting the application, ensure you set the following environment variables in a `.env` file:

```
QWEN_API_KEY=your-qwen-api-key-here
QDRANT_URL=your-qdrant-url-here
QDRANT_API_KEY=your-qdrant-api-key-here
```

## Getting Started

1. Install dependencies: `uv sync` (or `pip install -r requirements.txt`)
2. Set up environment variables (see `.env.example` and the "Environment Variables" section above)
3. Index your documents (optional but recommended for RAG functionality):
   ```bash
   uv run python indexer.py
   ```
4. Start the server:
   ```bash
   uv run uvicorn main:app --reload
   ```
5. API documentation available at `http://localhost:8000/docs`
6. Health check endpoint at `http://localhost:8000/api/health`

## API Endpoints

- `GET /api/health` - Health check with detailed status of services
- `POST /api/query` - Query endpoint for asking questions against indexed documents
- `POST /api/selection` - Selection endpoint for asking questions about selected text
- `GET /chat/health` - ChatKit API health check
- `POST /chat/chatkit` - ChatKit API endpoint

## Frontend Integration

The frontend component (`ChatKitInterface.jsx`) has been updated to connect directly to the RAG API endpoints rather than using the non-existent ChatKit React library. It now:
- Makes direct API calls to `/api/query` and `/api/selection` endpoints
- Handles both general queries and selection-based queries
- Displays source documents with responses
- Provides better error handling and user feedback

## Running the Application

To run the full application:

1. Start the backend server:
   ```bash
   cd backend
   uv run uvicorn main:app --reload
   ```

2. In a separate terminal, start the frontend:
   ```bash
   cd frontend
   npm run start
   ```

The application will be available at `http://localhost:3000`

## Troubleshooting

- If you get API key errors, ensure your environment variables are set correctly
- If Qdrant connection fails, verify your Qdrant configuration and credentials
- If document indexing fails, check that the `../docs` directory exists and contains readable files
- Check the detailed health endpoint at `/api/health` to diagnose service availability