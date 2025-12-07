# Quickstart Guide: Backend RAG Pipeline

## Overview
This guide provides step-by-step instructions for setting up and running the Backend RAG System for Physical AI & Humanoid Robotics.

## Prerequisites
- Python 3.10 or higher
- uv package manager (or pip)
- Access to Qdrant Cloud instance
- Gemini API key

## Setup Instructions

### 1. Clone and Navigate to Project
```bash
cd /path/to/dockathon/backend
```

### 2. Install Dependencies
Using uv (recommended):
```bash
uv sync
```

Or using pip:
```bash
pip install -r requirements.txt
```

### 3. Configure Environment Variables
Create a `.env` file based on the `.env.example` template:
```bash
cp .env.example .env
```

Edit the `.env` file and add your credentials:
```
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
GEMINI_API_KEY=your_gemini_api_key
```

### 4. Index Documents
Run the indexer to process .md and .mdx files from the `../docs` directory:
```bash
python indexer.py
```

This will:
- Recursively scan `../docs` for .md and .mdx files
- Chunk documents into appropriate segments
- Generate 384-dimensional embeddings using FastEmbed
- Store chunks in Qdrant vector database

### 5. Start the Server
Run the FastAPI application:
```bash
uv run python main.py
```

The server will start on `http://localhost:8000` with:
- API endpoints at `/api/*`
- Interactive API documentation at `/docs`

## API Usage

### Health Check
Verify the service is operational:
```bash
curl -X GET "http://localhost:8000/api/health"
```

Expected response:
```json
{
  "status": "ok"
}
```

### Query Endpoint
Submit a question to get an answer based on indexed documents:
```bash
curl -X POST "http://localhost:8000/api/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is the principle of force control in humanoid robotics?"
  }'
```

Expected response:
```json
{
  "answer": "Force control in humanoid robotics involves...",
  "sources": ["/docs/introduction.md", "/docs/force-control.md"]
}
```

### Selection Endpoint
Submit selected text and a question to get an answer based only on the provided text:
```bash
curl -X POST "http://localhost:8000/api/selection" \
  -H "Content-Type: application/json" \
  -d '{
    "selected_text": "Force control is a crucial aspect of humanoid robotics. It involves precise manipulation...",
    "question": "How does force control work in humanoid robotics?"
  }'
```

Expected response:
```json
{
  "answer": "Based on the selected text, force control works by..."
}
```

## Troubleshooting

### Common Issues
1. **Environment Variables Not Set**: Ensure all required environment variables are properly set in your `.env` file
2. **Qdrant Connection Issues**: Verify QDRANT_URL and QDRANT_API_KEY are correct
3. **Gemini API Errors**: Check that GEMINI_API_KEY is valid and has sufficient quota
4. **Document Processing Issues**: Verify that `../docs` directory exists and contains .md/.mdx files

### Logging
The application uses loguru for logging. Check logs for detailed error information during indexing or API operations.