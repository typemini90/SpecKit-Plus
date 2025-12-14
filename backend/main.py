import logging
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from agents import Runner
from simple_agents.aagents import Triage_Agent
from pydantic import BaseModel
from services.rag import RAGService
from data.vector_store import VectorStore

# Initialize services globally but handle initialization errors gracefully
try:
    vector_store = VectorStore()
    rag_service = RAGService()
    rag_service.set_vector_store(vector_store)
except Exception as e:
    logging.error(f"Failed to initialize services: {e}")
    vector_store = None
    rag_service = None

app = FastAPI()

# CORS middleware for Vercel deployment
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://muhammedsuhaib.github.io",
        "http://localhost:3000",
        "http://localhost:5173",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ---------------------------
# Pydantic Models for Frontend Requests
# ---------------------------

# Matches the payload for the general chat endpoint (/api/query)
class QueryRequest(BaseModel):
    query: str

# Matches the payload for the selection endpoint (/api/selection)
class SelectionRequest(BaseModel):
    selected_text: str
    question: str

# Matches the payload for the translation endpoint (/api/translate-text)
class TranslationRequest(BaseModel):
    text: str
    target_language: str

# ---------------------------
# FastAPI Endpoints (Matching React expectations)
# ---------------------------

@app.get("/")
def read_root():
    return {"message": "Python Assistant Backend is running."}

@app.post("/api/query")
async def handle_query(req: QueryRequest):
    """Handles general chat queries from the React component."""
    logging.info(f"Received general query: {req.query}")

    # Check if services are properly initialized
    if not rag_service or not vector_store:
        logging.error("RAG service not initialized")
        return {
            "answer": "Service temporarily unavailable",
            "sources": []
        }

    # Use global RAG service to get context from Qdrant
    # Get relevant context from Qdrant
    try:
        rag_result = await rag_service.query(req.query)
        print(rag_result)
        print(rag_result.sources)
        context = rag_result.answer if rag_result.answer != "I don't know" else ""
    except Exception as e:
        logging.error(f"RAG query failed: {e}")
        # Fallback to no context if RAG fails
        rag_result = None
        context = ""

    # Include context in the agent's query if available
    if context and context != "I don't know":
        enhanced_query = f"Based on the following context: {context}\n\nQuestion: {req.query}"
    else:
        enhanced_query = req.query

    # Run the main agent with the enhanced query
    result = await Runner.run(
        Triage_Agent,
        enhanced_query
    )

    # CRITICAL: Response structure must match React component: {"answer": "...", "sources": []}
    return {
        "answer": result.final_output,
        "sources": rag_result.sources if rag_result and hasattr(rag_result, 'sources') else [] # Must be included, even if empty
    }

@app.post("/api/selection")
async def handle_selection(req: SelectionRequest):
    """Handles queries based on selected text (RAG context)."""
    logging.info(f"Received selection query. Question: {req.question}")

    # Check if services are properly initialized
    if not rag_service or not vector_store:
        logging.error("RAG service not initialized")
        return {
            "answer": "Service temporarily unavailable",
            "sources": []
        }

    # Use global RAG service to get additional context from Qdrant
    # Get relevant context from Qdrant based on the question
    try:
        rag_result = await rag_service.query(req.question)
        additional_context = rag_result.answer if rag_result.answer != "I don't know" else ""
    except Exception as e:
        logging.error(f"RAG query failed: {e}")
        # Fallback to no context if RAG fails
        rag_result = None
        additional_context = ""

    # Construct a RAG-style prompt for the agent
    if additional_context and additional_context != "I don't know":
        prompt = (
            f"Based *only* on the following context, answer the user's question. "
            f"If the context does not contain the answer, state that. "
            f"Context: \"{req.selected_text}\"\n\nAdditional context from knowledge base: {additional_context} "
            f"Question: {req.question}"
        )
    else:
        prompt = (
            f"Based *only* on the following context, answer the user's question. "
            f"If the context does not contain the answer, state that. "
            f"Context: \"{req.selected_text}\" "
            f"Question: {req.question}"
        )
    # Run the agent with the context-aware prompt
    result = await Runner.run(
        Triage_Agent,
        prompt
    )

    # CRITICAL: Response structure must match React component: {"answer": "...", "sources": []}
    return {
        "answer": result.final_output,
        "sources": rag_result.sources if rag_result and hasattr(rag_result, 'sources') else [] # Must be included, even if empty
    }

@app.get("/health")
def health_check():
    """Health check endpoint for Vercel deployment."""
    return {"status": "healthy", "message": "Backend is running"}

@app.post("/api/translate-text")
async def translate_text(req: TranslationRequest):
    """Translates text to the specified target language."""
    from deep_translator import GoogleTranslator

    try:
        # Validate target language
        if req.target_language != 'ur':
            return {"error": "Currently only Urdu (ur) translation is supported"}

        # Perform translation
        translated = GoogleTranslator(source='en', target=req.target_language).translate(req.text)

        return {"translated_text": translated}
    except Exception as e:
        logging.error(f"Translation error: {e}")
        return {"error": str(e)}
