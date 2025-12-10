"""
Main FastAPI application for the Backend RAG System with ChatKit integration.
"""
import logging
from contextlib import asynccontextmanager
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Optional
from loguru import logger
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import our modules (these will be imported after they are defined)
from embeddings import EmbeddingService
from vector_store import VectorStore
from rag import RAGService, QueryRequest, QueryResponse, SelectionRequest, SelectionResponse

# Import unified ChatKit API with RAG capabilities
from chatkit_service import app as chatkit_app

# Global variable to hold the RAG service instance
rag_service = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan event handler for FastAPI application.
    Initializes services on startup and cleans up on shutdown.
    """
    # Startup
    global rag_service
    try:
        # Check if required environment variables are set
        qwen_api_key = os.getenv("QWEN_API_KEY")
        if not qwen_api_key:
            logger.warning("QWEN_API_KEY not set. RAG service will not function properly.")

        # Only initialize RAG service if Qdrant credentials are available (for RAG features)
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if qdrant_url and qdrant_api_key:
            try:
                # Initialize the vector store and RAG service for full functionality
                vector_store = VectorStore()

                # Test vector store connection
                if vector_store.check_connection():
                    logger.info("Qdrant connection successful")
                else:
                    logger.error("Could not connect to Qdrant. RAG service will not function properly.")
                    vector_store = None
            except Exception as e:
                logger.error(f"Failed to initialize vector store: {e}")
                vector_store = None

            if vector_store:
                try:
                    rag_service = RAGService()

                    # Test API connection
                    if rag_service.check_api_connection():
                        logger.info("OpenAI API connection successful")
                    else:
                        logger.error("Could not connect to OpenAI API. RAG service will not function properly.")
                        rag_service = None
                except Exception as e:
                    logger.error(f"Failed to initialize RAG service: {e}")
                    rag_service = None

                if rag_service and vector_store:
                    rag_service.set_vector_store(vector_store)
                    logger.info("Full RAG services initialized successfully")
                else:
                    logger.error("RAG service initialization failed due to API or vector store connection issues.")
                    rag_service = None
            else:
                logger.error("Vector store initialization failed. RAG service will not be available.")
                rag_service = None
        else:
            # For MVP without Qdrant, just initialize the ChatKit service but warn about missing RAG
            logger.info("Qdrant not configured - Only ChatKit service available, RAG functionality will be limited")
            rag_service = None

        yield
    except Exception as e:
        logger.error(f"Error during application startup: {e}")
        raise
    finally:
        # Shutdown
        logger.info("Application shutdown")

# Create the main FastAPI app with lifespan
app = FastAPI(
    title="Backend RAG API with ChatKit",
    description="API for the Backend RAG System for Physical AI & Humanoid Robotics with ChatKit integration",
    version="1.0.0",
    lifespan=lifespan
)

# Mount the ChatKit API
app.mount("/chat", chatkit_app)

# Add logging configuration
logging.basicConfig(level=logging.INFO)

class HealthResponse(BaseModel):
    status: str
    api_connected: bool = False
    vector_store_connected: bool = False
    rag_available: bool = False

@app.get("/api/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint to verify the service is operational.

    Returns:
        HealthResponse: Status of the service with detailed information
    """
    api_connected = False
    vector_store_connected = False
    rag_available = False

    # Check if RAG service is initialized and connections are working
    if rag_service:
        # Check if both API and vector store are connected
        try:
            api_connected = rag_service.check_api_connection()
        except:
            api_connected = False

        try:
            vector_store_connected = rag_service.vector_store.check_connection() if rag_service.vector_store else False
        except:
            vector_store_connected = False

        rag_available = api_connected and vector_store_connected

    return HealthResponse(
        status="ok" if rag_available else "degraded",
        api_connected=api_connected,
        vector_store_connected=vector_store_connected,
        rag_available=rag_available
    )

@app.post("/api/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """
    Query endpoint that processes user questions against the indexed documents.

    Args:
        request: QueryRequest containing the user's question

    Returns:
        QueryResponse with the answer and source documents
    """
    try:
        if not rag_service:
            # Check if the issue is due to missing configuration
            qwen_key_missing = not os.getenv("QWEN_API_KEY")
            qdrant_config_missing = not os.getenv("QDRANT_URL") or not os.getenv("QDRANT_API_KEY")

            missing_parts = []
            if qwen_key_missing:
                missing_parts.append("QWEN_API_KEY")
            if qdrant_config_missing:
                missing_parts.append("QDRANT configuration")

            error_msg = f"RAG service not available. Missing: {', '.join(missing_parts) if missing_parts else 'Unknown issue'}"
            raise HTTPException(status_code=503, detail=error_msg)

        # Process the query using the RAG service
        response = rag_service.query(request.query)
        return response
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing query: {e}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")

@app.post("/api/selection", response_model=SelectionResponse)
async def selection_endpoint(request: SelectionRequest):
    """
    Selection endpoint that answers questions based only on provided text.

    Args:
        request: SelectionRequest containing selected text and question

    Returns:
        SelectionResponse with the answer
    """
    try:
        if not rag_service:
            # Check if the issue is due to missing configuration
            qwen_key_missing = not os.getenv("QWEN_API_KEY")

            missing_parts = []
            if qwen_key_missing:
                missing_parts.append("QWEN_API_KEY")

            error_msg = f"Selection service not available. Missing: {', '.join(missing_parts) if missing_parts else 'Unknown issue'}"
            raise HTTPException(status_code=503, detail=error_msg)

        # Process the selection-based request
        response = rag_service.answer_from_selection(request.selected_text, request.question)
        return response
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing selection: {e}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")

# Include middleware for error handling and logging
@app.middleware("http")
async def log_requests(request, call_next):
    logger.info(f"Request: {request.method} {request.url}")
    try:
        response = await call_next(request)
        logger.info(f"Response status: {response.status_code}")
        return response
    except Exception as e:
        logger.error(f"Request failed: {e}")
        raise