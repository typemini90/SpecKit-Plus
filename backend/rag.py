"""
Module for RAG (Retrieval Augmented Generation) functionality.
"""
from typing import List, Dict, Any, Optional
import logging
import os
from openai import OpenAI
from dotenv import load_dotenv
from pydantic import BaseModel
import asyncio

# Load environment variables
load_dotenv()

class QueryRequest(BaseModel):
    query: str

class QueryResponse(BaseModel):
    answer: str
    sources: List[str] = []

class SelectionRequest(BaseModel):
    selected_text: str
    question: str

class SelectionResponse(BaseModel):
    answer: str

class RAGService:
    def __init__(self):
        """
        Initialize the RAG service with required components.
        """
        try:
            # Initialize Qwen client with Qwen API
            qwen_api_key = os.getenv("QWEN_API_KEY")
            if not qwen_api_key:
                raise ValueError("QWEN_API_KEY must be set in environment variables")

            self.client = OpenAI(
                api_key=qwen_api_key,
                base_url="https://portal.qwen.ai/v1"
            )
            self.model = "qwen3-coder-plus"

            # We'll initialize vector store separately to avoid circular dependencies
            self.vector_store = None
        except Exception as e:
            logging.error(f"Failed to initialize RAG service: {e}")
            raise

    def check_api_connection(self):
        """
        Check if the Gemini API connection is working.

        Returns:
            True if connection is successful, False otherwise
        """
        try:
            # Make a simple test call to the API
            response = self.openai_client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are a test assistant."},
                    {"role": "user", "content": "Test connection"}
                ],
                max_tokens=10
            )
            return True
        except Exception as e:
            logging.error(f"API connection test failed: {e}")
            return False

    def set_vector_store(self, vector_store):
        """
        Set the vector store instance to use for retrieval.
        
        Args:
            vector_store: Initialized VectorStore instance
        """
        self.vector_store = vector_store

    def query(self, query_text: str) -> QueryResponse:
        """
        Process a query using the RAG pipeline.
        
        Args:
            query_text: User's question
            
        Returns:
            QueryResponse with answer and sources
        """
        try:
            # Step 1: Generate embedding for the query
            from embeddings import EmbeddingService
            embedding_service = EmbeddingService()
            query_embedding = embedding_service.embed_text(query_text)
            
            # Step 2: Retrieve relevant documents from vector store
            if not self.vector_store:
                raise ValueError("Vector store not set in RAG service")
            
            retrieved_docs = self.vector_store.search(query_embedding, limit=5)
            
            if not retrieved_docs:
                return QueryResponse(answer="I don't know", sources=[])
            
            # Step 3: Format context from retrieved documents
            context = "\n\n".join([doc["content"] for doc in retrieved_docs])
            sources = list(set([doc["doc_path"] for doc in retrieved_docs]))  # Unique sources
            
            # Step 4: Generate answer using OpenAI with the context
            prompt = f"""
            Context information is below:
            {context}
            
            Using the provided context information, answer the question: {query_text}
            
            If the context does not contain sufficient information to answer the question, respond with "I don't know".
            """
            
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that answers questions based only on the provided context. If the context does not contain sufficient information to answer the question, respond with 'I don't know'."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=500,
                temperature=0.1
            )
            
            answer = response.choices[0].message.content.strip()
            
            # Check if the answer is "I don't know"
            if "i don't know" in answer.lower():
                return QueryResponse(answer="I don't know", sources=[])
            
            return QueryResponse(answer=answer, sources=sources)
            
        except Exception as e:
            logging.error(f"Error in query processing: {e}")
            raise

    def answer_from_selection(self, selected_text: str, question: str) -> SelectionResponse:
        """
        Answer a question based only on the selected text.
        
        Args:
            selected_text: Text selected by the user
            question: Question about the selected text
            
        Returns:
            SelectionResponse with answer
        """
        try:
            # Format the prompt using only the selected text
            prompt = f"""
            Selected text is below:
            {selected_text}
            
            Using only the provided selected text, answer the question: {question}
            
            If the selected text does not contain sufficient information to answer the question, respond with "I don't know".
            """
            
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that answers questions based only on the provided selected text. If the selected text does not contain sufficient information to answer the question, respond with 'I don't know'."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=500,
                temperature=0.1
            )
            
            answer = response.choices[0].message.content.strip()
            
            # Check if the answer is "I don't know"
            if "i don't know" in answer.lower():
                return SelectionResponse(answer="I don't know")
            
            return SelectionResponse(answer=answer)
            
        except Exception as e:
            logging.error(f"Error in selection-based answering: {e}")
            raise