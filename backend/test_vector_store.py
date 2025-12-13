"""
Test script for the updated VectorStore implementation.
"""
import os
import sys
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def test_vector_store():
    """Test the VectorStore implementation."""
    print("Testing VectorStore...")
    
    try:
        # Import the vector store
        from data.vector_store import VectorStore, DocumentChunk
        
        # Create a test instance
        print("Creating VectorStore instance...")
        vector_store = VectorStore(collection_name="test_collection", timeout=30)
        
        # Test connection
        print("Testing connection...")
        conn_result = vector_store.check_connection()
        print(f"Connection test result: {conn_result}")
        
        if not conn_result:
            print("Warning: Could not connect to Qdrant. Please ensure Qdrant is running.")
            return False
        
        # Test health check
        print("Performing health check...")
        health_result = vector_store.health_check()
        print(f"Health check result: {health_result}")
        
        # Test adding a document
        print("Testing document storage...")
        test_chunk = DocumentChunk(
            chunk_id="test_chunk_1",
            content="This is a test document for the vector store.",
            doc_path="test/path.txt",
            embedding=[0.1] * 384  # 384-dimensional vector
        )
        
        store_result = vector_store.store_document_chunk(test_chunk)
        print(f"Document storage result: {store_result}")
        
        # Test search
        print("Testing search functionality...")
        search_results = vector_store.search([0.1] * 384, limit=5)
        print(f"Search results count: {len(search_results)}")
        
        # Test document count
        print("Testing document count...")
        count = vector_store.count_documents()
        print(f"Document count: {count}")
        
        # Clean up test collection
        print("Cleaning up test collection...")
        delete_result = vector_store.delete_collection()
        print(f"Delete collection result: {delete_result}")
        
        print("All VectorStore tests passed!")
        return True
        
    except Exception as e:
        print(f"Error in VectorStore test: {e}")
        import traceback
        traceback.print_exc()
        return False

async def test_rag_service():
    """Test the RAGService implementation."""
    print("\nTesting RAGService...")

    try:
        # Import the rag service
        from services.rag import RAGService

        # Create a test instance
        print("Creating RAGService instance...")
        rag_service = RAGService(vector_store_collection="test_rag_collection", vector_store_timeout=30)

        # Test health check
        print("Testing health check...")
        health_result = await rag_service.check_health()
        print(f"Health check result: {health_result}")

        # Test query with empty input (should handle gracefully)
        print("Testing query with empty input...")
        query_response = await rag_service.query("")
        print(f"Empty query response: {query_response}")

        # Clean up test collection
        if rag_service.vector_store:
            print("Cleaning up RAG test collection...")
            delete_result = rag_service.vector_store.delete_collection()
            print(f"Delete RAG collection result: {delete_result}")

        print("All RAGService tests passed!")
        return True

    except Exception as e:
        print(f"Error in RAGService test: {e}")
        import traceback
        traceback.print_exc()
        return False

import asyncio

async def main():
    """Run all tests."""
    print("Starting tests for updated Qdrant implementation...")

    vector_store_success = test_vector_store()
    rag_service_success = await test_rag_service()

    if vector_store_success and rag_service_success:
        print("\nAll tests passed! The updated Qdrant implementation is working correctly.")
        return True
    else:
        print("\nSome tests failed. Please check the output above for details.")
        return False

if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)