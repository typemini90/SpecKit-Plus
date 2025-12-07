"""
Integration tests for the Backend RAG System.
These tests verify that different components work together correctly.
"""
import unittest
from unittest.mock import patch, Mock
import os
from main import app
from fastapi.testclient import TestClient
from rag import QueryRequest, SelectionRequest


class TestAPIIntegration(unittest.TestCase):
    def setUp(self):
        self.client = TestClient(app)
        
        # Mock the services to avoid needing real credentials
        with patch('main.VectorStore'):
            with patch('main.RAGService'):
                # Mock vector store
                mock_vector_store = Mock()
                
                # Mock rag service
                mock_rag_service = Mock()
                mock_rag_service.query.return_value = {"answer": "Test answer", "sources": ["/test/doc.md"]}
                mock_rag_service.answer_from_selection.return_value = {"answer": "Test selection answer"}
                
                # Set the mocked services in the app
                from main import rag_service
                # In the actual app, we would set these, but for testing
                # the TestClient will handle the app lifecycle

    def test_health_endpoint(self):
        """Test the health check endpoint."""
        response = self.client.get("/api/health")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["status"], "ok")

    @patch.dict(os.environ, {"GEMINI_API_KEY": "test_key"})
    @patch('main.rag_service')
    def test_query_endpoint(self, mock_rag_service):
        """Test the query endpoint."""
        # Mock the response
        mock_response = Mock()
        mock_response.answer = "Test answer"
        mock_response.sources = ["/test/doc.md"]
        mock_rag_service.query.return_value = mock_response
        
        request_data = {"query": "What is the principle of force control in humanoid robotics?"}
        response = self.client.post("/api/query", json=request_data)
        
        self.assertEqual(response.status_code, 200)
        response_json = response.json()
        self.assertIn("answer", response_json)
        self.assertIn("sources", response_json)

    @patch.dict(os.environ, {"GEMINI_API_KEY": "test_key"})
    @patch('main.rag_service')
    def test_selection_endpoint(self, mock_rag_service):
        """Test the selection endpoint."""
        # Mock the response
        mock_response = Mock()
        mock_response.answer = "Test selection answer"
        mock_rag_service.answer_from_selection.return_value = mock_response
        
        request_data = {
            "selected_text": "Force control is a crucial aspect of humanoid robotics...",
            "question": "How does force control work?"
        }
        response = self.client.post("/api/selection", json=request_data)
        
        self.assertEqual(response.status_code, 200)
        response_json = response.json()
        self.assertIn("answer", response_json)
        self.assertEqual(response_json["answer"], "Test selection answer")


if __name__ == '__main__':
    unittest.main()