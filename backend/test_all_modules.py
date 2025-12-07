"""
Comprehensive unit tests for the Backend RAG System.
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
import os
from embeddings import EmbeddingService
from vector_store import VectorStore, DocumentChunk
from rag import RAGService, QueryRequest, QueryResponse, SelectionRequest, SelectionResponse


class TestEmbeddingService(unittest.TestCase):
    def setUp(self):
        self.embedding_service = EmbeddingService()

    def test_384_dimension_embeddings(self):
        """Test that embeddings are 384-dimensional as required."""
        text = "This is a test sentence."
        embedding = self.embedding_service.embed_text(text)
        self.assertEqual(len(embedding), 384)
        
        # Test multiple texts
        texts = ["First sentence.", "Second sentence.", "Third sentence."]
        embeddings = self.embedding_service.embed_texts(texts)
        self.assertEqual(len(embeddings), 3)
        for embedding in embeddings:
            self.assertEqual(len(embedding), 384)


class TestVectorStore(unittest.TestCase):
    def setUp(self):
        # Mock the Qdrant client to avoid needing real credentials for tests
        with patch('vector_store.QdrantClient'):
            self.vector_store = VectorStore()
            self.vector_store.client = Mock()

    def test_store_document_chunk(self):
        """Test storing a single document chunk."""
        chunk = DocumentChunk(
            chunk_id="test_chunk_1",
            content="Test content for chunk",
            doc_path="/test/doc.md",
            embedding=[0.1] * 384  # 384-dim embedding
        )
        
        result = self.vector_store.store_document_chunk(chunk)
        self.assertTrue(result)
        self.vector_store.client.upsert.assert_called_once()

    def test_search(self):
        """Test searching for similar documents."""
        query_embedding = [0.1] * 384  # 384-dim embedding
        
        # Mock search results
        mock_result = Mock()
        mock_result.payload = {
            "content": "Test content",
            "doc_path": "/test/doc.md",
            "metadata": {}
        }
        mock_result.score = 0.95
        
        self.vector_store.client.search.return_value = [mock_result]
        
        results = self.vector_store.search(query_embedding, limit=1)
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0]["content"], "Test content")
        self.assertEqual(results[0]["doc_path"], "/test/doc.md")


class TestRAGService(unittest.TestCase):
    def setUp(self):
        # Mock the OpenAI client to avoid needing real credentials for tests
        with patch.dict(os.environ, {"GEMINI_API_KEY": "test_key"}):
            self.rag_service = RAGService()
        
        # Mock vector store
        self.mock_vector_store = Mock()
        self.rag_service.set_vector_store(self.mock_vector_store)

    def test_query_with_context(self):
        """Test query processing with context."""
        # Mock the embedding service to return a fixed embedding
        with patch('rag.EmbeddingService') as mock_emb_class:
            mock_emb_service = Mock()
            mock_emb_service.embed_text.return_value = [0.1] * 384
            mock_emb_class.return_value = mock_emb_service
            
            # Mock vector store search results
            mock_docs = [
                {
                    "content": "Test context content",
                    "doc_path": "/test/doc.md",
                    "score": 0.95
                }
            ]
            self.mock_vector_store.search.return_value = mock_docs
            
            # Mock OpenAI response
            mock_response = Mock()
            mock_response.choices = [Mock()]
            mock_response.choices[0].message = Mock()
            mock_response.choices[0].message.content = "Test answer based on context"
            
            with patch.object(self.rag_service.openai_client.chat.completions, 'create', return_value=mock_response):
                result = self.rag_service.query("Test query?")
                
                self.assertIsInstance(result, QueryResponse)
                self.assertEqual(result.answer, "Test answer based on context")
                self.assertIn("/test/doc.md", result.sources)

    def test_query_insufficient_context(self):
        """Test query response when context is insufficient."""
        # Mock the embedding service to return a fixed embedding
        with patch('rag.EmbeddingService') as mock_emb_class:
            mock_emb_service = Mock()
            mock_emb_service.embed_text.return_value = [0.1] * 384
            mock_emb_class.return_value = mock_emb_service
            
            # Mock empty search results
            self.mock_vector_store.search.return_value = []
            
            result = self.rag_service.query("Test query?")
            
            self.assertIsInstance(result, QueryResponse)
            self.assertEqual(result.answer, "I don't know")

    def test_answer_from_selection(self):
        """Test selection-based answering."""
        # Mock OpenAI response
        mock_response = Mock()
        mock_response.choices = [Mock()]
        mock_response.choices[0].message = Mock()
        mock_response.choices[0].message.content = "Test answer based on selection"
        
        with patch.object(self.rag_service.openai_client.chat.completions, 'create', return_value=mock_response):
            result = self.rag_service.answer_from_selection("Selected text", "Question about text?")
            
            self.assertIsInstance(result, SelectionResponse)
            self.assertEqual(result.answer, "Test answer based on selection")


if __name__ == '__main__':
    unittest.main()