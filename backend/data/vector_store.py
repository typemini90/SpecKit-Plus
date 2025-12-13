"""
Module for interacting with Qdrant vector database.
"""
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from typing import List, Dict, Any, Optional
import logging
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from pydantic import BaseModel
import os
from dotenv import load_dotenv
load_dotenv()

class DocumentChunk(BaseModel):
    chunk_id: str
    content: str
    doc_path: str
    embedding: List[float]
    metadata: Dict[str, Any] = {}

class VectorStore:
    def __init__(self,
                 collection_name: str = "Humanoids",
                 timeout: int = 30,
                 grpc_port: int = 6334,
                 prefer_grpc: bool = True,
                 recreate_collection: bool = False):
        """
        Initialize the vector store with Qdrant client.

        Args:
            collection_name: Name of the Qdrant collection to use
            timeout: Timeout for Qdrant API requests in seconds
            grpc_port: gRPC port for Qdrant communication
            prefer_grpc: Whether to prefer gRPC communication (faster than HTTP)
            recreate_collection: Whether to recreate the collection if it exists
        """
        try:
            qdrant_url = os.getenv("QDRANT_URL")
            qdrant_api_key = os.getenv("QDRANT_API_KEY")

            if not qdrant_url or not qdrant_api_key:
                raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set in environment variables")

            self.client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
                timeout=timeout,
                grpc_port=grpc_port,
                prefer_grpc=prefer_grpc
            )
            self.collection_name = collection_name
            self.timeout = timeout
            self.recreate_collection = recreate_collection

            # Create or verify the collection exists
            self._ensure_collection_exists()
        except Exception as e:
            logging.error(f"Failed to initialize Qdrant client: {e}")
            raise

    def check_connection(self):
        """
        Check if the Qdrant connection is working.

        Returns:
            True if connection is successful, False otherwise
        """
        try:
            # Try to get collections to verify connection
            # Note: get_collections doesn't accept timeout parameter, but uses the client's global timeout
            collections = self.client.get_collections()
            logging.info("Qdrant connection test succeeded")
            return True
        except Exception as e:
            logging.error(f"Qdrant connection test failed: {e}")
            return False

    def _ensure_collection_exists(self):
        """
        Ensure the Qdrant collection exists with proper configuration.
        """
        try:
            # Check if collection exists
            # Note: get_collections doesn't accept timeout parameter, but uses the client's global timeout
            collections = self.client.get_collections().collections
            collection_names = [c.name for c in collections]

            collection_exists = self.collection_name in collection_names

            if collection_exists and self.recreate_collection:
                # Delete and recreate the collection
                self.client.delete_collection(collection_name=self.collection_name, timeout=self.timeout)
                collection_exists = False
                logging.info(f"Deleted Qdrant collection '{self.collection_name}' for recreation")

            if not collection_exists:
                # Create collection with 384-dimensional vectors and cosine similarity
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(size=384, distance=Distance.COSINE),
                    timeout=self.timeout
                )
                logging.info(f"Created Qdrant collection '{self.collection_name}' with 384-dim vectors and cosine similarity")
            else:
                # Verify collection configuration matches expected settings
                # Note: get_collection doesn't accept timeout parameter, but uses the client's global timeout
                collection_info = self.client.get_collection(
                    collection_name=self.collection_name
                )

                expected_size = 384
                expected_distance = Distance.COSINE

                # Check if vector configuration matches expectations
                if hasattr(collection_info.config.params, 'vectors'):
                    vec_params = collection_info.config.params.vectors
                    if hasattr(vec_params, 'size') and vec_params.size != expected_size:
                        logging.warning(f"Collection '{self.collection_name}' has unexpected vector size: {vec_params.size}, expected: {expected_size}")
                    if hasattr(vec_params, 'distance') and vec_params.distance != expected_distance:
                        logging.warning(f"Collection '{self.collection_name}' has unexpected distance metric: {vec_params.distance}, expected: {expected_distance}")

                logging.info(f"Qdrant collection '{self.collection_name}' already exists with proper configuration")
        except Exception as e:
            logging.error(f"Failed to create or verify Qdrant collection: {e}")
            raise

    def validate_embedding_dimensions(self, embedding: List[float]) -> bool:
        """Validate that the embedding has the correct dimensions for this collection."""
        expected_size = 384  # As configured in the collection
        if len(embedding) != expected_size:
            raise ValueError(f"Embedding dimension mismatch: got {len(embedding)}, expected {expected_size}")
        return True

    def store_document_chunk(self, chunk: DocumentChunk) -> bool:
        """
        Store a document chunk in the vector database.

        Args:
            chunk: DocumentChunk object containing content and embedding

        Returns:
            True if successful, False otherwise
        """
        try:
            # Validate embedding dimensions
            self.validate_embedding_dimensions(chunk.embedding)

            # Ensure chunk_id is a proper integer or UUID
            # Convert string IDs to integer if possible, otherwise generate an integer ID
            try:
                point_id = int(chunk.chunk_id) if chunk.chunk_id.isdigit() else hash(chunk.chunk_id) % (10**9)
            except (ValueError, AttributeError):
                # Fallback to hash of content
                import hashlib
                point_id = int(hashlib.md5(chunk.content.encode()).hexdigest(), 16) % (10**9)

            # Prepare the point for Qdrant
            points = [
                models.PointStruct(
                    id=point_id,
                    vector=chunk.embedding,
                    payload={
                        "content": chunk.content,
                        "doc_path": chunk.doc_path,
                        "metadata": chunk.metadata
                    }
                )
            ]

            # Upload the point to Qdrant
            # Note: upsert doesn't accept timeout parameter, but uses the client's global timeout
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            return True
        except Exception as e:
            logging.error(f"Failed to store document chunk: {e}")
            return False

    def store_document_chunks(self, chunks: List[DocumentChunk]) -> bool:
        """
        Store multiple document chunks in the vector database using batch operations.

        Args:
            chunks: List of DocumentChunk objects to store

        Returns:
            True if successful, False otherwise
        """
        try:
            if not chunks:
                logging.warning("No chunks to store")
                return True

            # Validate all embeddings have correct dimensions
            for chunk in chunks:
                self.validate_embedding_dimensions(chunk.embedding)

            # Prepare the points for Qdrant
            points = []
            for chunk in chunks:
                # Ensure chunk_id is a proper integer or UUID
                try:
                    point_id = int(chunk.chunk_id) if chunk.chunk_id.isdigit() else hash(chunk.chunk_id) % (10**9)
                except (ValueError, AttributeError):
                    # Fallback to hash of content
                    import hashlib
                    point_id = int(hashlib.md5(chunk.content.encode()).hexdigest(), 16) % (10**9)

                points.append(
                    models.PointStruct(
                        id=point_id,
                        vector=chunk.embedding,
                        payload={
                            "content": chunk.content,
                            "doc_path": chunk.doc_path,
                            "metadata": chunk.metadata
                        }
                    )
                )

            # Upload the points to Qdrant in batches for better performance
            batch_size = 64  # Recommended batch size for performance
            for i in range(0, len(points), batch_size):
                batch = points[i:i + batch_size]
                # Note: upsert doesn't accept timeout parameter, but uses the client's global timeout
                self.client.upsert(
                    collection_name=self.collection_name,
                    points=batch
                )

            logging.info(f"Successfully stored {len(chunks)} document chunks in batch")
            return True
        except Exception as e:
            logging.error(f"Failed to store document chunks: {e}")
            return False

    def search(self, query_embedding: List[float], limit: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar document chunks based on the query embedding.

        Args:
            query_embedding: 384-dimensional embedding vector to search for
            limit: Maximum number of results to return

        Returns:
            List of documents with similarity scores
        """
        try:
            # Validate embedding dimensions
            self.validate_embedding_dimensions(query_embedding)

            # Perform the search in Qdrant with timeout
            # Use query_points method which is the new universal method for searching
            search_results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=limit,
                timeout=self.timeout
            )

            # Format the results
            results = []
            for result in search_results.points:
                results.append({
                    "content": result.payload["content"],
                    "doc_path": result.payload["doc_path"],
                    "metadata": result.payload.get("metadata", {}),
                    "score": result.score
                })

            return results
        except Exception as e:
            logging.error(f"Failed to search in vector store: {e}")
            return []

    def delete_collection(self) -> bool:
        """
        Delete the entire collection (useful for re-indexing).

        Returns:
            True if successful, False otherwise
        """
        try:
            self.client.delete_collection(collection_name=self.collection_name, timeout=self.timeout)
            logging.info(f"Deleted Qdrant collection '{self.collection_name}'")
            return True
        except Exception as e:
            logging.error(f"Failed to delete collection: {e}")
            return False

    def count_documents(self) -> int:
        """
        Count the total number of documents in the collection.

        Returns:
            Total number of documents in the collection
        """
        try:
            # Note: get_collection doesn't accept timeout parameter, but uses the client's global timeout
            collection_info = self.client.get_collection(
                collection_name=self.collection_name
            )
            return collection_info.points_count
        except Exception as e:
            logging.error(f"Failed to count documents in collection: {e}")
            return 0

    def health_check(self) -> Dict[str, Any]:
        """
        Perform a health check on the Qdrant instance.

        Returns:
            Health status information
        """
        try:
            # Use info() method which provides version and other information about the instance
            # Note: info() doesn't accept timeout parameter, but uses the client's global timeout
            info = self.client.info()
            return {
                "status": "healthy",
                "version": getattr(info, 'version', 'unknown'),
                "commit": getattr(info, 'commit', 'unknown')
            }
        except Exception as e:
            logging.error(f"Qdrant health check failed: {e}")
            return {
                "status": "unhealthy",
                "error": str(e)
            }