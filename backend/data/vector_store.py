"""
Module for interacting with Qdrant vector database.
"""
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from typing import List, Dict, Any
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
    def __init__(self, collection_name: str = "Humanoids"):
        """
        Initialize the vector store with Qdrant client.

        Args:
            collection_name: Name of the Qdrant collection to use
        """
        try:
            qdrant_url = os.getenv("QDRANT_URL")
            qdrant_api_key = os.getenv("QDRANT_API_KEY")

            if not qdrant_url or not qdrant_api_key:
                raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set in environment variables")

            self.client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
                prefer_grpc=False
            )
            self.collection_name = collection_name

            # Create the collection if it doesn't exist
            self._create_collection_if_not_exists()
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
            self.client.get_collections()
            return True
        except Exception as e:
            logging.error(f"Qdrant connection test failed: {e}")
            return False

    def _create_collection_if_not_exists(self):
        """
        Create the Qdrant collection with 384-dimensional vectors and cosine similarity if it doesn't exist.
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections().collections
            collection_names = [c.name for c in collections]

            if self.collection_name not in collection_names:
                # Create collection with 384-dimensional vectors and cosine similarity
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(size=384, distance=Distance.COSINE)
                )
                logging.info(f"Created Qdrant collection '{self.collection_name}' with 384-dim vectors and cosine similarity")
            else:
                logging.info(f"Qdrant collection '{self.collection_name}' already exists")
        except Exception as e:
            logging.error(f"Failed to create or verify Qdrant collection: {e}")
            raise

    def store_document_chunk(self, chunk: DocumentChunk) -> bool:
        """
        Store a document chunk in the vector database.

        Args:
            chunk: DocumentChunk object containing content and embedding

        Returns:
            True if successful, False otherwise
        """
        try:
            # Prepare the point for Qdrant
            points = [
                models.PointStruct(
                    id=chunk.chunk_id,
                    vector=chunk.embedding,
                    payload={
                        "content": chunk.content,
                        "doc_path": chunk.doc_path,
                        "metadata": chunk.metadata
                    }
                )
            ]

            # Upload the point to Qdrant
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
        Store multiple document chunks in the vector database.

        Args:
            chunks: List of DocumentChunk objects to store

        Returns:
            True if successful, False otherwise
        """
        try:
            # Prepare the points for Qdrant
            points = []
            for chunk in chunks:
                points.append(
                    models.PointStruct(
                        id=chunk.chunk_id,
                        vector=chunk.embedding,
                        payload={
                            "content": chunk.content,
                            "doc_path": chunk.doc_path,
                            "metadata": chunk.metadata
                        }
                    )
                )

            # Upload the points to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

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
            # Perform the search in Qdrant
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit
            )

            # Format the results
            results = []
            for result in search_results:
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
            self.client.delete_collection(self.collection_name)
            logging.info(f"Deleted Qdrant collection '{self.collection_name}'")
            return True
        except Exception as e:
            logging.error(f"Failed to delete collection: {e}")
            return False