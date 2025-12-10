"""
Unified ChatKit Server with direct OpenAI integration and RAG capabilities.
Combines features from simple_chatkit_api and chatkit_api into a single implementation.
"""
from typing import List, Dict, Any, Optional
from pydantic import BaseModel
from fastapi import FastAPI
import os
from dotenv import load_dotenv
from openai import OpenAI
import asyncio
import uuid
from chatkit.server import ChatKitServer
from chatkit.store import Store
from chatkit.types import ThreadMetadata, ThreadItem, Page, UserMessageItem, AssistantMessageItem
from chatkit.types import AssistantMessageContent
from qdrant_client import QdrantClient
from qdrant_client.http import models
import logging
from loguru import logger

load_dotenv()

# Initialize Qwen client with Qwen API
qwen_api_key = os.getenv("QWEN_API_KEY")
if not qwen_api_key:
    raise ValueError("QWEN_API_KEY must be set in environment variables")

client = AsyncOpenAI(
    api_key=qwen_api_key,
    base_url="https://portal.qwen.ai/v1"
)

model = "qwen3-coder-plus"

# Initialize Qdrant client for RAG functionality
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

if not qdrant_url or not qdrant_api_key:
    raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set in environment variables")

qdrant_client = QdrantClient(
    url=qdrant_url,
    api_key=qdrant_api_key,
    prefer_grpc=False
)

# Initialize Qdrant collection if it doesn't exist
collection_name = "book_content"
collections = qdrant_client.get_collections().collections
collection_names = [c.name for c in collections]

if collection_name not in collection_names:
    qdrant_client.create_collection(
        collection_name=collection_name,
        vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE)
    )
    logger.info(f"Created Qdrant collection '{collection_name}' with 384-dim vectors")
else:
    logger.info(f"Qdrant collection '{collection_name}' already exists")

# In-memory store for MVP
memory_store: Dict[str, Any] = {
    "threads": {},
    "items": {}
}

class UnifiedStore(Store[dict]):
    """Unified in-memory store with RAG capabilities."""

    def generate_thread_id(self, context: dict) -> str:
        return f"thread_{uuid.uuid4().hex[:12]}"

    def generate_item_id(self, item_type: str, thread: ThreadMetadata, context: dict) -> str:
        return f"{item_type}_{uuid.uuid4().hex[:12]}"

    async def load_thread(self, thread_id: str, context: dict) -> ThreadMetadata:
        if thread_id in memory_store["threads"]:
            return memory_store["threads"][thread_id]
        else:
            # Create new thread
            thread = ThreadMetadata(
                id=thread_id,
                created_at=0,
                updated_at=0,
                version=0,
                metadata=None
            )
            memory_store["threads"][thread_id] = thread
            return thread

    async def save_thread(self, thread: ThreadMetadata, context: dict) -> None:
        memory_store["threads"][thread.id] = thread

    async def load_thread_items(self, thread_id: str, after: str | None,
                               limit: int, order: str, context: dict) -> Page[ThreadItem]:
        items = memory_store["items"].get(thread_id, [])
        return Page(data=items, has_more=False, next_cursor=None)

    async def add_thread_item(self, thread_id: str, item: ThreadItem, context: dict) -> None:
        if thread_id not in memory_store["items"]:
            memory_store["items"][thread_id] = []
        memory_store["items"][thread_id].append(item)

    async def save_item(self, thread_id: str, item: ThreadItem, context: dict) -> None:
        await self.add_thread_item(thread_id, item, context)

    async def load_item(self, thread_id: str, item_id: str, context: dict) -> ThreadItem:
        items = memory_store["items"].get(thread_id, [])
        for item in items:
            if item.id == item_id:
                return item
        raise ValueError(f"Item {item_id} not found in thread {thread_id}")

    async def delete_thread_item(self, thread_id: str, item_id: str, context: dict) -> None:
        if thread_id in memory_store["items"]:
            memory_store["items"][thread_id] = [
                item for item in memory_store["items"][thread_id]
                if item.id != item_id
            ]

    async def load_threads(self, limit: int, after: str | None,
                          order: str, context: dict) -> Page[ThreadMetadata]:
        threads = list(memory_store["threads"].values())
        return Page(data=threads[:limit], has_more=False, next_cursor=None)

    async def delete_thread(self, thread_id: str, context: dict) -> None:
        if thread_id in memory_store["threads"]:
            del memory_store["threads"][thread_id]
        if thread_id in memory_store["items"]:
            del memory_store["items"][thread_id]

    async def save_attachment(self, attachment: Any, context: dict) -> None:
        pass

    async def load_attachment(self, attachment_id: str, context: dict) -> Any:
        raise ValueError(f"Attachment {attachment_id} not found")

    async def delete_attachment(self, attachment_id: str, context: dict) -> None:
        pass

# Create store instance
store = UnifiedStore()

# Function to search documents in Qdrant
def search_documents(query: str, limit: int = 5) -> List[Dict[str, Any]]:
    """Search documents in Qdrant based on the query."""
    try:
        # Import embedding service to generate query embedding
        from embeddings import EmbeddingService
        embedding_service = EmbeddingService()
        query_embedding = embedding_service.embed_text(query)
        
        search_results = qdrant_client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=limit
        )

        results = []
        for result in search_results:
            results.append({
                "content": result.payload.get("content", ""),
                "doc_path": result.payload.get("doc_path", ""),
                "score": result.score
            })

        return results
    except Exception as e:
        logger.error(f"Error searching documents: {e}")
        return []

# Create unified ChatKit server with RAG capabilities
class UnifiedChatKitServer(ChatKitServer):
    def __init__(self, store):
        super().__init__(store=store)

    async def respond(self, thread: ThreadMetadata, input: Any, context: dict):
        """Response handler with conversation history and RAG capabilities."""
        # Extract text from input
        input_text = ""
        if hasattr(input, 'content') and input.content:
            for part in input.content:
                if hasattr(part, 'text'):
                    input_text = part.text
        elif isinstance(input, dict) and 'content' in input:
            input_text = input['content']
        else:
            input_text = str(input)

        # Create conversation history from thread items (simplified for MVP)
        try:
            page = await self.store.load_thread_items(thread.id, None, 100, "asc", context)
            messages = [
                {
                    "role": "system",
                    "content": "You are a helpful assistant for the Physical AI & Humanoid Robotics textbook. Answer questions about robotics, AI, and embodied intelligence."
                }
            ]

            for item in page.data:
                if isinstance(item, UserMessageItem):
                    text = ""
                    if hasattr(item, 'content') and item.content:
                        for part in item.content:
                            if hasattr(part, 'text'):
                                text += part.text
                    if text:
                        messages.append({"role": "user", "content": text})
                elif isinstance(item, AssistantMessageItem):
                    text = ""
                    if hasattr(item, 'content') and item.content:
                        for part in item.content:
                            if hasattr(part, 'text'):
                                text += part.text
                    if text:
                        messages.append({"role": "assistant", "content": text})

            # Add the current input
            messages.append({"role": "user", "content": input_text})
        except:
            # If there's an issue getting history, just use the current message
            messages = [
                {
                    "role": "system",
                    "content": "You are a helpful assistant for the Physical AI & Humanoid Robotics textbook. Answer questions about robotics, AI, and embodied intelligence."
                },
                {
                    "role": "user",
                    "content": input_text
                }
            ]

        # Search for relevant documents using RAG
        docs = search_documents(input_text)
        context_str = ""
        if docs:
            context_str = "\n\n".join([doc["content"] for doc in docs])

        # If we have relevant context, enhance the prompt with it
        if context_str:
            # Build prompt with RAG context
            prompt = f"""
            Context information is below:
            {context_str}

            Using the provided context information, answer the question: {input_text}

            If the context does not contain sufficient information to answer the question, respond with "I don't know".
            """
            
            # Call Qwen with RAG context
            response = client.chat.completions.create(
                model=model,
                messages=[
                    {
                        "role": "system",
                        "content": "You are a helpful assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based on the provided context. If the context is insufficient, respond with 'I don't know'."
                    },
                    {"role": "user", "content": prompt}
                ],
                max_tokens=500,
                temperature=0.1
            )

            answer = response.choices[0].message.content.strip()
        else:
            # Call Qwen without RAG context
            response = client.chat.completions.create(
                model=model,
                messages=messages,
                max_tokens=500,
                temperature=0.7
            )

            answer = response.choices[0].message.content.strip()

        # Check if the answer is "I don't know"
        if "i don't know" in answer.lower():
            answer = "I don't know based on the available information. Try asking about specific topics in the Physical AI & Humanoid Robotics textbook."

        # Create assistant message
        assistant_message = AssistantMessageItem(
            id=f"assistant_{uuid.uuid4().hex}",  # Using full UUID to ensure uniqueness
            thread_id=thread.id,  # Add the required thread_id
            created_at=int(asyncio.get_event_loop().time()) if hasattr(asyncio.get_event_loop(), 'time') else 0,
            content=[AssistantMessageContent(text=answer)]
        )

        # Add the message to the store
        await self.store.add_thread_item(thread.id, assistant_message, context)

        # Return the assistant message
        return assistant_message

server = UnifiedChatKitServer(store=store)

# Create FastAPI app and mount ChatKit server
app = FastAPI(
    title="Unified ChatKit Server with RAG",
    description="ChatKit-based chatbot server with direct OpenAI integration and RAG capabilities",
    version="1.0.0"
)

# Mount the ChatKit server at /chatkit
app.mount("/chatkit", server)

@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)