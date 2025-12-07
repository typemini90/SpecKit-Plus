"""
Simple MVP ChatKit Server with direct Gemini integration
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

load_dotenv()

# Initialize OpenAI client with Gemini
gemini_api_key = os.getenv("GEMINI_API_KEY")
if not gemini_api_key:
    raise ValueError("GEMINI_API_KEY must be set in environment variables")

openai_client = OpenAI(
    api_key=gemini_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

model = "gemini-2.0-flash"

# In-memory store for MVP
memory_store: Dict[str, Any] = {
    "threads": {},
    "items": {}
}

class SimpleStore(Store[dict]):
    """Simple in-memory store for MVP"""

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

# Create ChatKit server
store = SimpleStore()

# Create ChatKit server with proper respond method implementation
class SimpleChatKitServer(ChatKitServer):
    def __init__(self, store):
        super().__init__(store=store)

    async def respond(self, thread: ThreadMetadata, input: Any, context: dict):
        """Simple response handler using Gemini API directly"""
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
                    "content": "You are a helpful assistant for the Embodied Intelligence textbook. Answer questions about robotics, AI, and embodied intelligence."
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
                    "content": "You are a helpful assistant for the Embodied Intelligence textbook. Answer questions about robotics, AI, and embodied intelligence."
                },
                {
                    "role": "user",
                    "content": input_text
                }
            ]

        # Call Gemini API
        response = openai_client.chat.completions.create(
            model=model,
            messages=messages,
            max_tokens=500,
            temperature=0.7
        )

        answer = response.choices[0].message.content.strip()

        # Create and return assistant message
        assistant_message = AssistantMessageItem(
            id=f"msg_{uuid.uuid4().hex[:12]}",
            thread_id=thread.id,  # Add the required thread_id
            created_at=0,
            content=[AssistantMessageContent(text=answer)]
        )

        # Add the message to the store
        await self.store.add_thread_item(thread.id, assistant_message, context)

        # Return the assistant message
        return assistant_message

server = SimpleChatKitServer(store=store)

# Create FastAPI app and mount ChatKit server
app = FastAPI(
    title="Simple ChatKit MVP Server",
    description="Minimal ChatKit-based chatbot server with direct Gemini integration",
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