from dotenv import load_dotenv
import os
from openai import AsyncOpenAI
import asyncio

load_dotenv()

client = AsyncOpenAI(
    api_key=os.getenv("QWEN_API_KEY"),
    base_url="https://portal.qwen.ai/v1"
)

async def test():
    try:
        r = await client.chat.completions.create(
            model="qwen3-coder-plus",
            messages=[{"role": "user", "content": "ping"}]
        )
        print(r.choices[0].message.content)
    except Exception as e:
        print("ERROR:", e)

asyncio.run(test())
