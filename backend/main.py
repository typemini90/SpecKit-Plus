import logging
import json
from fastapi import FastAPI, Request, Response
from fastapi.responses import StreamingResponse
from fastapi.middleware.cors import CORSMiddleware

from agents import Agent, Runner
from agents.models.openai_chatcompletions import OpenAIChatCompletionsModel
from agents.run import RunConfig
from openai import AsyncOpenAI

logging.basicConfig(level=logging.INFO)

# QWEN CLIENT
external_client = AsyncOpenAI(
    api_key="6illN1cvAe5bRWVi3jIfMVwrzYNG3mciaS7-r5GGLQB5bvUxYoAaUAE80acFUeIsh-JIciuCwIrmc74dr1B7bw",
    base_url="https://portal.qwen.ai/v1"
)

model = OpenAIChatCompletionsModel(
    model="qwen3-coder-plus",
    openai_client=external_client
)

agent = Agent(
    name="SocialMediaPoster",
    instructions="hi",
    model=model
)

config = RunConfig()

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/test-qwen")
async def test_qwen():
    try:
        r = await client.chat.completions.create(
            model="qwen3-coder-plus",
            messages=[{"role": "user", "content": "ping"}]
        )
        return {"reply": r.choices[0].message.content}
    except Exception as e:
        return {"error": str(e)}

import logging
import json
from fastapi import FastAPI, Request, Response
from fastapi.responses import StreamingResponse
from fastapi.middleware.cors import CORSMiddleware

from agents import Agent, Runner
from agents.models.openai_chatcompletions import OpenAIChatCompletionsModel
from agents.run import RunConfig
from openai import AsyncOpenAI

logging.basicConfig(level=logging.INFO)

# QWEN CLIENT
external_client = AsyncOpenAI(
    api_key="6illN1cvAe5bRWVi3jIfMVwrzYNG3mciaS7-r5GGLQB5bvUxYoAaUAE80acFUeIsh-JIciuCwIrmc74dr1B7bw",
    base_url="https://portal.qwen.ai/v1"
)

model = OpenAIChatCompletionsModel(
    model="qwen3-coder-plus",
    openai_client=external_client
)

agent = Agent(
    name="SocialMediaPoster",
    instructions="hi",
    model=model
)

config = RunConfig()

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# TEST ENDPOINT (fixed client variable)
@app.get("/test-qwen")
async def test_qwen():
    try:
        r = await external_client.chat.completions.create(
            model="qwen3-coder-plus",
            messages=[{"role": "user", "content": "ping"}]
        )
        return {"reply": r.choices[0].message.content}
    except Exception as e:
        return {"error": str(e)}


@app.post("/api/query")
async def chatkit_endpoint(request: Request):
    try:
        raw = await request.body()
        text = raw.decode().strip()
        logging.info(f"Received raw: {text}")

        # Accept both JSON body and plain text
        try:
            data = json.loads(text)
            user_input = data.get("message") or data.get("input") or text
        except:
            user_input = text

        result = await Runner.run(
            starting_agent=agent,
            input=user_input,
            run_config=config
        )

        # SSE streaming
        async def event_generator():
            # If the agent streams tokens/steps
            if hasattr(result, "__aiter__"):
                async for step in result:
                    yield f"data: {step.dict()}\n\n"
            else:
                yield f"data: {result.dict()}\n\n"

        return StreamingResponse(event_generator(), media_type="text/event-stream")

    except Exception as e:
        logging.exception("Error in /api/query")
        return Response(
            content=json.dumps({"error": str(e)}),
            media_type="application/json"
        )


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="127.0.0.1", port=8000, reload=True)
