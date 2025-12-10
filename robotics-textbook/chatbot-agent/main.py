from fastapi import FastAPI
from pydantic import BaseModel
from qdrant_client import QdrantClient, AsyncQdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import uuid, os
from dotenv import load_dotenv

from agents import AsyncOpenAI, OpenAIChatCompletionsModel, RunConfig, Agent, Runner, set_tracing_disabled

# Load environment variables
load_dotenv(override=True)
set_tracing_disabled(True)

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY_FREE")
QDRANT_URL = os.getenv("Qdrant_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_Key")

if not GEMINI_API_KEY:
    raise ValueError("GEMINI_API_KEY_FREE missing in .env")
if not QDRANT_URL or not QDRANT_API_KEY:
    raise ValueError("Qdrant credentials missing in .env")

COLLECTION_NAME = "my_docs"

# 1) Sync client for adding documents
sync_qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
try:
    sync_qdrant.get_collection(COLLECTION_NAME)
except Exception:
    sync_qdrant.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(size=768, distance=Distance.COSINE)
    )

# 2) Async client for querying
async_qdrant = AsyncQdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# FastAPI app
app = FastAPI()

class AddText(BaseModel):
    text: str

class UserInput(BaseModel):
    message: str

# Add text to Qdrant
@app.post("/add")
async def add_to_qdrant(data: AddText):
    ai_client = AsyncOpenAI(
        api_key=GEMINI_API_KEY,
        base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
    )

    # Create embedding
    embed = await ai_client.embeddings.create(
        model="text-embedding-004",
        input=data.text
    )
    vector = embed.data[0].embedding

    # Insert into Qdrant
    sync_qdrant.upsert(
        collection_name=COLLECTION_NAME,
        points=[PointStruct(
            id=str(uuid.uuid4()),
            vector=vector,
            payload={"text": data.text}
        )]
    )
    return {"msg": "Text added successfully"}

# Chat using Qdrant
@app.post("/chat")
async def chat_with_agent(data: UserInput):
    ai_client = AsyncOpenAI(
        api_key=GEMINI_API_KEY,
        base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
    )

    # 1) Generate embedding for user query
    embed = await ai_client.embeddings.create(
        model="text-embedding-004",
        input=data.message
    )
    query_vec = embed.data[0].embedding

    # 2) Query Qdrant asynchronously
    result = await async_qdrant.query_points(
        collection_name=COLLECTION_NAME,
        query=query_vec,
        limit=3,
        with_payload=True
    )

    hits = result.points  # get points list from QueryResponse
    context_texts = [p.payload.get("text", "") for p in hits]
    context = "\n".join(context_texts)

    # 3) Setup the chat model
    google_model = OpenAIChatCompletionsModel(
        openai_client=ai_client,
        model="gemini-2.5-flash",
    )
    config = RunConfig(model_provider=ai_client, model=google_model, tracing_disabled=True)

    agent = Agent(
        name="Helpful Assistant",
        instructions=(
            "Answer ONLY using the provided context from the Qdrant database. "
            "If the answer is not present or only partially matches, reply: "
            "'This information is not included in the book.'"
        )
    )

    # 4) Create final prompt
    final_prompt = f"CONTEXT:\n{context}\n\nUSER QUESTION:\n{data.message}\n\nAnswer ONLY from context."

    # 5) Run the agent
    result = await Runner.run(agent, input=final_prompt, run_config=config)

    return {"context": context, "response": result.final_output}
