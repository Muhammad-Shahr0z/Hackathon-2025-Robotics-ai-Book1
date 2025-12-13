import logging
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from qdrant_client import QdrantClient, AsyncQdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import uuid, os
from dotenv import load_dotenv

from agents import AsyncOpenAI, OpenAIChatCompletionsModel, RunConfig, Agent, Runner, set_tracing_disabled

# Configure logging for serverless environment
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import Better Auth
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from better_auth import add_auth_routes, initialize_auth

# Load environment variables
load_dotenv(override=True)
set_tracing_disabled(True)

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY_FREE")
QDRANT_URL = os.getenv("Qdrant_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_Key")

# Only require GEMINI_API_KEY if we're using the AI features
if not QDRANT_URL or not QDRANT_API_KEY:
    print("Warning: Qdrant credentials missing in .env - AI features will be disabled")
    # Disable AI routes if credentials are missing
    GEMINI_API_KEY = None

COLLECTION_NAME = "my_docs"

# 1) Sync client for adding documents (only if Qdrant credentials are available)
if QDRANT_URL and QDRANT_API_KEY:
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
else:
    sync_qdrant = None
    async_qdrant = None

# FastAPI app
app = FastAPI(
    title="Robotics Textbook Chatbot API",
    description="API for the Physical AI & Humanoid Robotics textbook chatbot",
    version="1.0.0"
)

# Add exception handlers for serverless environment
@app.exception_handler(RequestValidationError)
async def validation_exception_handler(request, exc):
    logger.error(f"Validation error: {exc}")
    return JSONResponse(
        status_code=422,
        content={"detail": exc.errors()}
    )

@app.exception_handler(Exception)
async def general_exception_handler(request, exc):
    logger.error(f"General error: {exc}")
    return JSONResponse(
        status_code=500,
        content={"detail": "Internal server error"}
    )

# Add a simple root endpoint for basic health check
@app.get("/")
async def root():
    return {"message": "Robotics Textbook Chatbot API is running"}

# Add CORS middleware to allow requests from the frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:3000/Hackathon-2025-Robotics-ai-Book1/", "http://localhost:3001"],  # Frontend origin
    allow_credentials=True,
    allow_methods=["*"],  # Allow all methods
    allow_headers=["*"],  # Allow all headers
    # expose_headers=["Access-Control-Allow-Origin"]
)

class AddText(BaseModel):
    text: str

class UserInput(BaseModel):
    message: str

# Add text to Qdrant (only if AI credentials are available)
if GEMINI_API_KEY and sync_qdrant:
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
else:
    print("AI features disabled due to missing credentials")
    # Add placeholder endpoints for AI routes if needed
    @app.post("/add")
    async def add_disabled():
        return {"msg": "AI features disabled due to missing credentials"}

    @app.post("/chat")
    async def chat_disabled():
        return {"context": "", "response": "AI features disabled due to missing credentials"}


# Health check endpoint
@app.get("/health")
async def health_check():
    return {
        "status": "healthy",
        "timestamp": "2025-12-14T10:00:00Z"
    }

# Add Better Auth routes to the app
add_auth_routes(app)

# For serverless environments, we'll initialize auth on first request if needed
# Don't initialize auth at startup since Vercel functions are stateless
# The auth initialization will happen per request as needed
