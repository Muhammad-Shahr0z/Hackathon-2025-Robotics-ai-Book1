from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from agents import Agent, Runner, OpenAIChatCompletionsModel, AsyncOpenAI
from agents import set_tracing_disabled, function_tool
import os
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from agents import enable_verbose_stdout_logging

# -------------------------------
# Load config & clients
# -------------------------------
enable_verbose_stdout_logging()
load_dotenv()
set_tracing_disabled(disabled=True)

gemini_api_key = os.getenv("GEMINI_API_KEY")
cohere_client_key = os.getenv("COHERE")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
qdrant_url = os.getenv("QDRANT_URL")

provider = AsyncOpenAI(
    api_key=gemini_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",
    openai_client=provider
)

# Cohere client
cohere_client = cohere.Client(cohere_client_key)
# Qdrant client
qdrant = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

# -------------------------------
# Helper function for embedding & retrieval
# -------------------------------
def get_embedding(text):
    response = cohere_client.embed(
        model="embed-english-v3.0",
        input_type="search_query",
        texts=[text],
    )
    return response.embeddings[0]

@function_tool
def retrieve(query):
    embedding = get_embedding(query)
    result = qdrant.query_points(
        collection_name="AiBook_COllection-01",
        query=embedding,
        limit=5
    )
    return [point.payload["text"] for point in result.points]

# -------------------------------
# Create the agent
# -------------------------------
agent = Agent(
    name="Assistant",
    instructions="""
You are an AI tutor for the Physical AI & Humanoid Robotics textbook.
To answer the user question, first call the tool `retrieve` with the user query.
Use ONLY the content returned from `retrieve` to answer the question.
If the answer is not present in the retrieved content, respond: 
"I don't know. This content, word, or topic is not in the book. I can only answer questions based on the book content."
""",
    model=model,
    tools=[retrieve]
)

# -------------------------------
# FastAPI app
# -------------------------------
app = FastAPI()

class QueryRequest(BaseModel):
    question: str

@app.post("/ask")
async def ask_ai(request: QueryRequest):
    user_input = request.question
    try:
        result = await Runner.run(agent, input=user_input)
        return {"answer": result.final_output}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
