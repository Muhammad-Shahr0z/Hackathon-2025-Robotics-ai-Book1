import logging
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
import os
# Configure logging for serverless environment
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import Better Auth
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from better_auth import add_auth_routes, initialize_auth




# FastAPI app
app = FastAPI(
    title="Robotics Textbook Chatbot API",
    description="API for the Physical AI & Humanoid Robotics textbook chatbot",
    version="1.0.0"
)

# Add exception handlers
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

# Root endpoint
@app.get("/")
async def root():
    return {"message": "Robotics Textbook Chatbot API is running"}

# CORS middleware: allow all origins, headers, methods
# CORS middleware: allow only specific origins
origins = [
    "https://hackathon-2025-robotics-ai-book1-sr.vercel.app",
    "http://localhost:3000",
    "*"
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,  # Only these origins allowed
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"]
)


# Health check
@app.get("/health")
async def health_check():
    return {
        "status": "healthy",
        "timestamp": "2025-12-14T10:00:00Z"
    }

# Add Better Auth routes
add_auth_routes(app)
