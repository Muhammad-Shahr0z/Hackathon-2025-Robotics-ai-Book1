from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from better_auth import add_auth_routes

app = FastAPI()

# Explicit frontend origins (replace with your deployed frontend URL)
origins = [
    "https://muhammad-shahr0z.github.io",
    "http://localhost:3000"
  
]

# Proper CORS setup
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,       # must be explicit when allow_credentials=True
    allow_credentials=True,      # allow cookies / auth headers
    allow_methods=["*"],
    allow_headers=["*"]
)

# Health check
@app.get("/health")
async def health_check():
    return {"status": "healthy"}

# Add Better Auth routes
add_auth_routes(app)
