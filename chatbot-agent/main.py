from fastapi import FastAPI
import os
import sys

# allow local imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from better_auth import add_auth_routes

app = FastAPI(
    title="Better Auth API",
    version="1.0.0"
)

@app.get("/health")
async def health():
    return {
        "status": "ok",
        "service": "better-auth"
    }

# register better auth routes
add_auth_routes(app)





if __name__ == "__main__":
    import uvicorn
    "main:app", host="0.0.0.0", port=5001, reload=True