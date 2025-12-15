# Quickstart: Deploy FastAPI Backend to Vercel

## Prerequisites
- Python 3.11
- FastAPI application ready for serverless deployment
- Vercel account with MCP server connected
- Git repository initialized

## Setup Steps

### 1. Prepare Vercel Configuration
Create a `vercel.json` file in your project root:
```json
{
  "version": 2,
  "builds": [
    {
      "src": "main.py",
      "use": "@vercel/python",
      "config": { "runtime": "python3.11" }
    }
  ],
  "routes": [
    {
      "src": "/(.*)",
      "dest": "main.py"
    }
  ]
}
```

### 2. Update requirements.txt
Ensure your requirements.txt includes FastAPI and uvicorn:
```
fastapi==0.104.1
uvicorn==0.24.0
```

### 3. Modify FastAPI Application (if needed)
Ensure your main.py exports the FastAPI app instance:
```python
from fastapi import FastAPI

app = FastAPI()

# Your routes here

# This ensures Vercel can import the app
# The Vercel Python runtime will look for this
```

### 4. Deploy Using Vercel MCP
Use the Vercel MCP server to deploy your application:
1. Connect to the Vercel MCP server
2. Initiate the deployment process
3. Monitor the deployment status
4. Validate the deployment once complete

## Verification
After deployment, verify that:
- The deployment URL is accessible
- API endpoints return expected responses
- Environment variables are properly configured
- The application responds within 5 seconds

## Troubleshooting
- If deployment fails, check that all dependencies are compatible with Vercel's Python runtime
- Ensure the entry point is correctly configured in vercel.json
- Verify that environment variables are set in the Vercel dashboard