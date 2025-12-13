# Deployment Guide: FastAPI Backend to Vercel

## Overview
This document provides instructions for deploying the existing Python FastAPI backend to Vercel as serverless functions using the Vercel MCP server.

## Prerequisites
- Node.js and npm installed
- Vercel CLI installed: `npm install -g vercel`
- Git repository initialized
- Valid Vercel account with MCP server access

## Deployment Process

### 1. Prepare Your Environment
```bash
# Install Vercel CLI if not already installed
npm install -g vercel

# Login to Vercel
vercel login
```

### 2. Set Up Environment Variables
Before deployment, configure your environment variables in the Vercel dashboard or using the CLI:
```bash
vercel env add GEMINI_API_KEY_FREE
vercel env add Qdrant_URL
vercel env add QDRANT_API_Key
```

### 3. Deploy Using the Deployment Script
```bash
python deploy_to_vercel.py
```

### 4. Alternative Manual Deployment
```bash
# Navigate to the project root
cd /path/to/project

# Deploy using Vercel CLI with our configuration
vercel --local-config=vercel.json --public --yes
```

## Post-Deployment Validation

After deployment, validate that your application is working correctly:

1. Visit the deployment URL provided by Vercel
2. Test the health endpoint: `GET /health`
3. Verify other API endpoints are accessible

You can also use the validation script:
```bash
DEPLOYMENT_URL="https://your-deployment-url.vercel.app" python validate_deployment.py
```

## Configuration Files

### vercel.json
This file configures how Vercel builds and deploys your application:
- Sets Python 3.11 runtime
- Configures the entry point at `chatbot-agent/main.py`
- Routes all requests to the FastAPI application

### Requirements
Dependencies are specified in `chatbot-agent/requirements.txt` and will be installed automatically during deployment.

## Troubleshooting

### Common Issues
- **Cold Start**: First requests after inactivity may take longer due to serverless nature
- **Timeouts**: Ensure your functions complete within Vercel's timeout limits
- **Environment Variables**: Verify all required environment variables are set in the Vercel dashboard

### Health Check
Use the `/health` endpoint to verify application status after deployment.

## Rollback Plan

If issues occur after deployment:
1. Identify the problematic deployment in your Vercel dashboard
2. Use `vercel alias` to point your domain to a previous working deployment
3. Investigate and fix the issue in your code
4. Redeploy with the fixes

## Performance Considerations

- The application is configured to respond within 5 seconds as required
- Serverless functions automatically scale based on demand
- Consider implementing caching for frequently accessed data to reduce cold start impact