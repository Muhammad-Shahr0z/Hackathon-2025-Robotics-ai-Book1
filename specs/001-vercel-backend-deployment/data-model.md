# Data Model: Backend Deployment on Vercel

## Entities

### FastAPI Application
- **Name**: The application identifier for Vercel deployment
- **Description**: The existing Python FastAPI backend that needs to be deployed
- **Configuration**: Settings required for Vercel serverless compatibility
- **State**: Deployment status (pending, deployed, failed)

### Vercel Deployment
- **DeploymentId**: Unique identifier for the Vercel deployment
- **Url**: Public URL of the deployed backend
- **Status**: Current status of the deployment (building, ready, error)
- **CreatedAt**: Timestamp of deployment creation
- **Environment**: Target environment (development, preview, production)

### Serverless Function Configuration
- **Runtime**: Python runtime version for Vercel (e.g., python3.11)
- **Entrypoint**: Path to the FastAPI application entry point
- **BuildCommand**: Command to run during build phase
- **OutputDirectory**: Directory containing the built application
- **EnvironmentVariables**: Configuration for environment variables

## Relationships
- One FastAPI Application maps to one Vercel Deployment
- One Vercel Deployment uses one Serverless Function Configuration

## Validation Rules
- FastAPI Application must be compatible with Vercel's Python runtime
- Vercel Deployment URL must be accessible after successful deployment
- Serverless Function Configuration must follow Vercel's specification

## State Transitions
- FastAPI Application: [development] → [configured-for-vercel] → [deployed]
- Vercel Deployment: [not-started] → [building] → [ready/failed]