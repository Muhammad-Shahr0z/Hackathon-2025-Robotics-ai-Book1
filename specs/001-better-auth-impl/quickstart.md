# Quickstart: Better Auth Implementation

## Prerequisites

- Python 3.8+
- Neon database with MCP server access
- Environment variables configured (.env file)

## Setup

1. Install required dependencies:
```bash
pip install better-auth-python
```

2. Configure environment variables in `.env`:
```
BETTER_AUTH_SECRET=your_secret_key
BETTER_AUTH_URL=your_auth_url
DATABASE_URL=your_neon_db_connection_string
```

3. Create the `better_auth.py` file in the `chatbot-agent` directory

## Basic Usage

### 1. User Registration
```python
# Register a new user with first name, last name, email, and password
result = await better_auth.register(first_name, last_name, email, password)
```

### 2. User Authentication
```python
# Authenticate user with email and password
result = await better_auth.login(email, password)
```

### 3. Password Reset
```python
# Initiate password reset
await better_auth.forgot_password(email)

# Complete password reset with token
await better_auth.reset_password(token, new_password)
```

## MCP Server Integration

The authentication system connects to Neon database through MCP servers as specified in the requirements. All database operations use the MCP server connection for security and compliance.

## Environment Configuration

The system loads all configuration from the `.env` file to ensure secure credential management. The required environment variables are:

- `BETTER_AUTH_SECRET`: Secret key for authentication
- `BETTER_AUTH_URL`: Authentication service URL
- `DATABASE_URL`: Neon database connection string