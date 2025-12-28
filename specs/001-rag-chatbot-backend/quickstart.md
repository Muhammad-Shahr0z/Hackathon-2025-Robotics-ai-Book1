# Quickstart Guide: RAG Chatbot Backend

## Prerequisites

- Python 3.11+
- pip package manager
- Access to OpenAI API (for OpenAI Agents SDK)
- Access to Cohere API (for embeddings)
- Qdrant vector database (cloud or self-hosted)
- PostgreSQL database (for metadata storage)

## Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/your-org/rag-chatbot-backend.git
   cd rag-chatbot-backend
   ```

2. **Create a virtual environment:**
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

4. **Set up environment variables:**
   Create a `.env` file with the following:
   ```env
   OPENAI_API_KEY=your_openai_api_key
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   DATABASE_URL=postgresql://user:password@host:port/database
   SECRET_KEY=your_secret_key_for_authentication
   ```

## Running the Application

1. **Start the development server:**
   ```bash
   uvicorn main:app --reload --host 0.0.0.0 --port 8000
   ```

2. **Access the API:**
   - API documentation: `http://localhost:8000/docs`
   - API base URL: `http://localhost:8000/api/v1`

## Basic Usage

### 1. Create a Document Collection
```bash
curl -X POST http://localhost:8000/api/v1/collections \
  -H "Authorization: Bearer your_jwt_token" \
  -H "Content-Type: application/json" \
  -d '{
    "name": "My Documents",
    "description": "Collection for my research documents"
  }'
```

### 2. Upload a Document
```bash
curl -X POST http://localhost:8000/api/v1/documents/upload \
  -H "Authorization: Bearer your_jwt_token" \
  -F "file=@path/to/document.pdf" \
  -F "collection_id=collection_uuid"
```

### 3. Start a Chat Session
```bash
curl -X POST http://localhost:8000/api/v1/chat/sessions \
  -H "Authorization: Bearer your_jwt_token" \
  -H "Content-Type: application/json" \
  -d '{
    "collection_id": "collection_uuid"
  }'
```

### 4. Send a Message
```bash
curl -X POST http://localhost:8000/api/v1/chat/sessions/session_uuid/messages \
  -H "Authorization: Bearer your_jwt_token" \
  -H "Content-Type: application/json" \
  -d '{
    "content": "What does the document say about AI safety?"
  }'
```

## Configuration

### Environment Variables
- `OPENAI_API_KEY`: API key for OpenAI services
- `COHERE_API_KEY`: API key for Cohere embedding service
- `QDRANT_URL`: URL for Qdrant vector database
- `QDRANT_API_KEY`: API key for Qdrant (if using cloud version)
- `DATABASE_URL`: Connection string for PostgreSQL database
- `SECRET_KEY`: Secret key for JWT token signing
- `MAX_FILE_SIZE`: Maximum file size for uploads (default: 10MB)
- `EMBEDDING_BATCH_SIZE`: Number of chunks to process in a single embedding call (default: 10)

### Application Settings
The application can be configured via `config.py`:
- `QDRANT_COLLECTION_NAME`: Name of the collection in Qdrant (default: "documents")
- `EMBEDDING_MODEL`: Cohere model to use for embeddings (default: "embed-english-v3.0")
- `MAX_CONTEXT_LENGTH`: Maximum number of tokens in context (default: 2000)

## Architecture Overview

The RAG Chatbot Backend follows a service-oriented architecture:

1. **API Layer**: FastAPI handles HTTP requests and authentication
2. **Document Service**: Manages document upload, parsing, and processing
3. **Embedding Service**: Creates embeddings using Cohere and stores in Qdrant
4. **Retrieval Service**: Performs semantic search in Qdrant
5. **Agent Service**: Uses OpenAI Agents SDK for response generation
6. **Storage Layer**: PostgreSQL for metadata, Qdrant for vector storage

## Development

### Running Tests
```bash
pytest tests/
```

### Running with Docker
```bash
docker-compose up --build
```

### API Documentation
The API is documented using Swagger/OpenAPI at `/docs` endpoint when running.