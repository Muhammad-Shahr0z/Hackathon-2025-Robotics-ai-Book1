# API Contract: Chatbot Service

## Overview
This contract defines the interface for the chatbot backend service that the ChatbotUI component will communicate with.

## Endpoints

### POST /api/chat
Sends a user message to the chatbot and receives a response.

#### Request
```json
{
  "prompt": "string (the user's message)"
}
```

#### Response
```json
{
  "response": "string (the chatbot's response)",
  "timestamp": "ISO 8601 datetime string"
}
```

#### Example Request
```
POST /api/chat
Content-Type: application/json

{
  "prompt": "Hello, how are you?"
}
```

#### Example Response
```
HTTP/1.1 200 OK
Content-Type: application/json

{
  "response": "I'm doing well, thank you for asking!",
  "timestamp": "2025-12-15T10:30:00.000Z"
}
```

#### Error Responses
- `400 Bad Request`: Invalid request format
- `500 Internal Server Error`: Server processing error

## Future Extensions
- Authentication headers for user identification
- Session management for conversation history
- Streaming responses for real-time typing simulation