# Data Model: RAG Chatbot Backend

## Entity: Document
**Representation**: An uploaded document that will be processed for RAG functionality

**Fields**:
- `id`: UUID (primary key)
- `user_id`: UUID (foreign key to user)
- `collection_id`: UUID (foreign key to document collection)
- `filename`: String (original filename)
- `file_type`: String (PDF, DOCX, TXT, etc.)
- `file_size`: Integer (size in bytes)
- `upload_date`: DateTime (timestamp of upload)
- `processing_status`: Enum (pending, processing, completed, failed)
- `total_chunks`: Integer (number of text chunks created)
- `metadata`: JSON (extracted document metadata)

**Relationships**:
- Belongs to one User
- Belongs to one DocumentCollection
- Has many DocumentChunks

## Entity: DocumentCollection
**Representation**: A logical grouping of documents that belong to a user

**Fields**:
- `id`: UUID (primary key)
- `user_id`: UUID (foreign key to user)
- `name`: String (collection name)
- `description`: String (optional description)
- `created_date`: DateTime (timestamp of creation)
- `total_documents`: Integer (count of documents)
- `total_chunks`: Integer (count of all chunks in collection)

**Relationships**:
- Belongs to one User
- Has many Documents

## Entity: DocumentChunk
**Representation**: A processed segment of a document that has been embedded for vector search

**Fields**:
- `id`: UUID (primary key)
- `document_id`: UUID (foreign key to document)
- `chunk_index`: Integer (position of chunk in original document)
- `content`: Text (the actual text content of the chunk)
- `content_length`: Integer (character count)
- `embedding_id`: String (ID in vector database)
- `metadata`: JSON (additional metadata for filtering)

**Relationships**:
- Belongs to one Document
- Belongs to one DocumentCollection (through document)

## Entity: ChatSession
**Representation**: A conversation session between a user and the chatbot

**Fields**:
- `id`: UUID (primary key)
- `user_id`: UUID (foreign key to user)
- `collection_id`: UUID (foreign key to document collection used in session)
- `created_date`: DateTime (timestamp of session creation)
- `last_interaction`: DateTime (timestamp of last message)
- `title`: String (auto-generated session title)
- `active`: Boolean (whether session is active)

**Relationships**:
- Belongs to one User
- Belongs to one DocumentCollection
- Has many ChatMessages

## Entity: ChatMessage
**Representation**: A single message in a chat conversation

**Fields**:
- `id`: UUID (primary key)
- `session_id`: UUID (foreign key to chat session)
- `sender_type`: Enum (user, agent)
- `content`: Text (message content)
- `timestamp`: DateTime (when message was sent)
- `sources`: JSON (document sources referenced in response)
- `message_type`: Enum (query, response, system)

**Relationships**:
- Belongs to one ChatSession

## Entity: User
**Representation**: A system user with authentication and profile information

**Fields**:
- `id`: UUID (primary key)
- `email`: String (user email)
- `name`: String (display name)
- `created_date`: DateTime (account creation)
- `last_login`: DateTime (last login timestamp)
- `profile_data`: JSON (user preferences, background, etc.)

**Relationships**:
- Has many DocumentCollections
- Has many Documents
- Has many ChatSessions

## Entity: AgentToolCall
**Representation**: Record of a tool call made by the OpenAI Agent during response generation

**Fields**:
- `id`: UUID (primary key)
- `session_id`: UUID (foreign key to chat session)
- `message_id`: UUID (foreign key to chat message)
- `tool_name`: String (name of the tool called)
- `tool_input`: JSON (input parameters to the tool)
- `tool_output`: JSON (output from the tool)
- `timestamp`: DateTime (when tool was called)

**Relationships**:
- Belongs to one ChatSession
- Belongs to one ChatMessage