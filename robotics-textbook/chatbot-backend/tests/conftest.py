"""Pytest configuration and shared fixtures for RAG chatbot tests."""

import os
import pytest
from unittest.mock import MagicMock, AsyncMock, patch
from typing import Generator
import asyncio # Added missing import

# Set environment variable as early as possible for module imports
if "OPENAI_API_KEY" not in os.environ:
    os.environ["OPENAI_API_KEY"] = "dummy_openai_api_key_for_testing"

# Now import modules that depend on settings
from httpx import AsyncClient
from src.main import app # Import app here to avoid ScopeMismatch in some cases


@pytest.fixture(scope="session") # Changed to session scope
def event_loop():
    """Session-scoped event loop for pytest-asyncio."""
    policy = asyncio.get_event_loop_policy()
    loop = policy.new_event_loop()
    yield loop
    loop.close()


@pytest.fixture(scope="module")
def anyio_backend():
    """Configure anyio backend for async tests to module scope."""
    return "asyncio"

@pytest.fixture(scope="module")
async def test_client(): # Removed anyio_backend from dependency, as event_loop is now session-scoped
    """Fixture for an asynchronous test client, scoped to module."""
    async with AsyncClient(app=app, base_url="http://test") as client:
        yield client


@pytest.fixture
def mock_qdrant_client():
    """Mock Qdrant client for vector search tests."""
    client = MagicMock()

    # Mock search response
    client.search.return_value = [
        MagicMock(
            id=1,
            score=0.95,
            payload={
                "chapter": "Module 1: ROS 2 Basics",
                "section": "Nodes and Topics",
                "content": "ROS 2 nodes communicate via topics using a publisher-subscriber pattern."
            }
        ),
        MagicMock(
            id=2,
            score=0.87,
            payload={
                "chapter": "Module 1: ROS 2 Basics",
                "section": "Services",
                "content": "Services provide request-reply communication between nodes."
            }
        )
    ]

    return client


@pytest.fixture
def mock_gemini_client():
    """Mock Gemini LLM client for response generation tests."""
    client = AsyncMock()

    # Mock generation response
    client.generate_content.return_value = MagicMock(
        text="ROS 2 nodes communicate through a publish-subscribe mechanism called topics. "
             "Publishers send messages to a topic, and subscribers receive those messages. "
             "This decouples nodes from each other, allowing flexible distributed communication."
    )

    return client


@pytest.fixture
def mock_neon_connection():
    """Mock Neon PostgreSQL connection pool."""
    pool = AsyncMock()

    # Mock acquire connection
    mock_conn = AsyncMock()
    pool.acquire.return_value.__aenter__.return_value = mock_conn

    return pool


@pytest.fixture
def mock_session():
    """Mock user session data."""
    return {
        "session_id": "test-session-123",
        "anonymous_browser_id": "browser-456",
        "created_at": "2025-11-30T10:00:00Z",
        "conversation_ids": ["conv-1", "conv-2"],
        "expires_at": "2025-12-30T10:00:00Z"
    }


@pytest.fixture
def mock_message():
    """Mock chat message."""
    return {
        "id": "msg-1",
        "conversation_id": "conv-1",
        "role": "user",
        "content": "How do ROS 2 nodes communicate?",
        "timestamp": "2025-11-30T10:05:00Z",
        "source_references": []
    }


@pytest.fixture
def mock_response():
    """Mock chatbot response."""
    return {
        "answer": "ROS 2 nodes communicate via topics using a publisher-subscriber pattern.",
        "sources": [
            {
                "chapter": "Module 1: ROS 2 Basics",
                "section": "Nodes and Topics",
                "content_excerpt": "ROS 2 nodes communicate via topics...",
                "link": "/docs/module-1/chapter-1#nodes-and-topics"
            }
        ],
        "session_id": "test-session-123",
        "message_id": "msg-1"
    }


@pytest.fixture
def mock_embedding():
    """Mock text embedding."""
    return [0.1, 0.2, 0.3] * 100  # 300-dimensional embedding