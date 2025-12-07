"""Unit tests for chat query endpoint."""

import pytest
from unittest.mock import AsyncMock, patch
from uuid import uuid4

from src.main import app
from src.models.schemas import QueryRequest, QueryResponse
from src.config import settings
from src.services.rag_service import RAGService, get_rag_service
from src.services.cache_service import CacheService, get_cache_service
from src.utils.errors import RateLimitException
from fastapi import HTTPException # Added HTTPException import


@pytest.mark.asyncio
class TestQueryEndpoint:
    """Tests for the /api/v1/chat/query endpoint."""

    @pytest.fixture(autouse=True)
    def setup_and_teardown_dependency_overrides(self):
        """
        Fixture to clear FastAPI dependency overrides after each test,
        ensuring a clean state.
        """
        yield
        app.dependency_overrides = {}


    @pytest.fixture
    def mock_rag_service(self):
        """Mock RAGService dependency."""
        service = AsyncMock(spec=RAGService)
        service.process_query.return_value = {
            "answer": "Robotics is the study and design of robots.",
            "sources": [
                {
                    "id": "1",
                    "payload": {
                        "chapter": "Chapter 1",
                        "section": "Introduction",
                        "content": "Robotics is the study of robots...",
                    },
                    "score": 0.9,
                }
            ],
            "confidence": 0.9,
        }
        return service


    @pytest.fixture
    def mock_cache_service(self):
        """Mock CacheService dependency."""
        service = AsyncMock(spec=CacheService)
        service.get_query_cache.return_value = None
        service.set_query_cache.return_value = True
        service.close.return_value = None
        return service


    async def test_query_endpoint_success(self, test_client, mock_rag_service, mock_cache_service):
        """Test successful query endpoint response."""
        app.dependency_overrides[get_rag_service] = lambda: mock_rag_service
        app.dependency_overrides[get_cache_service] = lambda: mock_cache_service

        session_id = uuid4()
        question = "What is robotics?"
        page_context = "Chapter 1"

        response = await test_client.post(
            "/api/v1/chat/query",
            json={
                "question": question,
                "session_id": str(session_id),
                "page_context": page_context,
            },
        )

        assert response.status_code == 200
        data = response.json()
        assert data["answer"] == "Robotics is the study and design of robots."
        assert len(data["sources"]) > 0
        assert data["session_id"] == str(session_id)
        assert "message_id" in data
        assert 0 <= data["confidence"] <= 1

        mock_rag_service.process_query.assert_called_once_with(
            question=question,
            session_id=str(session_id),
            page_context=page_context
        )
        mock_cache_service.close.assert_called_once()


    async def test_query_endpoint_invalid_session(self, test_client, mock_rag_service, mock_cache_service):
        """Test query endpoint with invalid session (simulated by RAG service raising error)."""
        app.dependency_overrides[get_rag_service] = lambda: mock_rag_service
        app.dependency_overrides[get_cache_service] = lambda: mock_cache_service

        mock_rag_service.process_query.side_effect = HTTPException(status_code=401, detail="Invalid session")

        response = await test_client.post(
            "/api/v1/chat/query",
            json={
                "question": "What is robotics?",
                "session_id": str(uuid4()),
            },
        )

        assert response.status_code == 401
        assert "invalid session" in response.json()["detail"].lower()
        mock_cache_service.close.assert_called_once()


    async def test_query_endpoint_empty_results(self, test_client, mock_rag_service, mock_cache_service):
        """Test query endpoint when no relevant content found."""
        app.dependency_overrides[get_rag_service] = lambda: mock_rag_service
        app.dependency_overrides[get_cache_service] = lambda: mock_cache_service

        mock_rag_service.process_query.return_value = {
            "answer": "I don't find this information in the textbook.",
            "sources": [],
            "confidence": 0.0,
        }

        session_id = uuid4()

        response = await test_client.post(
            "/api/v1/chat/query",
            json={
                "question": "Unknown obscure topic?",
                "session_id": str(session_id),
            },
        )

        assert response.status_code == 200
        data = response.json()
        assert "i don't find this information in the textbook." in data["answer"].lower()
        assert len(data["sources"]) == 0
        assert data["confidence"] == 0.0
        mock_cache_service.close.assert_called_once()


    async def test_query_endpoint_validation_error(self, test_client): # Removed service mocks
        """Test query endpoint with invalid input (e.g., empty question)."""
        # No need to override services as validation happens before dependencies are called
        response = await test_client.post(
            "/api/v1/chat/query",
            json={
                "question": "",  # Empty question
                "session_id": str(uuid4()),
            },
        )

        assert response.status_code == 422
        assert "ensure this value has at least 1 characters" in response.json()["detail"][0]["msg"]


    async def test_query_endpoint_rate_limit_graceful_degradation(
        self, test_client, mock_rag_service, mock_cache_service
    ):
        """Test graceful degradation under rate limit."""
        app.dependency_overrides[get_rag_service] = lambda: mock_rag_service
        app.dependency_overrides[get_cache_service] = lambda: mock_cache_service

        mock_rag_service.process_query.side_effect = RateLimitException("Rate limited by external service")

        session_id = uuid4()
        request_payload = {
            "question": "What is AI?",
            "session_id": str(session_id),
        }

        # Temporarily enable graceful degradation in settings for this test
        original_degradation_setting = settings.enable_rate_limit_graceful_degradation
        settings.enable_rate_limit_graceful_degradation = True

        try:
            response = await test_client.post(
                "/api/v1/chat/query",
                json=request_payload,
            )

            assert response.status_code == 429
            assert "rate limited" in response.json()["detail"].lower()
            mock_cache_service.close.assert_called_once()
        finally:
            settings.enable_rate_limit_graceful_degradation = original_degradation_setting


    async def test_query_endpoint_question_length_validation(self, test_client): # Removed service mocks
        """Test question length validation."""
        long_question = "x" * 501

        response = await test_client.post(
            "/api/v1/chat/query",
            json={
                "question": long_question,
                "session_id": str(uuid4()),
            },
        )

        assert response.status_code == 422
        assert "Question must be 1-500 characters" in response.json()["detail"][0]["msg"]


    async def test_query_endpoint_response_structure(self, test_client, mock_rag_service, mock_cache_service):
        """Test that response has correct structure."""
        app.dependency_overrides[get_rag_service] = lambda: mock_rag_service
        app.dependency_overrides[get_cache_service] = lambda: mock_cache_service

        session_id = uuid4()
        question = "Test?"

        response = await test_client.post(
            "/api/v1/chat/query",
            json={
                "question": question,
                "session_id": str(session_id),
            },
        )

        data = response.json()

        # Verify response structure matches QueryResponse schema
        assert "answer" in data
        assert "sources" in data
        assert "session_id" in data
        assert "message_id" in data
        assert "confidence" in data

        # Verify sources structure
        for source in data["sources"]:
            assert "id" in source
            assert "chapter" in source
            assert "section" in source
            assert "content_excerpt" in source
            assert "confidence_score" in source
        
        mock_cache_service.close.assert_called_once()