"""Unit tests for selection endpoint."""

import pytest
from unittest.mock import AsyncMock, patch, MagicMock # Added MagicMock import
from uuid import uuid4

from src.main import app
from src.models.schemas import QueryResponse
from src.services.selection_service import SelectionService, get_selection_service
from src.services.cache_service import CacheService, get_cache_service
from fastapi import HTTPException


@pytest.mark.asyncio
class TestSelectionEndpoint:
    """Tests for the /api/v1/chat/selection endpoint."""

    @pytest.fixture(autouse=True)
    def setup_and_teardown_dependency_overrides(self):
        """
        Fixture to clear FastAPI dependency overrides after each test,
        ensuring a clean state.
        """
        yield
        app.dependency_overrides = {}

    @pytest.fixture
    def mock_selection_service(self):
        """Mock SelectionService dependency."""
        service = AsyncMock(spec=SelectionService)
        service.process_selection_question.return_value = (
            "Publisher-subscriber pattern explanation",
            [
                MagicMock( # MagicMock is defined now
                    chapter="Module 1",
                    section="Communication",
                    content_excerpt="Pub-sub decouples producers from consumers...",
                    link=None,
                    confidence_score=0.92
                )
            ],
            0.92
        )
        return service
    
    @pytest.fixture
    def mock_cache_service(self):
        """Mock CacheService dependency."""
        service = AsyncMock(spec=CacheService)
        service.close.return_value = None
        return service


    async def test_selection_endpoint_success(self, test_client, mock_selection_service, mock_cache_service):
        """Test successful selection endpoint response."""
        app.dependency_overrides[get_selection_service] = lambda: mock_selection_service
        app.dependency_overrides[get_cache_service] = lambda: mock_cache_service

        session_id = uuid4()
        selected_text = "Robots are autonomous machines that perform tasks."
        question = "What are the key characteristics?"
        chapter = "Chapter 1"

        response = await test_client.post(
            "/api/v1/chat/selection",
            json={
                "selected_text": selected_text,
                "question": question,
                "session_id": str(session_id),
                "chapter": chapter,
            },
        )

        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        assert "sources" in data
        assert len(data["sources"]) > 0
        assert data["session_id"] == str(session_id)

        mock_selection_service.process_selection_question.assert_called_once_with(
            selected_text=selected_text,
            question=question,
            chapter=chapter,
            section=""
        )
        mock_cache_service.close.assert_called_once()


    async def test_selection_endpoint_invalid_session(self, test_client, mock_selection_service, mock_cache_service):
        """Test selection endpoint with invalid session."""
        mock_selection_service.process_selection_question.side_effect = HTTPException(status_code=401, detail="Invalid session")
        app.dependency_overrides[get_selection_service] = lambda: mock_selection_service
        app.dependency_overrides[get_cache_service] = lambda: mock_cache_service


        response = await test_client.post(
            "/api/v1/chat/selection",
            json={
                "selected_text": "Some text here",
                "question": "What is this?",
                "session_id": str(uuid4()),
                "chapter": "Chapter 1",
            },
        )

        assert response.status_code == 401
        assert "invalid session" in response.json()["detail"].lower()
        mock_cache_service.close.assert_called_once()


    async def test_selection_endpoint_too_short(self, test_client):
        """Test selection endpoint with text too short."""
        response = await test_client.post(
            "/api/v1/chat/selection",
            json={
                "selected_text": "Short",  # Less than 5 chars (min_length=5 in schema)
                "question": "What?",
                "session_id": str(uuid4()),
                "chapter": "Ch1",
            },
        )

        assert response.status_code == 422
        assert "String should have at least 5 characters" in response.json()["detail"][0]["msg"]


    async def test_selection_endpoint_too_long(self, test_client):
        """Test selection endpoint with text too long."""
        response = await test_client.post(
            "/api/v1/chat/selection",
            json={
                "selected_text": "x" * 5001,  # More than 5000 chars (max_length=5000 in schema)
                "question": "What?",
                "session_id": str(uuid4()),
                "chapter": "Ch1",
            },
        )

        assert response.status_code == 422
        assert "String should have at most 5000 characters" in response.json()["detail"][0]["msg"]


    async def test_selection_endpoint_question_required(self, test_client):
        """Test selection endpoint with missing question."""
        response = await test_client.post(
            "/api/v1/chat/selection",
            json={
                "selected_text": "Some text here",
                "session_id": str(uuid4()),
                "chapter": "Chapter 1",
            },
        )

        assert response.status_code == 422
        assert "Field required" in response.json()["detail"][0]["msg"]


    async def test_selection_endpoint_chapter_required(self, test_client):
        """Test selection endpoint with missing chapter."""
        response = await test_client.post(
            "/api/v1/chat/selection",
            json={
                "selected_text": "Some text here",
                "question": "What?",
                "session_id": str(uuid4()),
            },
        )

        assert response.status_code == 422
        assert "Field required" in response.json()["detail"][0]["msg"]


    async def test_selection_endpoint_response_structure(self, test_client, mock_selection_service, mock_cache_service):
        """Test selection response has correct structure."""
        app.dependency_overrides[get_selection_service] = lambda: mock_selection_service
        app.dependency_overrides[get_cache_service] = lambda: mock_cache_service

        session_id = uuid4()

        response = await test_client.post(
            "/api/v1/chat/selection",
            json={
                "selected_text": "Selected passage about robots",
                "question": "Explain this?",
                "session_id": str(session_id),
                "chapter": "Chapter 1",
            },
        )

        assert response.status_code == 200
        data = response.json()

        # Verify QueryResponse schema
        assert "answer" in data
        assert "sources" in data
        assert "session_id" in data
        assert "message_id" in data
        assert "confidence" in data

        # Verify citation structure
        for source in data["sources"]:
            assert "id" in source
            assert "chapter" in source
            assert "section" in source
            assert "content_excerpt" in source
            assert "confidence_score" in source
        
        mock_cache_service.close.assert_called_once()


    async def test_selection_endpoint_primary_citation_high_confidence(self, test_client, mock_selection_service, mock_cache_service):
        """Test that selected text has highest confidence in citations."""
        app.dependency_overrides[get_selection_service] = lambda: mock_selection_service
        app.dependency_overrides[get_cache_service] = lambda: mock_cache_service

        mock_selection_service.process_selection_question.return_value = (
            "Answer",
            [
                MagicMock(
                    chapter="Chapter 1",
                    section="Intro",
                    content_excerpt="Selected text about topic",
                    link=None,
                    confidence_score=1.0,
                ),
                MagicMock(
                    chapter="Chapter 2",
                    section="Related",
                    content_excerpt="Related content",
                    link=None,
                    confidence_score=0.7,
                ),
            ],
            0.88 # Calculated confidence
        )
        session_id = uuid4()

        response = await test_client.post(
            "/api/v1/chat/selection",
            json={
                "selected_text": "Selected text about topic",
                "question": "Explain this?",
                "session_id": str(session_id),
                "chapter": "Chapter 1",
            },
        )

        assert response.status_code == 200
        data = response.json()

        # First source (selected text) should have highest confidence
        assert len(data["sources"]) >= 1
        assert data["sources"][0]["confidence_score"] == 1.0
        mock_cache_service.close.assert_called_once()


    async def test_selection_endpoint_service_error(self, test_client, mock_selection_service, mock_cache_service):
        """Test selection endpoint handling service errors."""
        mock_selection_service.process_selection_question.side_effect = Exception("Selection service error")
        app.dependency_overrides[get_selection_service] = lambda: mock_selection_service
        app.dependency_overrides[get_cache_service] = lambda: mock_cache_service

        session_id = uuid4()

        response = await test_client.post(
            "/api/v1/chat/selection",
            json={
                "selected_text": "Some text here",
                "question": "What?",
                "session_id": str(session_id),
                "chapter": "Chapter 1",
            },
        )

        assert response.status_code == 500
        assert "selection service error" in response.json()["detail"].lower()
        mock_cache_service.close.assert_called_once()