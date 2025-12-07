import pytest
from unittest.mock import AsyncMock, patch

from src.config import settings
from src.services.qdrant_service import QdrantService, get_qdrant_service
from src.services.openai_service import OpenAIService, get_openai_service
from src.services.cache_service import CacheService, get_cache_service
from src.main import app # Import app to use app.dependency_overrides


@pytest.fixture(autouse=True)
def mock_settings():
    """
    Fixture to mock settings that might affect tests, ensuring consistent test environment.
    This fixture is autoused for all tests in this file.
    """
    with patch('src.main.settings') as mock_app_settings:
        # Set default values for testing
        mock_app_settings.debug = True
        mock_app_settings.log_level = "INFO"
        mock_app_settings.allowed_origins = ["http://localhost:3000"]
        mock_app_settings.rate_limit_enabled = False
        mock_app_settings.qdrant_url = "http://mock-qdrant:6333"
        mock_app_settings.qdrant_api_key = "mock-qdrant-key"
        mock_app_settings.qdrant_collection_name = "mock_collection"
        mock_app_settings.openai_api_key = "mock-openai-key"
        mock_app_settings.redis_url = "redis://mock-redis:6379"
        yield mock_app_settings


@pytest.mark.asyncio
async def test_root_endpoint(test_client): # test_client from conftest.py
    """Test the root endpoint."""
    response = await test_client.get("/")
    assert response.status_code == 200
    assert response.json() == {"message": "RAG Chatbot API", "version": "0.1.0"}

@pytest.mark.asyncio
async def test_metrics_endpoint(test_client): # test_client from conftest.py
    """Test the metrics endpoint."""
    response = await test_client.get("/metrics")
    assert response.status_code == 200
    assert "text/plain" in response.headers["content-type"]

@pytest.mark.asyncio
async def test_docs_endpoint(test_client): # test_client from conftest.py
    """Test the docs endpoint."""
    response = await test_client.get("/api/v1/docs")
    assert response.status_code == 200
    assert "RAG Chatbot API Documentation" in response.json()["title"]

@pytest.mark.asyncio
async def test_health_check_all_healthy(test_client): # test_client from conftest.py
    """Test the health check endpoint when all services are healthy."""
    mock_qdrant_service = AsyncMock(spec=QdrantService)
    mock_qdrant_service.health_check.return_value = True

    mock_openai_service = AsyncMock(spec=OpenAIService)
    mock_openai_service.health_check.return_value = True

    mock_cache_service = AsyncMock(spec=CacheService)
    mock_cache_service.initialize.return_value = None
    mock_cache_service.health_check.return_value = True

    app.dependency_overrides[get_qdrant_service] = lambda: mock_qdrant_service
    app.dependency_overrides[get_openai_service] = lambda: mock_openai_service
    app.dependency_overrides[get_cache_service] = lambda: mock_cache_service

    response = await test_client.get("/api/v1/health")
    assert response.status_code == 200
    assert response.json()["status"] == "ok"
    assert response.json()["qdrant_status"] == "healthy"
    assert response.json()["gemini_status"] == "healthy" # gemini_status refers to openai_ok
    assert response.json()["neon_status"] == "healthy" # Neon status is hardcoded to healthy for now

    app.dependency_overrides = {}


@pytest.mark.asyncio
async def test_health_check_degraded(test_client): # test_client from conftest.py
    """Test the health check endpoint when one service is degraded (e.g., Qdrant down)."""
    mock_qdrant_service = AsyncMock(spec=QdrantService)
    mock_qdrant_service.health_check.return_value = False

    mock_openai_service = AsyncMock(spec=OpenAIService)
    mock_openai_service.health_check.return_value = True

    mock_cache_service = AsyncMock(spec=CacheService)
    mock_cache_service.initialize.return_value = None
    mock_cache_service.health_check.return_value = True

    app.dependency_overrides[get_qdrant_service] = lambda: mock_qdrant_service
    app.dependency_overrides[get_openai_service] = lambda: mock_openai_service
    app.dependency_overrides[get_cache_service] = lambda: mock_cache_service

    response = await test_client.get("/api/v1/health")
    assert response.status_code == 200
    assert response.json()["status"] == "degraded"
    assert response.json()["qdrant_status"] == "down"
    assert response.json()["gemini_status"] == "healthy"
    assert response.json()["neon_status"] == "healthy"

    app.dependency_overrides = {}


@pytest.mark.asyncio
async def test_health_check_all_down(test_client): # test_client from conftest.py
    """Test the health check endpoint when all services are down."""
    mock_qdrant_service = AsyncMock(spec=QdrantService)
    mock_qdrant_service.health_check.return_value = False

    mock_openai_service = AsyncMock(spec=OpenAIService)
    mock_openai_service.health_check.return_value = False

    mock_cache_service = AsyncMock(spec=CacheService)
    mock_cache_service.initialize.return_value = None
    mock_cache_service.health_check.return_value = False

    app.dependency_overrides[get_qdrant_service] = lambda: mock_qdrant_service
    app.dependency_overrides[get_openai_service] = lambda: mock_openai_service
    app.dependency_overrides[get_cache_service] = lambda: mock_cache_service

    # Mock settings.openai_api_key to be empty to simulate OpenAI being down
    with patch.object(settings, 'openai_api_key', new=''):
        response = await test_client.get("/api/v1/health")
        assert response.status_code == 200
        assert response.json()["status"] == "down"
        assert response.json()["qdrant_status"] == "down"
        assert response.json()["gemini_status"] == "down"
        assert response.json()["neon_status"] == "healthy"

    app.dependency_overrides = {}