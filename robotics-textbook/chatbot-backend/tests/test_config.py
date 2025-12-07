import os
import pytest
from unittest.mock import patch

from src.config import Settings


def test_settings_default_values(monkeypatch):
    """Test that default values are loaded correctly when no environment variables are set."""
    # Ensure NEON_DATABASE_URL is not set to test its default value
    monkeypatch.delenv("NEON_DATABASE_URL", raising=False)
    # Set a dummy OPENAI_API_KEY to satisfy the validator for default value checks
    monkeypatch.setenv("OPENAI_API_KEY", "dummy_key")

    settings = Settings()
    assert settings.neon_database_url == "postgresql://user:password@localhost/chatbot_dev"
    assert settings.database_pool_size == 10
    assert settings.qdrant_url == "https://1521bc26-af63-4594-8df5-c4a2e64c549b.us-east4-0.gcp.cloud.qdrant.io:6333"
    assert settings.openai_api_key == "dummy_key"
    assert settings.environment == "development"
    assert settings.debug is True
    assert settings.log_level == "INFO"
    assert settings.allowed_origins == ["http://localhost:3000", "http://localhost:8000", "https://Muhammad-Shahr0z.github.io"]
    assert settings.redis_url == ""
    assert settings.rate_limit_enabled is True


def test_settings_environment_variables_loading(monkeypatch):
    """Test that environment variables are loaded correctly."""
    monkeypatch.setenv("NEON_DATABASE_URL", "postgres://test:test@test/test")
    monkeypatch.setenv("DATABASE_POOL_SIZE", "20")
    monkeypatch.setenv("QDRANT_URL", "http://test-qdrant:6333")
    monkeypatch.setenv("QDRANT_API_KEY", "test-qdrant-key")
    monkeypatch.setenv("OPENAI_API_KEY", "test-openai-key")
    monkeypatch.setenv("OPENAI_MODEL", "gpt-3.5-turbo")
    monkeypatch.setenv("ENVIRONMENT", "production")
    monkeypatch.setenv("DEBUG", "false")
    monkeypatch.setenv("LOG_LEVEL", "DEBUG")
    monkeypatch.setenv("ALLOWED_ORIGINS", "http://test.com, https://test.com")
    monkeypatch.setenv("REDIS_URL", "redis://test-redis:6379")
    monkeypatch.setenv("RATE_LIMIT_ENABLED", "false")

    settings = Settings()
    assert settings.neon_database_url == "postgres://test:test@test/test"
    assert settings.database_pool_size == 20
    assert settings.qdrant_url == "http://test-qdrant:6333"
    assert settings.qdrant_api_key == "test-qdrant-key"
    assert settings.openai_api_key == "test-openai-key"
    assert settings.openai_model == "gpt-3.5-turbo"
    assert settings.environment == "production"
    assert settings.debug is False
    assert settings.log_level == "DEBUG"
    assert settings.allowed_origins == ["http://test.com", "https://test.com"]
    assert settings.redis_url == "redis://test-redis:6379"
    assert settings.rate_limit_enabled is False


def test_openai_api_key_validator(monkeypatch):
    """Test that ValueError is raised if OPENAI_API_KEY is not set."""
    # Ensure OPENAI_API_KEY is not set for this test
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    with pytest.raises(ValueError, match="OPENAI_API_KEY must be set"):
        Settings()


def test_allowed_origins_parser_single_origin(monkeypatch):
    """Test parsing of a single allowed origin string."""
    monkeypatch.setenv("OPENAI_API_KEY", "dummy_key") # Required by validator
    monkeypatch.setenv("ALLOWED_ORIGINS", "http://single.com")
    settings = Settings()
    assert settings.allowed_origins == ["http://single.com"]


def test_allowed_origins_parser_multiple_origins(monkeypatch):
    """Test parsing of multiple comma-separated allowed origins."""
    monkeypatch.setenv("OPENAI_API_KEY", "dummy_key") # Required by validator
    monkeypatch.setenv("ALLOWED_ORIGINS", "http://first.com,https://second.com ")
    settings = Settings()
    assert settings.allowed_origins == ["http://first.com", "https://second.com"]


def test_allowed_origins_parser_empty_string(monkeypatch):
    """Test parsing of an empty allowed origins string."""
    monkeypatch.setenv("OPENAI_API_KEY", "dummy_key") # Required by validator
    monkeypatch.setenv("ALLOWED_ORIGINS", "")
    settings = Settings()
    assert settings.allowed_origins == []


def test_allowed_origins_parser_whitespace(monkeypatch):
    """Test parsing of allowed origins with extra whitespace."""
    monkeypatch.setenv("OPENAI_API_KEY", "dummy_key") # Required by validator
    monkeypatch.setenv("ALLOWED_ORIGINS", " http://a.com , https://b.com ")
    settings = Settings()
    assert settings.allowed_origins == ["http://a.com", "https://b.com"]