import pytest
from unittest.mock import patch, AsyncMock
from sqlalchemy.ext.asyncio import AsyncSession
import sqlalchemy.ext.asyncio as sa_asyncio # Import as alias to patch correctly

# Mock the settings first, as db imports settings
from src.config import settings
from src.db import _get_engine, _get_session_factory, get_db
import src.db as db_module # Import the module to access global variables

@pytest.fixture(autouse=True)
def mock_settings_db():
    """Fixture to mock database-related settings."""
    with patch('src.db.settings') as mock_db_settings:
        mock_db_settings.neon_database_url = "postgresql+asyncpg://user:password@localhost/test_db"
        mock_db_settings.debug = False
        mock_db_settings.database_pool_size = 5
        mock_db_settings.database_max_overflow = 10
        yield mock_db_settings

@pytest.fixture(autouse=True)
def reset_db_globals():
    """Fixture to reset global db module variables before each test."""
    original_engine = db_module._engine
    original_async_session_local = db_module._AsyncSessionLocal
    db_module._engine = None
    db_module._AsyncSessionLocal = None
    yield
    db_module._engine = original_engine
    db_module._AsyncSessionLocal = original_async_session_local

@pytest.mark.asyncio
async def test_get_engine_creation(mock_settings_db):
    """Test that _get_engine creates an engine with correct parameters."""
    with patch("src.db.create_async_engine") as mock_create_async_engine: # Patch where _get_engine looks it up
        engine = _get_engine()
        mock_create_async_engine.assert_called_once_with(
            mock_settings_db.neon_database_url,
            echo=mock_settings_db.debug,
            future=True,
            pool_size=mock_settings_db.database_pool_size,
            max_overflow=mock_settings_db.database_max_overflow,
            pool_recycle=3600,
            pool_pre_ping=True,
        )
        assert engine is not None

@pytest.mark.asyncio
async def test_get_engine_lazy_loading(mock_settings_db):
    """Test that _get_engine is lazy-loaded and only created once."""
    with patch("src.db.create_async_engine") as mock_create_async_engine: # Patch where _get_engine looks it up
        engine1 = _get_engine()
        engine2 = _get_engine()

        mock_create_async_engine.assert_called_once()
        assert engine1 is engine2

@pytest.mark.asyncio
async def test_get_session_factory_creation(mock_settings_db):
    """Test that _get_session_factory creates an async_sessionmaker."""
    with patch("src.db._get_engine") as mock_get_engine, \
         patch("src.db.async_sessionmaker") as mock_async_sessionmaker: # Patch where _get_session_factory looks it up
        
        mock_engine = AsyncMock()
        mock_get_engine.return_value = mock_engine

        session_factory = _get_session_factory()
        mock_async_sessionmaker.assert_called_once_with(
            mock_engine,
            class_=AsyncSession,
            expire_on_commit=False,
        )
        assert session_factory is not None

@pytest.mark.asyncio
async def test_get_session_factory_lazy_loading(mock_settings_db):
    """Test that _get_session_factory is lazy-loaded and only created once."""
    with patch("src.db.async_sessionmaker") as mock_async_sessionmaker, \
         patch("src.db._get_engine") as mock_get_engine:
        
        mock_get_engine.return_value = AsyncMock() # Ensure engine is mocked

        factory1 = _get_session_factory()
        factory2 = _get_session_factory()

        mock_async_sessionmaker.assert_called_once()
        assert factory1 is factory2

@pytest.mark.asyncio
async def test_get_db_dependency():
    """Test that get_db yields an AsyncSession and closes it."""
    # Mock the AsyncSession as an async context manager
    mock_session = AsyncMock(spec=AsyncSession)
    mock_session_maker_instance = AsyncMock()
    mock_session_maker_instance.__aenter__.return_value = mock_session
    mock_session_maker = AsyncMock(return_value=mock_session_maker_instance)

    with patch("src.db._get_session_factory", return_value=mock_session_maker):
        async for session in get_db():
            assert session is mock_session
            break # Exit after first yield

    mock_session.close.assert_called_once()
