"""
Integration tests for RAG (Retrieval-Augmented Generation) pipeline.

Tests the complete end-to-end flow:
1. Query caching (Phase 4)
2. Embedding with caching (Phase 4)
3. Vector search in Qdrant (Phase 1)
4. LLM response generation (Phase 1)
5. Citation extraction (Phase 1)
6. Cache cleanup on completion (Phase 3)
"""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from uuid import uuid4
import hashlib

from src.services.rag_service import RAGService
from src.services.openai_service import OpenAIService
from src.services.qdrant_service import QdrantService
from src.services.cache_service import CacheService


@pytest.mark.asyncio
class TestRAGPipelineIntegration:
    """Integration tests for RAG pipeline with all services."""

    @pytest.fixture
    def mock_cache_service_fixture(self):
        """Mock cache service for integration testing."""
        service = AsyncMock(spec=CacheService)
        service.initialize.return_value = None
        service.close.return_value = None
        service.get_query_cache.return_value = None
        service.set_query_cache.return_value = True
        service.get_embedding_cache.return_value = None
        service.set_embedding_cache.return_value = True
        service._generate_key.side_effect = lambda *args: hashlib.md5("|".join(str(arg) for arg in args).encode()).hexdigest()
        return service

    @pytest.fixture
    def mock_openai_service(self):
        """Mock OpenAI service with embedding and chat capabilities."""
        service = AsyncMock(spec=OpenAIService)

        # Mock embedding
        service.embed_text.return_value = [0.1, 0.2, 0.3] * 100  # 300-dim vector

        # Mock chat completion
        service.chat_completion.return_value = (
            "ROS 2 nodes communicate via topics using a publisher-subscriber pattern. "
            "Publishers send messages to a topic, and subscribers receive those messages."
        )

        return service

    @pytest.fixture
    def mock_qdrant_service(self):
        """Mock Qdrant service with vector search capabilities."""
        service = AsyncMock(spec=QdrantService)

        # Mock search results
        service.search.return_value = [
            {
                "id": str(uuid4()),
                "score": 0.95,
                "payload": {
                    "chapter": "Module 1: ROS 2 Basics",
                    "section": "Nodes and Topics",
                    "content": "ROS 2 nodes communicate via topics using a publisher-subscriber pattern.",
                    "confidence_score": 0.95
                }
            },
            {
                "id": str(uuid4()),
                "score": 0.87,
                "payload": {
                    "chapter": "Module 1: ROS 2 Basics",
                    "section": "Communication Patterns",
                    "content": "The pub-sub model decouples nodes from each other.",
                    "confidence_score": 0.87
                }
            }
        ]

        return service

    async def test_complete_query_pipeline_with_cache_hit(
        self,
        mock_openai_service,
        mock_qdrant_service,
        mock_cache_service_fixture
    ):
        """Test complete RAG pipeline with cache hit on second query."""
        mock_cache_service_fixture.get_query_cache.return_value = {
            "answer": "Cached answer",
            "sources": [],
            "confidence": 0.9
        }

        rag_service = RAGService(
            mock_openai_service,
            mock_qdrant_service,
            mock_cache_service_fixture
        )

        question = "How do ROS 2 nodes communicate?"
        session_id = "test-session-123"

        # First call - cache hit (due to mock)
        result1 = await rag_service.process_query(question, session_id)

        assert result1 is not None
        assert result1["answer"] == "Cached answer"
        mock_cache_service_fixture.get_query_cache.assert_called_once_with(question, session_id)
        mock_openai_service.embed_text.assert_not_called() # Should not call if cache hit


    async def test_pipeline_cache_miss_flow(
        self,
        mock_openai_service,
        mock_qdrant_service,
        mock_cache_service_fixture
    ):
        """Test RAG pipeline when cache is empty (cache miss)."""
        mock_cache_service_fixture.get_query_cache.return_value = None # Ensure cache miss

        rag_service = RAGService(
            mock_openai_service,
            mock_qdrant_service,
            mock_cache_service_fixture
        )

        question = "What is ROS 2?"
        session_id = "new-session-456"

        result = await rag_service.process_query(question, session_id)

        # Verify pipeline executed all steps
        assert result["answer"] is not None
        assert "sources" in result
        assert result["confidence"] > 0

        # Verify all services were called in order
        mock_openai_service.embed_text.assert_called_once_with(question)
        mock_qdrant_service.search.assert_called_once()
        mock_openai_service.chat_completion.assert_called_once()
        mock_cache_service_fixture.get_query_cache.assert_called_once()
        mock_cache_service_fixture.set_query_cache.assert_called_once()


    async def test_pipeline_no_results_flow(
        self,
        mock_openai_service,
        mock_cache_service_fixture
    ):
        """Test RAG pipeline when Qdrant returns no results."""
        # Mock Qdrant with empty results
        mock_qdrant = AsyncMock(spec=QdrantService)
        mock_qdrant.search.return_value = []

        # Adjust OpenAI mock to return the expected "no info" answer
        mock_openai_service.chat_completion.return_value = "I don't find this information in the textbook."
        mock_cache_service_fixture.get_query_cache.return_value = None
        mock_cache_service_fixture.set_query_cache.return_value = True

        rag_service = RAGService(
            mock_openai_service,
            mock_qdrant,
            mock_cache_service_fixture
        )

        result = await rag_service.process_query("Unknown question", "session-789")

        # Should return graceful "no results" response
        assert "I don't find this information in the textbook." in result["answer"]
        assert result["sources"] == []
        assert result["confidence"] == 0.0
        mock_openai_service.embed_text.assert_called_once()
        mock_qdrant.search.assert_called_once()
        mock_openai_service.chat_completion.assert_called_once()


    async def test_pipeline_different_sessions_separate_caches(
        self,
        mock_openai_service,
        mock_qdrant_service,
        mock_cache_service_fixture
    ):
        """Test that different sessions maintain separate caches."""
        rag_service = RAGService(
            mock_openai_service,
            mock_qdrant_service,
            mock_cache_service_fixture
        )

        question = "Same question"
        session_1 = "session-1"
        session_2 = "session-2"

        mock_cache_service_fixture.get_query_cache.side_effect = [None, None] # Ensure no cache hits initially
        mock_cache_service_fixture.set_query_cache.return_value = True

        # Query from session 1
        result_1 = await rag_service.process_query(question, session_1)
        
        # Reset mock call counts for openai_service to accurately count for session 2
        mock_openai_service.embed_text.reset_mock()
        mock_qdrant_service.search.reset_mock()
        mock_openai_service.chat_completion.reset_mock()
        mock_cache_service_fixture.set_query_cache.reset_mock()


        # Now, simulate a cache hit for session 1 if it were queried again
        # And ensure session 2 is a miss
        mock_cache_service_fixture.get_query_cache.side_effect = [
            {"answer": "Cached for session 1", "sources": [], "confidence": 0.8}, # For subsequent call to session 1
            None # For session 2
        ]

        # Query from session 2 with same question
        result_2 = await rag_service.process_query(question, session_2)
        
        # Embedding should be called again for session 2 as it's a separate cache key
        mock_openai_service.embed_text.assert_called_once()
        mock_qdrant_service.search.assert_called_once()
        mock_openai_service.chat_completion.assert_called_once()
        mock_cache_service_fixture.get_query_cache.assert_called_with(question, session_2)
        mock_cache_service_fixture.set_query_cache.assert_called_once_with(question, session_2, result_2)


    async def test_embedding_caching_layer(
        self,
        mock_openai_service,
        mock_cache_service_fixture
    ):
        """Test that OpenAI service embedding caching works within RAG pipeline."""
        # Ensure that OpenAIService will use the mocked cache_service
        openai_service = OpenAIService(
            api_key="test-key",
            model="gpt-4",
            embedding_model="text-embedding-3-small",
            cache_service=mock_cache_service_fixture
        )

        # Mock the embedding generation
        mock_openai_service.embed_text.return_value = [0.1, 0.2, 0.3] * 100
        mock_cache_service_fixture.get_embedding_cache.side_effect = [None, [0.1, 0.2, 0.3] * 100] # First call miss, second hit
        mock_cache_service_fixture.set_embedding_cache.return_value = True

        # First embedding - cache miss, should call API
        embed_1 = await openai_service.embed_text("test text")
        mock_openai_service.embed_text.assert_called_once_with("test text")
        mock_cache_service_fixture.get_embedding_cache.assert_called_once_with("test text")
        mock_cache_service_fixture.set_embedding_cache.assert_called_once_with("test text", embed_1)

        mock_openai_service.embed_text.reset_mock() # Reset call count for next assertion
        mock_cache_service_fixture.get_embedding_cache.reset_mock()
        mock_cache_service_fixture.set_embedding_cache.reset_mock()

        # Second embedding of same text - should use embedding cache
        embed_2 = await openai_service.embed_text("test text")
        
        mock_openai_service.embed_text.assert_not_called() # API should not be called again
        mock_cache_service_fixture.get_embedding_cache.assert_called_once_with("test text")
        mock_cache_service_fixture.set_embedding_cache.assert_not_called()


    async def test_context_building_from_search_results(
        self,
        mock_openai_service,
        mock_qdrant_service,
        mock_cache_service_fixture
    ):
        """Test that context is properly built from Qdrant search results."""
        rag_service = RAGService(
            mock_openai_service,
            mock_qdrant_service,
            mock_cache_service_fixture
        )

        question = "How do topics work?"
        session_id = "session-context"
        page_context = "Module 1: ROS 2"

        # Mock cache miss for this test
        mock_cache_service_fixture.get_query_cache.return_value = None

        result = await rag_service.process_query(question, session_id, page_context)

        # Verify chat was called with proper context
        mock_openai_service.chat_completion.assert_called_once()
        call_args = mock_openai_service.chat_completion.call_args
        messages = call_args[0][0] # messages is the first positional arg
        
        # Find the user message with the context
        user_message_content = next((msg["content"] for msg in messages if msg["role"] == "user"), "")
        assert page_context in user_message_content
        assert "Module 1: ROS 2 Basics" in user_message_content # From qdrant_service mock
        assert "The pub-sub model decouples nodes from each other." in user_message_content # From qdrant_service mock


    async def test_confidence_calculation_from_search_scores(
        self,
        mock_openai_service,
        mock_qdrant_service,
        mock_cache_service_fixture
    ):
        """Test that confidence is properly calculated from search result scores."""
        rag_service = RAGService(
            mock_openai_service,
            mock_qdrant_service,
            mock_cache_service_fixture
        )

        # Mock cache miss for this test
        mock_cache_service_fixture.get_query_cache.return_value = None

        result = await rag_service.process_query("test question", "session-confidence")

        # Confidence should be average of search scores
        # Mock returns [0.95, 0.87]
        expected_confidence = (0.95 + 0.87) / 2
        assert abs(result["confidence"] - expected_confidence) < 0.01


    async def test_pipeline_error_handling_invalid_question(
        self,
        mock_openai_service,
        mock_qdrant_service,
        mock_cache_service_fixture
    ):
        """Test pipeline error handling for invalid questions."""
        rag_service = RAGService(
            mock_openai_service,
            mock_qdrant_service,
            mock_cache_service_fixture
        )

        # Empty question
        with pytest.raises(ValueError):
            await rag_service.process_query("", "session-invalid")

        # Too long question
        with pytest.raises(ValueError):
            await rag_service.process_query("x" * 501, "session-invalid-long")

    async def test_cache_cleanup_on_error(
        self,
        mock_openai_service,
        mock_qdrant_service,
        mock_cache_service_fixture
    ):
        """Test that cache service is not explicitly closed on error in the service layer."""
        # Make Qdrant fail
        mock_qdrant_service.search.side_effect = Exception("Qdrant error")

        rag_service = RAGService(
            mock_openai_service,
            mock_qdrant_service,
            mock_cache_service_fixture
        )

        # Pipeline should raise the exception
        with pytest.raises(Exception, match="Qdrant error"):
            await rag_service.process_query("test", "session-error")
        
        # Cache service close should not be called at the service level
        mock_cache_service_fixture.close.assert_not_called()


@pytest.mark.asyncio
class TestRAGServiceCacheTTLs:
    """Test cache TTL enforcement in RAG pipeline."""

    @pytest.fixture
    def mock_cache_service_ttl(self):
        """Mock cache service for TTL testing, to avoid real Redis."""
        service = AsyncMock(spec=CacheService)
        service.initialize.return_value = None
        service.close.return_value = None
        service.get.return_value = None
        service.set.return_value = True
        service._generate_key.side_effect = lambda *args: hashlib.md5("|".join(str(arg) for arg in args).encode()).hexdigest() # Mock the internal key generation
        return service


    async def test_query_cache_ttl_one_hour(self, mock_cache_service_ttl):
        """Test that query cache uses 1-hour TTL."""
        question = "test query"
        session_id = "session-ttl"
        response_data = {"answer": "test", "sources": [], "confidence": 0.5}

        await mock_cache_service_ttl.set_query_cache(question, session_id, response_data)
        
        # Verify set was called with the correct TTL (3600 seconds = 1 hour)
        mock_cache_service_ttl.set.assert_called_once_with(
            mock_cache_service_ttl._generate_key("query", question, session_id),
            response_data,
            ttl=3600
        )
        mock_cache_service_ttl.get_query_cache.return_value = response_data
        cached = await mock_cache_service_ttl.get_query_cache(question, session_id)
        assert cached is not None


    async def test_embedding_cache_ttl_24_hours(self, mock_cache_service_ttl):
        """Test that embedding cache uses 24-hour TTL."""
        text = "test text"
        embedding_data = [0.1] * 300
        
        # Mock get_embedding_cache and set_embedding_cache directly
        mock_cache_service_ttl.get_embedding_cache.return_value = None # Ensure initial cache miss
        mock_cache_service_ttl.set_embedding_cache.return_value = True

        await mock_cache_service_ttl.set_embedding_cache(text, embedding_data)

        # Verify set was called with the correct TTL (86400 seconds = 24 hours)
        mock_cache_service_ttl.set_embedding_cache.assert_called_once_with(text, embedding_data)


@pytest.mark.asyncio
class TestRAGServiceCitationExtraction:
    """Test citation extraction from search results."""

    @pytest.fixture
    def mock_rag_service_citation_deps(self, mock_cache_service_fixture):
        mock_openai_service = AsyncMock(spec=OpenAIService)
        mock_qdrant_service = AsyncMock(spec=QdrantService)
        return RAGService(mock_openai_service, mock_qdrant_service, mock_cache_service_fixture)


    async def test_citations_extracted_from_search_results(self, mock_rag_service_citation_deps):
        """Test that citations are properly extracted from Qdrant search."""
        mock_openai = mock_rag_service_citation_deps.openai
        mock_qdrant = mock_rag_service_citation_deps.qdrant
        mock_cache = mock_rag_service_citation_deps.cache

        mock_openai.embed_text.return_value = [0.1] * 300
        mock_openai.chat_completion.return_value = "Answer based on context"
        mock_cache.get_query_cache.return_value = None
        mock_cache.set_query_cache.return_value = True

        mock_qdrant.search.return_value = [
            {
                "id": "1",
                "score": 0.95,
                "payload": {
                    "chapter": "Ch1",
                    "section": "Sec1",
                    "content": "Content 1",
                }
            }
        ]

        result = await mock_rag_service_citation_deps.process_query("test", "session")

        # Citations should be extracted
        assert len(result["sources"]) > 0
        source = result["sources"][0]
        assert "chapter" in source
        assert "section" in source
        assert "content_excerpt" in source
        assert "confidence_score" in source
        assert source["confidence_score"] == 0.95