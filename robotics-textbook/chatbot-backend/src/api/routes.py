"""FastAPI routes for chatbot API - Simplified version without database."""

import logging
from uuid import uuid4
from typing import List, Optional

from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel

from ..config import settings
from ..services.qdrant_service import get_qdrant_service, QdrantService
from ..services.openai_service import get_openai_service, OpenAIService
from ..services.cache_service import get_cache_service, CacheService
from ..services.rag_service import get_rag_service, RAGService
from ..services.selection_service import get_selection_service, SelectionService

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/api/v1/chat", tags=["Chat"])


class QueryRequest(BaseModel):
    """Request model for chat query."""
    question: str
    session_id: str
    page_context: Optional[str] = None


class SelectionRequest(BaseModel):
    """Request model for selection-based query."""
    question: str
    session_id: str
    selected_text: str
    chapter: str = ""
    section: str = ""


class CitationSchema(BaseModel):
    """Citation/source schema."""
    id: str
    chapter: str = ""
    section: str = ""
    content_excerpt: str = ""
    link: Optional[str] = None
    confidence_score: float = 0.0


class QueryResponse(BaseModel):
    """Response model for chat query."""
    answer: str
    sources: List[CitationSchema] = []
    session_id: str
    message_id: str
    confidence: float = 0.0


@router.post("/query", response_model=QueryResponse)
async def query_endpoint(
    request: QueryRequest,
    rag_service: RAGService = Depends(get_rag_service),
    cache_service: CacheService = Depends(get_cache_service)
) -> QueryResponse:
    """
    Process natural language question about textbook content.
    Simplified version - no database, just RAG pipeline.
    """
    try:
        logger.info(f"📝 Processing query: {request.question[:50]}...")

        # Process through RAG pipeline
        rag_result = await rag_service.process_query(
            question=request.question,
            session_id=request.session_id,
            page_context=request.page_context
        )

        answer = rag_result.get("answer", "Sorry, I couldn't find an answer.")
        confidence = rag_result.get("confidence", 0.0)
        
        # Format citations
        citations = []
        for src in rag_result.get("sources", []):
            payload = src.get("payload", {})
            citations.append(CitationSchema(
                id=str(src.get("id", uuid4())),
                chapter=payload.get("chapter", ""),
                section=payload.get("section", ""),
                content_excerpt=payload.get("content", "")[:300],
                confidence_score=src.get("score", 0.0)
            ))

        message_id = str(uuid4())
        
        logger.info(f"✅ Query complete: {len(citations)} sources, confidence {confidence:.2f}")
        
        return QueryResponse(
            answer=answer,
            sources=citations,
            session_id=request.session_id,
            message_id=message_id,
            confidence=confidence
        )

    except Exception as e:
        logger.error(f"❌ Error processing query: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/selection", response_model=QueryResponse)
async def selection_endpoint(
    request: SelectionRequest,
    selection_service: SelectionService = Depends(get_selection_service)
) -> QueryResponse:
    """Process question about selected text from the textbook."""
    try:
        logger.info(f"📖 Processing selection question: {request.question[:50]}...")
        logger.info(f"   Selected text: {request.selected_text[:100]}...")

        # Process through Selection service
        answer, citations, confidence = await selection_service.process_selection_question(
            selected_text=request.selected_text,
            question=request.question,
            chapter=request.chapter,
            section=request.section
        )

        message_id = str(uuid4())
        
        # Convert Citation objects to CitationSchema
        citation_schemas = [
            CitationSchema(
                id=str(uuid4()), # Citations from SelectionService don't have IDs
                chapter=c.chapter,
                section=c.section,
                content_excerpt=c.content_excerpt,
                link=c.link,
                confidence_score=c.confidence_score
            ) for c in citations
        ]

        return QueryResponse(
            answer=answer,
            sources=citation_schemas,
            session_id=request.session_id,
            message_id=message_id,
            confidence=confidence
        )

    except Exception as e:
        logger.error(f"❌ Error processing selection: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))