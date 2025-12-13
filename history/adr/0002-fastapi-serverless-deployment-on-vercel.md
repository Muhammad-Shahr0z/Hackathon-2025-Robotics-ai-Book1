# ADR-0002: FastAPI Serverless Deployment on Vercel

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-14
- **Feature:** vercel-backend-deployment

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Deploy the existing Python FastAPI backend to Vercel as serverless functions using the Vercel MCP server. This decision encompasses:

- Framework: FastAPI (preserved from existing backend)
- Runtime: Python 3.11 with Vercel Python runtime
- Deployment: Vercel serverless functions via MCP server
- Architecture: Serverless function model with auto-scaling
- Configuration: vercel.json for deployment settings

## Consequences

### Positive

- Automatic scaling based on demand without manual intervention
- Reduced operational overhead with Vercel handling infrastructure
- Cost efficiency through pay-per-execution model
- Fast global deployments with Vercel's edge network
- Seamless integration with Vercel's development tools and preview deployments
- Maintains existing FastAPI codebase with minimal changes

### Negative

- Potential cold start latency for infrequently accessed endpoints
- Vendor lock-in to Vercel's platform and deployment model
- Limited control over server environment compared to traditional hosting
- Possible function size limitations that may affect dependency inclusion
- Dependency on Vercel's MCP server availability and functionality
- Constraints of serverless environment (execution time limits, memory)

## Alternatives Considered

Alternative Stack A: Traditional Container Deployment (Docker) on AWS/Azure
- Pros: More control over runtime environment, closer to standard deployment practices
- Cons: More complex setup, doesn't leverage Vercel MCP, violates requirement to use Vercel platform

Alternative Stack B: Alternative Platforms (Railway, Render, Fly.io)
- Pros: Similar serverless capabilities, potentially easier FastAPI integration
- Cons: Explicitly prohibited by requirements, violates constraint to use only Vercel

Alternative Stack C: Vercel with Different Frameworks (Next.js API Routes)
- Pros: Native Vercel integration, potentially simpler setup
- Cons: Would require significant refactoring of existing FastAPI backend, violates "no refactoring unrelated code" requirement

Alternative Stack D: Server-Side Deployment on Vercel
- Pros: Might be simpler for existing FastAPI code
- Cons: Doesn't meet requirement for serverless functions, doesn't use Vercel's primary serverless architecture

## References

- Feature Spec: specs/001-vercel-backend-deployment/spec.md
- Implementation Plan: specs/001-vercel-backend-deployment/plan.md
- Related ADRs: none
- Evaluator Evidence: history/prompts/001-vercel-backend-deployment/1-backend-deployment-on-vercel-spec.spec.prompt.md, history/prompts/001-vercel-backend-deployment/2-backend-deployment-on-vercel-plan.plan.prompt.md