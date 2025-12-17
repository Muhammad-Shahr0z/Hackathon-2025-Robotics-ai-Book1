# ADR-0003: Docusaurus i18n Approach for Urdu Translation

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-17
- **Feature:** Urdu Translation Support

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Implement multilingual support using Docusaurus built-in internationalization (i18n) functionality with right-to-left (RTL) layout support for Urdu. This approach involves:

- Configuring `docusaurus.config.js` with Urdu locale settings
- Setting up `i18n/ur/` directory structure for Urdu translations
- Implementing RTL (right-to-left) layout support for Urdu
- Maintaining all existing English content without modification
- Adding locale dropdown to navbar for language switching
- Following SEO-friendly URL structure with language-specific paths

## Consequences

### Positive

- Leverages Docusaurus native i18n capabilities ensuring compatibility with future updates
- Provides robust RTL layout support automatically when locale direction is set to 'rtl'
- Maintains all existing English content without modification
- SEO-friendly with language-specific URLs (e.g., /ur/docs/intro)
- Follows Docusaurus best practices for internationalization
- Preserves code blocks in English as required by project constitution
- Enables gradual translation with potential fallback to English

### Negative

- Requires creation of duplicate file structure for Urdu translations
- Increases build times with additional locale content
- Need for manual translation of all documentation content
- Potential for layout issues in RTL mode that require testing
- Additional maintenance overhead for keeping translations synchronized

## Alternatives Considered

Alternative 1: Custom Translation System
- Approach: Build custom infrastructure for multilingual support
- Why rejected: Would require significant development effort, harder to maintain, potential SEO problems, and doesn't leverage Docusaurus strengths

Alternative 2: Client-Side Translation API
- Approach: Use external API calls for real-time translation
- Why rejected: Would require external dependencies, potential latency issues, less reliable, doesn't work offline, and violates "no external API" requirement

Alternative 3: Pre-built Translation Plugins
- Approach: Use third-party translation plugins for Docusaurus
- Why rejected: Less control over translation quality and RTL implementation, potential compatibility issues, vendor lock-in concerns

## References

- Feature Spec: specs/001-urdu-translation/spec.md
- Implementation Plan: specs/001-urdu-translation/plan.md
- Related ADRs: ADR-0001 (Better Auth with MCP Server Integration), ADR-0002 (FastAPI Serverless Deployment on Vercel)
- Evaluator Evidence: specs/001-urdu-translation/research.md