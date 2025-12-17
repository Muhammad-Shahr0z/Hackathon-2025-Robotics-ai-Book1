# Implementation Plan: Docusaurus Language Switch Fix

**Branch**: `001-docusaurus-lang-switch-fix` | **Date**: 2025-12-17 | **Spec**: ../spec.md
**Input**: Feature specification from `/specs/001-docusaurus-lang-switch-fix/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Fix the Docusaurus language switch functionality that is causing malformed URLs like `/ur/ur/ur/` when switching to Urdu. The solution involves correcting the Docusaurus i18n configuration, ensuring proper localeDropdown implementation, and preventing URL duplication during language switching operations.

## Technical Context

**Language/Version**: JavaScript/TypeScript, Docusaurus v3.x
**Primary Dependencies**: Docusaurus core, @docusaurus/module-type-aliases, @docusaurus/types
**Storage**: N/A (static site generation)
**Testing**: Jest for unit tests, Cypress for e2e tests (NEEDS CLARIFICATION)
**Target Platform**: Web (static site deployment)
**Project Type**: Web application (Docusaurus documentation site)
**Performance Goals**: Fast page load times, proper routing behavior
**Constraints**: Must use Docusaurus built-in i18n only, no custom routing hacks
**Scale/Scope**: Single documentation site with English/Urdu languages

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Gate 1: Technical Accuracy
- [x] Docusaurus version specified (v3.x)
- [x] All configuration examples will be valid for Docusaurus v3.x
- [x] Implementation will follow Docusaurus official documentation

### Gate 2: Learning Progression
- [x] Solution will not affect existing content structure
- [x] Language switch functionality will enhance accessibility for Urdu speakers

### Gate 3: Simulation-First
- [x] Solution will be tested in development environment before deployment
- [x] Routing behavior will be validated locally

### Gate 4: AI-Native Workflow
- [x] Chapter has spec.md with clear requirements
- [x] Solution follows Docusaurus best practices

### Gate 5: Hardware Reality
- [x] Solution is purely frontend, no hardware requirements

### Gate 6: RAG Integration
- [x] Solution will not interfere with existing RAG chatbot functionality

### Gate 7: Personalization
- [x] Solution supports multilingual accessibility as required by constitution

### Gate 8: Multilingual Access
- [x] Solution enables Urdu language support as required by constitution

### Gate 9: Reusable Intelligence
- [x] Solution follows standard Docusaurus i18n patterns for reuse

## Project Structure

### Documentation (this feature)

```text
specs/001-docusaurus-lang-switch-fix/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
frontend/
├── docs/                # Documentation content
├── i18n/                # Translation files (ur/, en/)
│   └── ur/
│       └── docusaurus-plugin-content-docs/
│           └── current/ # Urdu translations
├── src/
│   ├── components/      # Custom React components
│   └── pages/           # Custom pages
├── docusaurus.config.js # Main Docusaurus configuration
├── package.json         # Dependencies
└── static/              # Static assets
```

**Structure Decision**: This is a web application focused on documentation with multilingual support. The fix will primarily involve updating the docusaurus.config.js file and ensuring proper i18n directory structure for Urdu translations.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [N/A] | [N/A] | [N/A] |