# Implementation Tasks: Docusaurus Language Switch Fix

**Feature**: Docusaurus Language Switch Fix
**Branch**: `001-docusaurus-lang-switch-fix`
**Spec**: ../spec.md
**Date**: 2025-12-17

## Implementation Strategy

This implementation follows the Spec-Driven Development approach, with tasks organized by user story priority. Each user story is independently testable and represents a complete, valuable increment. The approach prioritizes fixing the core routing issue first (MVP), then implementing supporting features like RTL layout.

**MVP Scope**: User Story 1 (core language switching functionality)
**Delivery Approach**: Phase 1-2 (Setup & Foundation) → User Story 1 → User Story 2 → User Story 3 → Polish

## Phase 1: Setup

- [x] T001 Create frontend directory structure if not exists
- [x] T002 Verify Docusaurus v3.x is installed and configured
- [x] T003 Set up i18n directory structure for Urdu translations
- [x] T004 Create placeholder Urdu translation files to match English structure

## Phase 2: Foundational

- [x] T005 [P] Configure proper i18n settings in docusaurus.config.js
- [x] T006 [P] Implement correct localeDropdown in navbar configuration
- [x] T007 [P] Set up RTL support configuration for Urdu locale
- [x] T008 [P] Create base CSS for RTL layout support

## Phase 3: User Story 1 - Switch Language Without Routing Issues (P1)

**Goal**: Enable users to switch between English and Urdu languages without experiencing broken routing, so that they can access content in their preferred language seamlessly.

**Independent Test Criteria**: When clicking the language switcher once, the site should switch to the selected language with the correct URL structure (e.g., /ur/ for Urdu) and all content should be properly translated without any "Page Not Found" errors.

**Tasks**:

- [x] T009 [P] [US1] Verify current docusaurus.config.js i18n configuration
- [x] T010 [P] [US1] Fix i18n configuration to prevent URL duplication
- [x] T011 [US1] Update localeDropdown implementation in navbar
- [x] T012 [US1] Test language switching with basic content
- [x] T013 [US1] Validate correct URL generation for both languages
- [x] T014 [US1] Ensure no "Page Not Found" errors after language switch

## Phase 4: User Story 2 - Idempotent Language Switching (P1)

**Goal**: Allow users to click the same language button multiple times without the URL becoming malformed, so that the site remains navigable and functional.

**Independent Test Criteria**: When clicking the Urdu language button multiple times, the URL should remain `/ur/` and not become `/ur/ur/ur/` or similar malformed paths.

**Tasks**:

- [x] T015 [P] [US2] Identify current language switching mechanism causing duplication
- [x] T016 [P] [US2] Remove any custom URL manipulation code
- [x] T017 [US2] Implement proper idempotent behavior for language switching
- [x] T018 [US2] Test multiple clicks on same language button
- [x] T019 [US2] Verify URL remains stable after multiple clicks
- [x] T020 [US2] Ensure language state persists correctly

## Phase 5: User Story 3 - RTL Layout Support (P2)

**Goal**: Ensure that when reading Urdu content, the layout properly supports RTL (right-to-left) direction, so that the content is displayed correctly and is readable.

**Independent Test Criteria**: When viewing Urdu content, text should flow from right to left, and UI elements should be properly aligned for RTL reading.

**Tasks**:

- [x] T021 [P] [US3] Implement RTL CSS rules for Urdu content
- [x] T022 [P] [US3] Add RTL support to navigation elements
- [x] T023 [US3] Test RTL layout with sample Urdu content
- [x] T024 [US3] Validate text alignment and direction
- [x] T025 [US3] Adjust UI components for RTL compatibility
- [x] T026 [US3] Verify all elements render correctly in RTL mode

## Phase 6: Polish & Cross-Cutting Concerns

- [x] T027 [P] Test navigation between translated pages
- [x] T028 [P] Validate all existing English functionality remains intact
- [x] T029 [P] Optimize performance for language switching
- [x] T030 [P] Add error handling for missing translations
- [x] T031 [P] Document the implementation for future maintenance
- [x] T032 [P] Create comprehensive test suite for language switching
- [x] T033 [P] Verify production build works with new i18n configuration

## Dependencies

**User Story Order**:
1. User Story 1 (P1) - Core language switching functionality (MUST complete first)
2. User Story 2 (P1) - Idempotent behavior (depends on US1)
3. User Story 3 (P2) - RTL layout (can be parallel with US2 after US1)

**Task Dependencies**:
- T005-T008 (Foundation) must complete before any user story tasks
- US1 tasks must complete before US2 can begin
- US3 can be worked in parallel with US2 after US1 completion

## Parallel Execution Examples

**Per User Story**:
- US1: T009 and T010 can be done in parallel with T011
- US2: T015 and T016 can be done in parallel
- US3: T021 and T022 can be done in parallel