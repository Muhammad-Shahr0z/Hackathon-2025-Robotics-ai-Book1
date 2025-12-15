---
description: "Task list for GitHub Pages Image Path Fix implementation"
---

# Tasks: GitHub Pages Image Path Fix

**Input**: Design documents from `/specs/001-image-path-fix/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: No explicit tests requested in the feature specification, so test tasks are not included.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus project**: `frontend/` at repository root
- **Components**: `frontend/src/components/`
- **Static assets**: `frontend/static/`
- **Configuration**: `frontend/docusaurus.config.ts`
- **Main page**: `frontend/src/pages/index.tsx`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Verify project structure exists with frontend/ directory
- [x] T002 [P] Confirm Docusaurus installation and dependencies
- [x] T003 [P] Locate all files that reference robologo.png

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Place robologo.png in frontend/static/img/ directory
- [x] T005 [P] Identify all components that use robologo.png (hero section, card slides)
- [x] T006 [P] Document current image paths in all components
- [x] T007 Verify local development environment (npm run start) works
- [x] T008 Verify build process (npm run build) works before changes

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - View Logo Correctly on GitHub Pages (Priority: P1) 🎯 MVP

**Goal**: Ensure robologo.png displays correctly in hero section and card slides on GitHub Pages

**Independent Test**: Visit GitHub Pages deployment and verify robologo.png appears correctly in all locations (hero section and card slides)

### Implementation for User Story 1

- [x] T009 [P] [US1] Update hero section component to use correct path to robologo.png
- [x] T010 [P] [US1] Update card slides component to use correct path to robologo.png
- [x] T011 [US1] Test locally with npm run start to verify images appear in both locations
- [x] T012 [US1] Build locally with npm run build and test with npm run serve
- [x] T013 [US1] Deploy changes and verify images appear correctly on GitHub Pages

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Consistent Image Display Across Environments (Priority: P2)

**Goal**: Ensure the same image path works consistently between local development and GitHub Pages

**Independent Test**: Verify the same image path works correctly in both local development and GitHub Pages deployment

### Implementation for User Story 2

- [x] T014 [P] [US2] Review all image path references to ensure they follow Docusaurus static asset convention
- [x] T015 [P] [US2] Update any remaining image references to use relative paths compatible with GitHub Pages
- [x] T016 [US2] Test local development environment (npm run start) to confirm images still work
- [x] T017 [US2] Test GitHub Pages deployment to confirm images work consistently
- [x] T018 [US2] Validate that same codebase functions in both environments per SC-003

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Maintain Image Accessibility (Priority: P3)

**Goal**: Ensure the logo image maintains proper alt text and accessibility features across environments

**Independent Test**: Examine image markup and verify accessibility attributes are preserved after the path fix

### Implementation for User Story 3

- [x] T019 [P] [US3] Review current robologo.png markup for accessibility attributes (alt text, title, etc.)
- [x] T020 [P] [US3] Update image implementations to preserve accessibility attributes with new paths
- [x] T021 [US3] Verify screen readers can access image alternative text after path changes
- [x] T022 [US3] Run accessibility tools to confirm no errors related to the image
- [x] T023 [US3] Validate that accessibility features work in both environments

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T024 [P] Update documentation to reflect correct image path practices
- [x] T025 [P] Review all image implementations for consistency
- [x] T026 Performance check: Verify page load times remain under 3 seconds (SC-004)
- [x] T027 Run quickstart.md validation to ensure deployment process works
- [x] T028 Verify all success criteria are met (SC-001, SC-002, SC-003, SC-004)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all implementation tasks for User Story 1 together:
Task: "Update hero section component to use correct path to robologo.png in book/src/components/"
Task: "Update card slides component to use correct path to robologo.png in book/src/components/"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence