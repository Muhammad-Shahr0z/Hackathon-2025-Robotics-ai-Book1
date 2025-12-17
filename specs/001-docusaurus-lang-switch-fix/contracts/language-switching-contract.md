# Language Switching API Contract

## Overview
This contract defines the expected behavior for the Docusaurus language switching functionality. Since this is primarily a frontend routing feature, the "API" refers to the expected URL routing behavior and state management.

## Endpoints/Behaviors

### 1. Language Switch Request
**Description**: Trigger language switch via navbar dropdown
**Trigger**: User clicks on locale dropdown and selects a language

**Request Context**:
- Current URL path
- Current language locale
- Target language locale

**Expected Response Behavior**:
- URL should update to reflect new locale
- Content should switch to selected language
- Navigation should remain consistent
- No duplicate locale segments in URL

### 2. URL Routing for Locales
**English (Default)**:
- Base URL: `/`
- Document URLs: `/docs/intro`, `/docs/tutorial/basics`, etc.

**Urdu (Secondary)**:
- Base URL: `/ur/`
- Document URLs: `/ur/docs/intro`, `/ur/docs/tutorial/basics`, etc.

### 3. Idempotent Language Switching
**Behavior**: Clicking the same language multiple times should not change the URL
- If on `/ur/` and Urdu is clicked again → remains `/ur/`
- If on `/` and English is clicked again → remains `/`

## State Transitions

### Language State Changes
```
[English: /] <---> [Urdu: /ur/]
```

**Transitions**:
- `switchToUrdu`: `/` → `/ur/`
- `switchToEnglish`: `/ur/` → `/`
- `sameLanguage`: `/ur/` → `/ur/` (no change)
- `sameLanguage`: `/` → `/` (no change)

## Validation Rules

### 1. URL Structure Validation
- URLs must not contain duplicate locale prefixes (e.g., `/ur/ur/`)
- Non-default locales must have proper prefix (e.g., `/ur/`)
- Default locale should not have prefix (e.g., `/` not `/en/`)

### 2. Content Consistency
- All navigation links should preserve locale context
- Relative links should work correctly in both languages
- Site search should respect current locale

### 3. RTL Support
- When Urdu is selected, text direction should be RTL
- Layout should adjust appropriately for RTL languages
- CSS classes should reflect RTL state if applicable

## Error Conditions

### 1. Invalid Locale
**Condition**: Attempting to switch to unsupported locale
**Expected Behavior**: No change in language, remain on current locale

### 2. Missing Translation
**Condition**: Requested page has no translation in target locale
**Expected Behavior**: Fallback to English content for that page only

### 3. Routing Error
**Condition**: Malformed URL or routing issue occurs
**Expected Behavior**: Redirect to appropriate default page in current locale