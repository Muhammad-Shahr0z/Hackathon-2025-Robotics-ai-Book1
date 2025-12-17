---
id: 1-docusaurus-lang-switch-fix
title: Docusaurus Language Switch Fix
stage: spec
date: 2025-12-17
surface: agent
model: claude-haiku-4-5-20251001
feature: docusaurus-lang-switch-fix
branch: 001-docusaurus-lang-switch-fix
user: default_user
command: /sp.specify
labels:
  - docusaurus
  - i18n
  - urdu-translation
  - routing
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - D:/Hackathon-2025-Robotics-ai-Book/specs/001-docusaurus-lang-switch-fix/spec.md
  - D:/Hackathon-25-Robotics-ai-Book/specs/001-docusaurus-lang-switch-fix/checklists/requirements.md
tests: []
prompt_text: "# Issue: Docusaurus Language Switch Causing /ur/ur/ur Routing & Page Not Found

## Project Context
- Project is built with **Docusaurus v3.x**
- English version of the site is **100% working**
- Docusaurus **i18n** is enabled to add **Urdu (RTL)** support
- No backend is used
- Local development URL:
  - English: `http://localhost:3001/`
  - Urdu (expected): `http://localhost:3001/ur/`

---

## Current Problem
1. When clicking the **Urdu language button** in the navbar:
   - The site redirects to an invalid route:
     ```
     /ur/ur/ur/ur/
     ```
   - This results in a **Page Not Found** error.

2. Each additional click on the Urdu button keeps appending `/ur` again:
/ur → /ur/ur → /ur/ur/ur → ...

markdown
Copy code

3. The same issue occurs when switching back to English.

4. Due to incorrect routing, translated pages do not load correctly and the site becomes unusable.

---

## Expected Behavior
- Default language must always be **English**
- When clicking **Urdu** once:
- Entire website (all docs, chapters, modules, UI text) should switch to **Urdu**
- URL should be:
 ```
 /ur/
 ```
- When clicking **English**:
- Site should switch back to English
- URL should return to:
 ```
 /
 ```
- Language switch must be **idempotent**:
- Re-clicking the same language must NOT modify the URL again
- No repeated `/ur/` segments should ever appear

---

## Requirements & Constraints
- Must use **Docusaurus built-in i18n only**
- Existing English docs and routes must **NOT be modified or broken**
- Urdu translation must reuse the same document structure as English
- Language switching must be handled via **localeDropdown**
- No custom routing hacks or manual URL manipulation
- Must fully support **RTL layout** for Urdu

---

## Current Configuration (Suspected Issues)
- Incorrect `baseUrl` or `url` configuration
- Manual links containing hardcoded `/ur`
- Incorrect use of `Link` instead of Docusaurus `@docusaurus/Link`
- Translation files missing or mismatched paths
- Navbar language switcher misconfiguration

---

## Required Fix
1. Correct `docusaurus.config.js`:
- Proper `i18n` setup
- Correct `url` and `baseUrl`
2. Ensure **no manual `/ur` prefixes** exist anywhere in:
- Markdown files
- Navbar links
- Custom components
3. Ensure Urdu translation files mirror English docs structure exactly:
docs/...
i18n/ur/docusaurus-plugin-content-docs/current/...

css
Copy code
4. Use only:
```js
{ type: 'localeDropdown' }
for language switching
5. Ensure repeated language switching does not mutate routes

Definition of Done
English loads by default

One click on Urdu → entire site in Urdu

One click on English → entire site in English

No /ur/ur/ur routing issues

No "Page Not Found" errors

Production & local build both pass successfully"
response_text: "Created specification for fixing the Docusaurus language switch issue that was causing malformed URLs like /ur/ur/ur/. The specification includes user scenarios, functional requirements, and success criteria focused on proper language switching behavior, idempotent operations, and RTL layout support."
outcome: "Specification created successfully for the Docusaurus language switch fix feature. The spec addresses the core routing issue where clicking the Urdu language button multiple times was causing URL duplication like /ur/ur/ur/. The specification includes all required sections with clear acceptance criteria and measurable outcomes."
evaluation: "The specification is complete and ready for the planning phase. All checklist items have been validated as complete, with no remaining clarifications needed."
---
