---
id: 1
title: Urdu Translation Spec
stage: spec
date: 2025-12-17
surface: agent
model: haiku-4-5-20251001
feature: urdu-translation
branch: 001-urdu-translation
user: Claude-Code-User
command: /sp.specify
labels: [i18n, docusaurus, translation, urdu]
links:
  spec: specs/001-urdu-translation/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-urdu-translation/spec.md
 - specs/001-urdu-translation/checklists/requirements.md
tests:
 - specs/001-urdu-translation/spec.md
---

## Prompt

Project: Existing Docusaurus website (100% working English version)
Goal: Add a Urdu translation feature without breaking the existing English site.

1️⃣ Feature Overview

Add Urdu language support to the existing Docusaurus site.

Language toggle will be available in navbar:

Default: English

Click Urdu icon → entire site translates to Urdu

Click English icon → switch back to English

Existing English content must not be modified.

2️⃣ Functional Requirements

Internationalization (i18n) Setup

Use Docusaurus built-in i18n support.

docusaurus.config.js updated:

i18n: {
  defaultLocale: 'en',
  locales: ['en', 'ur'],
  localeConfigs: {
    en: { label: 'English', direction: 'ltr' },
    ur: { label: 'اردو', direction: 'rtl' },
  },
},


All existing English docs remain unchanged.

Urdu Content

Create Urdu translation files in:

i18n/ur/docusaurus-plugin-content-docs/current/


Each Markdown file mirrors English version but content in Urdu.

Optional: AI-assisted translation can be used for draft, but manual proofreading recommended.

Navbar Language Switcher

Add localeDropdown in navbar:

navbar: {
  items: [
    { type: 'localeDropdown', position: 'right' },
  ],
},


Users can toggle English ↔ Urdu.

RTL Support

Urdu pages should automatically use right-to-left layout (direction: 'rtl')

Ensure CSS, typography, and layout adapt for RTL without breaking English layout.

SEO & Links

English URL example: /docs/intro

Urdu URL example: /ur/docs/intro

Internal links between docs should respect current locale.

Default Language

English is the default locale.

On first load, site shows English unless user explicitly switches to Urdu.

3️⃣ Non-functional Requirements

No existing English content or functionality should break.

Translation feature should be lightweight (no external API required).

Maintain Docusaurus build and deployment workflow.

Works seamlessly with existing navbar, theme, and plugins.

4️⃣ Optional Enhancements

Add Urdu flag or icon next to language switcher for better UX.

Automatic fallback: if Urdu translation missing → show English.

5️⃣ Deliverables

Updated docusaurus.config.js with i18n.

Fully structured Urdu Markdown files (i18n/ur/...).

Working language toggle in navbar.

RTL-compliant layout for Urdu pages.

Verified English content remains intact.

## Response snapshot

Created feature specification for Urdu translation support on Docusaurus website. The specification includes:

- User scenarios for switching between English and Urdu languages
- Functional requirements for language switching, RTL layout, and content preservation
- Success criteria for measuring the feature's effectiveness
- Quality checklist to validate the specification

The spec is stored in specs/001-urdu-translation/spec.md with a quality checklist at specs/001-urdu-translation/checklists/requirements.md.

## Outcome

- ✅ Impact: Created complete feature specification for Urdu translation with user scenarios, requirements and success criteria
- 🧪 Tests: Specification validated against quality checklist
- 📁 Files: specs/001-urdu-translation/spec.md, specs/001-urdu-translation/checklists/requirements.md
- 🔁 Next prompts: Ready for planning phase with /sp.plan
- 🧠 Reflection: Specification addresses all requirements from user request with clear acceptance criteria

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All checklist items validated
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to planning phase