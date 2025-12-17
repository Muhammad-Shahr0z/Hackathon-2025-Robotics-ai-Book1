# Research: Docusaurus Language Switch Fix

## Overview
This research document addresses the Docusaurus language switch issue where clicking the Urdu language button results in malformed URLs like `/ur/ur/ur/ur/`. The goal is to identify the root cause and determine the correct implementation approach.

## Root Cause Analysis

### 1. Docusaurus i18n Configuration Issues
The most likely cause of the repeated `/ur/` segments is incorrect i18n configuration in the `docusaurus.config.js` file. Common issues include:
- Incorrect `baseUrl` or `url` configuration
- Improper locale settings
- Incorrect usage of localeDropdown in navbar

### 2. Language Switching Mechanism
Docusaurus provides a built-in `localeDropdown` type for navbar items. If custom language switching logic was implemented instead of using the built-in mechanism, it could cause routing issues.

### 3. URL Generation Problems
The issue might stem from how Docusaurus generates URLs when switching between locales. If there are custom link components or manual URL manipulations, they might be causing the duplication.

## Docusaurus i18n Best Practices

### 1. Correct Configuration Structure
```js
module.exports = {
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
      },
      ur: {
        label: 'Urdu',
        direction: 'rtl',
      },
    },
  },
  themeConfig: {
    navbar: {
      items: [
        {
          type: 'localeDropdown',
          position: 'right',
        },
      ],
    },
  },
};
```

### 2. Directory Structure
The i18n directory structure should mirror the docs structure:
```
i18n/
├── ur/
│   ├── docusaurus-plugin-content-docs/
│   │   └── current/      # Urdu translations of docs
│   └── docusaurus-theme-classic/
│       └── navbar.json   # Urdu navbar translations
└── en/
    └── (default locale)
```

### 3. Language Switching Behavior
- Default language (English) should have URLs like `/`
- Urdu language should have URLs like `/ur/`
- Switching should be idempotent - clicking the same language multiple times should not change the URL
- Navigation between pages should preserve the language context

## Solution Approach

### 1. Verify Current Configuration
Check the existing `docusaurus.config.js` file for:
- Correct i18n configuration
- Proper localeDropdown usage in navbar
- Base URL settings

### 2. Check for Manual URL Manipulation
Look for any custom components or scripts that might be manually manipulating URLs when language switching occurs.

### 3. Validate Translation Files
Ensure the Urdu translation files exist in the correct directory structure and match the English documentation structure.

### 4. RTL Support
Implement proper RTL styling for Urdu content if not already done.

## Decision: Use Built-in Docusaurus i18n

**Rationale**: Docusaurus provides robust built-in internationalization support that properly handles routing and language switching. Using the built-in `localeDropdown` type ensures proper URL generation and prevents the routing issues currently being experienced.

**Alternatives Considered**:
1. Custom language switching logic - Rejected due to complexity and potential for routing bugs
2. Third-party i18n libraries - Rejected as it would conflict with Docusaurus's built-in system
3. Manual URL manipulation - Rejected as it's error-prone and not recommended by Docusaurus

## Recommended Implementation Steps

1. Ensure `docusaurus.config.js` has proper i18n configuration
2. Use `localeDropdown` type in navbar configuration
3. Verify i18n directory structure matches documentation
4. Test language switching behavior thoroughly
5. Implement RTL styling for Urdu content