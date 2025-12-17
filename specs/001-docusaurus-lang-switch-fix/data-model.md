# Data Model: Docusaurus Language Switch Fix

## Overview
This document outlines the data structures and configurations needed for the Docusaurus language switch functionality. Since this is primarily a configuration and routing issue, the "data model" focuses on the configuration structures and file organization.

## Configuration Entities

### 1. i18n Configuration Object
The i18n configuration object in `docusaurus.config.js` defines the internationalization settings:

```javascript
{
  defaultLocale: string,        // Default language locale (e.g., 'en')
  locales: string[],            // Array of supported locales (e.g., ['en', 'ur'])
  localeConfigs: {              // Configuration for each locale
    [locale: string]: {
      label: string,            // Display name for the locale
      direction?: string,       // Text direction ('ltr' or 'rtl')
      htmlLang?: string,        // HTML lang attribute
      path?: string             // URL path prefix (defaults to locale code)
    }
  }
}
```

**Validation Rules**:
- `defaultLocale` must be present in `locales` array
- `locales` array must contain at least one locale
- Each locale in `localeConfigs` must match a locale in the `locales` array
- `direction` defaults to 'ltr' if not specified

### 2. Navbar Item Configuration
The navbar configuration for language switching:

```javascript
{
  type: 'localeDropdown',       // Fixed type for language switcher
  position: 'left' | 'right',   // Position in navbar
  dropdownItemsBefore: array,   // Additional items before locale options
  dropdownItemsAfter: array     // Additional items after locale options
}
```

### 3. Translation File Structure
The directory structure for translation files:

```
i18n/
└── [locale]/
    ├── docusaurus-plugin-content-docs/
    │   └── [version]/          # Usually 'current' for latest version
    │       └── [doc-files]     # Translated documentation files
    ├── docusaurus-plugin-content-blog/
    │   └── [blog-files]        # Translated blog files (if applicable)
    └── docusaurus-theme-classic/
        ├── navbar.json         # Navbar translations
        └── footer.json         # Footer translations
```

## State Entities

### 1. Current Locale State
Represents the currently active language:

```javascript
{
  currentLocale: string,        // The active locale code (e.g., 'en', 'ur')
  availableLocales: string[],   // List of all available locales
  isRTL: boolean                // Whether the current locale uses right-to-left text
}
```

### 2. URL Routing State
Represents the URL structure for different locales:

```javascript
{
  defaultLocalePath: string,    // Path for default locale (typically '/')
  localePrefix: string,         // Prefix for non-default locales (e.g., '/ur/')
  currentPath: string,          // Current URL path without locale prefix
  localizedPath: string         // Full path with locale prefix when applicable
}
```

## File Organization

### 1. Documentation Files
English documentation files are the source of truth:
```
docs/
├── intro.md
├── tutorial/
│   ├── basics.md
│   └── advanced.md
└── reference/
    └── api.md
```

### 2. Translation Mapping
Urdu translations must mirror the English structure:
```
i18n/
└── ur/
    └── docusaurus-plugin-content-docs/
        └── current/
            ├── intro.md
            ├── tutorial/
            │   ├── basics.md
            │   └── advanced.md
            └── reference/
                └── api.md
```

## Validation Rules

### 1. Locale Configuration Validation
- Each locale in the configuration must have corresponding translation files
- Default locale must be a valid, existing locale
- Locale codes must follow ISO 639-1 standard (2-letter codes)

### 2. Translation Completeness
- All English documentation files must have corresponding Urdu translations
- Missing translations should fall back to English content
- Navigation structure must be preserved across locales

### 3. Routing Validation
- URLs must not contain duplicate locale prefixes
- Language switching must be idempotent
- Relative links must work correctly across locales