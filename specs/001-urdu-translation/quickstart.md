# Quickstart: Urdu Translation Feature

## Overview
This guide provides a quick setup for implementing Urdu translation support in the Docusaurus-based Physical AI & Humanoid Robotics textbook. The feature enables multilingual accessibility by adding Urdu language support while preserving all existing English content.

## Prerequisites
- Node.js 18+ installed
- Docusaurus 3.x project already set up
- Access to English documentation files
- Understanding of RTL (right-to-left) layout concepts

## Step-by-Step Setup

### 1. Configure Docusaurus for Urdu Locale
Update your `docusaurus.config.js` file to include Urdu language support:

```javascript
module.exports = {
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: { label: 'English', direction: 'ltr' },
      ur: { label: 'اردو', direction: 'rtl' },
    },
  },
  navbar: {
    items: [
      { type: 'localeDropdown', position: 'right' },
      // ... other navbar items
    ],
  },
  // ... rest of your config
};
```

### 2. Create Urdu Translation Directory Structure
Create the following directory structure for Urdu translations:

```
i18n/
└── ur/
    └── docusaurus-plugin-content-docs/
        └── current/
            ├── module-1/
            ├── module-2/
            ├── module-3/
            └── module-4/
```

### 3. Add Urdu Translation Files
For each English markdown file in your `docs/` directory, create a corresponding Urdu translation in the `i18n/ur/docusaurus-plugin-content-docs/current/` directory.

Example:
- English: `docs/module-1/intro.md`
- Urdu: `i18n/ur/docusaurus-plugin-content-docs/current/module-1/intro.md`

### 4. Build and Run
Build your site with i18n support:

```bash
npm run build
# or for development
npm run start -- --locale ur
```

## Key Features
- **Language Toggle**: Users can switch between English and Urdu using the locale dropdown in the navbar
- **RTL Support**: Urdu content automatically displays with right-to-left layout
- **SEO-Friendly URLs**: English `/docs/intro` becomes `/ur/docs/intro` for Urdu
- **Preserved Content**: All English content remains unchanged
- **Code Preservation**: Code blocks remain in English as per constitution requirements

## Testing
1. Verify the language toggle appears in the navbar
2. Test switching between English and Urdu
3. Confirm RTL layout for Urdu content
4. Check that all links work correctly in both languages
5. Verify that code blocks remain in English

## Next Steps
1. Begin translating your documentation content into Urdu
2. Review translated content for accuracy
3. Test the complete user experience
4. Optimize for performance if needed