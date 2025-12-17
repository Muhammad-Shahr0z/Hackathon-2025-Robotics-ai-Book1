# Quickstart: Docusaurus Language Switch Fix

## Overview
This quickstart guide provides the essential steps to implement the fix for the Docusaurus language switch issue that causes malformed URLs like `/ur/ur/ur/`.

## Prerequisites
- Node.js (v16 or higher)
- npm or yarn package manager
- Docusaurus v3.x installed
- Basic knowledge of Docusaurus configuration

## Step-by-Step Implementation

### 1. Verify Current Configuration
First, check your existing `docusaurus.config.js` file to identify the current i18n setup:

```bash
# Navigate to your Docusaurus project root
cd frontend  # or wherever your Docusaurus project is located

# View the current configuration
cat docusaurus.config.js
```

### 2. Update Docusaurus Configuration
Ensure your `docusaurus.config.js` includes proper i18n configuration:

```javascript
// docusaurus.config.js
module.exports = {
  // ... other configuration
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
      },
      ur: {
        label: 'Urdu',
        direction: 'rtl',
      },
    },
  },
  themeConfig: {
    // ... other theme config
    navbar: {
      // ... other navbar config
      items: [
        // ... other items
        {
          type: 'localeDropdown',
          position: 'right',
        },
      ],
    },
  },
};
```

### 3. Verify Translation Directory Structure
Ensure your i18n directory structure is correct:

```bash
# Create the Urdu translation directories if they don't exist
mkdir -p i18n/ur/docusaurus-plugin-content-docs/current
mkdir -p i18n/ur/docusaurus-theme-classic

# The structure should look like:
# i18n/
# └── ur/
#     └── docusaurus-plugin-content-docs/
#         └── current/
#             └── (Urdu translation files)
```

### 4. Add RTL CSS Support (Optional but Recommended)
Create or update your custom CSS to support RTL layout for Urdu:

```css
/* src/css/custom.css */
html[data-theme='ur'] {
  direction: rtl;
}

/* Or use CSS class-based approach */
[dir='rtl'] {
  direction: rtl;
  text-align: right;
}

[dir='rtl'] .navbar__items {
  flex-direction: row-reverse;
}

[dir='rtl'] .pagination-nav__link--next {
  flex-direction: row-reverse;
}
```

### 5. Test the Language Switching
After making the configuration changes:

```bash
# Start the development server
npm run start  # or yarn start

# Visit http://localhost:3000 and test language switching
```

## Common Issues and Solutions

### Issue: URLs still show duplicated locale segments
**Solution**: Ensure you're using the `localeDropdown` type and not custom URL manipulation code.

### Issue: Urdu content doesn't appear after switching
**Solution**: Verify that translation files exist in `i18n/ur/docusaurus-plugin-content-docs/current/` and mirror the structure of your `docs/` directory.

### Issue: RTL styling not working
**Solution**: Make sure you've added the RTL CSS rules and that your theme supports RTL direction.

## Verification Steps

1. **Language Switching Test**:
   - Click Urdu language button → URL should be `/ur/`
   - Click English language button → URL should be `/`
   - Click Urdu button multiple times → URL should remain `/ur/`

2. **Content Verification**:
   - Ensure all content appears in Urdu when language is switched
   - Verify that navigation and layout work properly

3. **RTL Layout**:
   - Text should flow from right to left in Urdu
   - Navigation elements should be properly aligned

## Next Steps
- Add complete Urdu translations for all documentation
- Test on different browsers and devices
- Deploy to production environment
- Monitor for any routing issues in production