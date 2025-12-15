# Foundational Changes for UI Redesign

## CSS Custom Properties (Design Tokens) Implementation

The following design tokens need to be added to the custom.css file in the :root block:

### Primary Colors
- --color-primary-500: #0066CC
- --color-primary-600: #0099FF

### Secondary Colors
- --color-secondary-500: #6600CC
- --color-secondary-600: #9966FF

### Accent Colors
- --color-accent-500: #00FFFF
- --color-accent-600: #CC66FF

### Status Colors
- --color-success: #00CC66
- --color-warning: #FF9900
- --color-error: #FF3333

### Background Colors
- --color-background-default: #FFFFFF
- --color-background-surface: #F8F9FA

### Text Colors
- --color-text-primary: #1A1A1A
- --color-text-secondary: #4A4A4A

### Spacing Scale
- --spacing-xs: 4px
- --spacing-sm: 8px
- --spacing-md: 12px
- --spacing-base: 16px
- --spacing-lg: 24px
- --spacing-xl: 32px
- --spacing-xxl: 48px
- --spacing-xxxl: 64px

### Typography
- --font-family: 'Inter', 'Roboto', system-ui, -apple-system, sans-serif
- --font-size-h1: 32px
- --font-size-h2: 24px
- --font-size-h3: 20px
- --font-size-body: 16px
- --font-size-small: 14px
- --font-weight-bold: 700
- --font-weight-regular: 400

### Border Radius
- --border-radius-sm: 4px
- --border-radius-md: 8px
- --border-radius-lg: 12px
- --border-radius-pill: 9999px

### Shadows
- --shadow-subtle: 0 1px 2px rgba(0, 0, 0, 0.05)
- --shadow-medium: 0 4px 8px rgba(0, 0, 0, 0.1)
- --shadow-elevated: 0 8px 16px rgba(0, 0, 0, 0.15)

### Updated Color Palette
The primary color palette should be updated from the current blue (#2563eb) to the new futuristic tech theme colors:
- Primary should change to #0066CC with appropriate shades
- Secondary purple colors should be added (#6600CC, etc.)
- Accent colors should include neon blues and purples (#00FFFF, #CC66FF)