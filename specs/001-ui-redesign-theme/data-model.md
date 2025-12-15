# Data Model for UI Redesign and Theme Enhancement

## Overview
This document outlines the data models relevant to the UI redesign and theme enhancement for the AI and robotics learning platform. The focus is on the data structures that support the theme system and user interface components.

## Theme Data Model

### ThemeConfiguration
Represents a complete theme configuration with all design tokens and settings.

```
ThemeConfiguration {
  id: string (primary key)
  name: string (e.g., "Default Tech Theme", "Dark Mode", "High Contrast")
  description: string
  primaryColors: ColorPalette
  secondaryColors: ColorPalette
  accentColors: ColorPalette
  backgroundColors: ColorPalette
  textColors: ColorPalette
  spacingScale: SpacingScale
  typography: TypographySettings
  borderRadius: BorderRadiusSettings
  shadows: ShadowSettings
  createdAt: timestamp
  updatedAt: timestamp
}
```

### ColorPalette
Contains color values for different purposes within the theme.

```
ColorPalette {
  primary: string[] (array of primary color hex values)
  secondary: string[] (array of secondary color hex values)
  accent: string[] (array of accent color hex values)
  success: string (success state color)
  warning: string (warning state color)
  error: string (error state color)
  background: string[] (background color options)
  surface: string[] (surface color options)
  text: string[] (text color options)
}
```

### SpacingScale
Defines the spacing system used throughout the application.

```
SpacingScale {
  xs: number (4px)
  sm: number (8px)
  md: number (12px)
  base: number (16px)
  lg: number (24px)
  xl: number (32px)
  xxl: number (48px)
  xxxl: number (64px)
}
```

### TypographySettings
Defines font families, sizes, and styles for different text elements.

```
TypographySettings {
  fontFamily: string (e.g., "Inter", "Roboto", "system-ui")
  heading: {
    h1: {
      size: number (32px)
      weight: string ("bold")
      lineHeight: number
    }
    h2: {
      size: number (24px)
      weight: string ("bold")
      lineHeight: number
    }
    h3: {
      size: number (20px)
      weight: string ("bold")
      lineHeight: number
    }
  }
  body: {
    size: number (16px)
    weight: string ("regular")
    lineHeight: number
  }
  small: {
    size: number (14px)
    weight: string ("regular")
    lineHeight: number
  }
}
```

### BorderRadiusSettings
Defines border radius values for different components.

```
BorderRadiusSettings {
  small: number (4px)
  medium: number (8px)
  large: number (12px)
  pill: number (9999px)
}
```

### ShadowSettings
Defines shadow values for different depth levels.

```
ShadowSettings {
  subtle: string ("0 1px 2px rgba(0,0,0,0.05)")
  medium: string ("0 4px 8px rgba(0,0,0,0.1)")
  elevated: string ("0 8px 16px rgba(0,0,0,0.15)")
}
```

## User Preference Data Model

### UserThemePreference
Stores user-specific theme preferences.

```
UserThemePreference {
  userId: string (foreign key reference to user)
  themeId: string (foreign key reference to ThemeConfiguration)
  isDefault: boolean (whether this is the default theme)
  createdAt: timestamp
  updatedAt: timestamp
}
```

## Component State Data Models

### ButtonState
Represents the state of a button component.

```
ButtonState {
  variant: string ("primary", "secondary", "tertiary", "danger", "success")
  size: string ("small", "medium", "large")
  disabled: boolean
  loading: boolean
  fullWidth: boolean
  hover: boolean
  active: boolean
  focus: boolean
}
```

### CardState
Represents the state of a card component.

```
CardState {
  variant: string ("default", "elevated", "outlined")
  interactive: boolean
  hover: boolean
  active: boolean
  padding: string (e.g., "sm", "md", "lg")
}
```

### ModalState
Represents the state of a modal component.

```
ModalState {
  isOpen: boolean
  size: string ("sm", "md", "lg", "xl", "full")
  backdrop: boolean
  closable: boolean
  animation: string ("fade", "slide", "zoom")
}
```

### FormFieldState
Represents the state of a form field component.

```
FormFieldState {
  value: string | number | boolean
  error: string | null
  touched: boolean
  disabled: boolean
  required: boolean
  validationStatus: string ("valid", "invalid", "pending")
}
```

## Validation Rules

### ThemeConfiguration Validation
- name must be unique
- all color values must be valid hex codes or CSS color names
- spacing values must be positive numbers
- typography sizes must be positive numbers
- all required fields must be present

### ColorPalette Validation
- All color values must follow the format #RRGGBB or be valid CSS color names
- Primary, secondary, and accent color arrays must contain at least one color
- Background and text colors must meet accessibility contrast ratios (WCAG 2.1 AA)

### UserThemePreference Validation
- Each user can only have one default theme
- themeId must reference an existing ThemeConfiguration
- userId must reference an existing user

## State Transitions

### Theme Switching Flow
1. User selects theme
2. System validates theme exists
3. System updates UserThemePreference
4. System applies new CSS custom properties
5. Components re-render with new theme values

### Component State Transitions
- Button: enabled → hover → active → disabled (or similar variations)
- Form Field: untouched → touched → valid/invalid → submitted
- Modal: closed → opening → open → closing → closed

## Relationships

### ThemeConfiguration ↔ UserThemePreference
- One ThemeConfiguration can be preferred by many users
- One User can have one preferred theme
- Relationship: One-to-Many (ThemeConfiguration to UserThemePreference)