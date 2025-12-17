# Data Model: Urdu Translation Feature

## Entities

### 1. Language Locale
- **Name**: Language Locale
- **Description**: Represents the language and text direction settings
- **Fields**:
  - `code`: String (e.g., 'en', 'ur') - Unique identifier for the locale
  - `label`: String (e.g., 'English', 'اردو') - Display name for the locale
  - `direction`: String ('ltr', 'rtl') - Text direction for the locale
  - `isDefault`: Boolean - Whether this is the default locale

### 2. Translation File
- **Name**: Translation File
- **Description**: Markdown content file containing translated documentation
- **Fields**:
  - `id`: String - Unique identifier matching English source file
  - `locale`: String - Locale code ('ur' for Urdu translations)
  - `path`: String - File path in the i18n structure
  - `content`: String - Translated content in the target language
  - `originalFile`: String - Reference to the English source file
  - `lastModified`: Date - When the translation was last updated

### 3. Navigation State
- **Name**: Navigation State
- **Description**: Current language selection that persists across user session
- **Fields**:
  - `currentLocale`: String - Currently selected locale code
  - `previousLocale`: String - Previously selected locale code
  - `userSessionId`: String - Identifier for user session (if applicable)
  - `lastUpdated`: Date - When the locale was last changed

## Relationships
- One Language Locale contains many Translation Files
- One Translation File belongs to one Language Locale
- Navigation State references one Language Locale

## Validation Rules
- Locale code must be a valid IETF language tag
- Translation files must maintain the same structure as English originals
- Direction must be either 'ltr' or 'rtl'
- Default locale must be one of the configured locales

## State Transitions
- Navigation State can transition from one Locale to another based on user selection
- Translation availability may affect fallback behavior when a translation is missing