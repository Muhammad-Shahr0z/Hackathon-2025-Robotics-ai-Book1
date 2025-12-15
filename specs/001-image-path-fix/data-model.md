# Data Model: GitHub Pages Image Path Fix

## Entities

### Image Resource
- **Name**: robologo.png
- **Location**: static/img/robologo.png
- **Format**: PNG image file
- **Purpose**: Logo image for the robotics textbook site
- **Accessibility**: Must include alt text for screen readers

### Path Configuration
- **Type**: String (file path)
- **Format**: Relative path from site root
- **Value**: /img/robologo.png
- **Validation**: Must exist in static directory and be accessible via GitHub Pages

## Relationships
- The Image Resource is referenced by multiple UI components (Hero section, Card slides)
- Path Configuration determines how the Image Resource is accessed across different environments