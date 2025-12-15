# Research: GitHub Pages Image Path Fix

## Decision: Path Structure for Docusaurus Deployment
**Rationale**: The robologo.png image needs to be accessible in both local development (localhost:3000) and GitHub Pages deployment. Using a consistent relative path structure ensures compatibility across both environments.

**Alternatives considered**:
1. Absolute paths - Would require different paths for local vs deployed environments
2. Public folder approach - Docusaurus serves static files from `/static` directory
3. Asset import approach - Using React imports for image assets
4. Relative path from root - Using `/img/robologo.png` format

**Chosen approach**: Place the image in the `static/img/` directory and reference it using relative paths that work with Docusaurus's static asset handling.

## Decision: Implementation locations
**Rationale**: The image is used in multiple locations (hero section and card slides), so we need to identify all files that reference the image and update them consistently.

**Locations to update**:
- Hero section component
- Card slides components
- Any other components using robologo.png

## Decision: Testing approach
**Rationale**: Need to verify the fix works in both environments before deployment.

**Testing steps**:
1. Update paths locally and test with `npm run start`
2. Build the site locally with `npm run build` and test
3. Deploy to GitHub Pages and verify