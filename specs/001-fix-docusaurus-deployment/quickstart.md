# Quickstart: Docusaurus GitHub Pages Deployment Fix

## Overview
This guide will help you fix the Docusaurus GitHub Pages deployment so that the full UI renders correctly instead of showing raw markdown.

## Prerequisites
- Node.js (v20+) installed locally
- Access to the GitHub repository
- Basic understanding of GitHub Actions

## Steps to Fix Deployment

### 1. Update GitHub Actions Workflow
Update `.github/workflows/deploy.yml` to reference the correct directory:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches:
      - main  # or gh-pages if deploying from that branch
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

concurrency:
  group: "pages"
  cancel-in-progress: false

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0

      - uses: actions/setup-node@v4
        with:
          node-version: 20
          cache: npm
          cache-dependency-path: frontend/package-lock.json  # Changed from book/package-lock.json

      - name: Install dependencies
        run: |
          cd frontend  # Changed from cd book
          npm ci

      - name: Build website
        run: |
          cd frontend  # Changed from cd book
          npm run build

      - name: Upload artifact
        uses: actions/upload-pages-artifact@v3
        with:
          path: frontend/build  # Changed from book/build

  deploy:
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    runs-on: ubuntu-latest
    needs: build
    steps:
      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
```

### 2. Alternative: Manual Deployment
If you prefer to manually update the gh-pages branch:

1. Build the site locally:
```bash
cd frontend
npm install
npm run build
```

2. Switch to gh-pages branch and replace contents with build output:
```bash
git checkout gh-pages
# Remove all current files
git rm -rf .
# Copy built files (contents of frontend/build/)
cp -r ../frontend/build/* .
# Commit and push
git add .
git commit -m "Update with built Docusaurus site"
git push origin gh-pages
```

### 3. Verify Configuration
Ensure `frontend/docusaurus.config.ts` has correct settings:
- `url: 'https://Muhammad-Shahr0z.github.io'`
- `baseUrl: '/Hackathon-2025-Robotics-ai-Book1/'`
- `organizationName: 'Muhammad-Shahr0z'`
- `projectName: 'Hackathon-2025-Robotics-ai-Book1'`

## Verification
After deployment:
1. Visit your GitHub Pages URL: https://muhammad-shahr0z.github.io/Hackathon-2025-Robotics-ai-Book1/
2. Verify that you see the complete Docusaurus UI with proper styling
3. Check that navigation works correctly
4. Confirm that CSS and JavaScript assets load without errors