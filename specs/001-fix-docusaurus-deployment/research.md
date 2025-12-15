# Research: Docusaurus GitHub Pages Deployment Fix

## Issue Analysis

### Current State
- Repository: `Hackathon-2025-Robotics-ai-Book` (but deployed as `Hackathon-2025-Robotics-ai-Book1`)
- Docusaurus site is located in `frontend/` directory (not `book/` as workflow expects)
- GitHub Pages configured to use `gh-pages` branch
- Current `gh-pages` branch contains source code, not built static files
- Site shows raw markdown instead of Docusaurus UI

### Root Causes Identified

1. **Workflow Configuration Mismatch**:
   - GitHub Actions workflow (`/.github/workflows/deploy.yml`) expects Docusaurus in `book/` directory
   - Actual Docusaurus site is in `frontend/` directory
   - Workflow tries `cd book` but should be `cd frontend`

2. **Branch Deployment Strategy**:
   - Workflow triggers on `master` branch, but user needs `gh-pages` branch updated
   - `gh-pages` branch currently contains source files instead of build output

3. **Build Output Missing**:
   - `gh-pages` branch should contain the output of `npm run build`
   - Currently contains source code which GitHub Pages serves as raw content

### Technical Details

- **Configuration**: `frontend/docusaurus.config.ts` has correct `baseUrl: '/Hackathon-2025-Robotics-ai-Book1/'`
- **Build Command**: `npm run build` in `frontend/` directory
- **Output Path**: `frontend/build/` directory contains static files
- **GitHub Pages**: Serves content from `gh-pages` branch root

### Recommended Solution

1. **Fix GitHub Actions Workflow**:
   - Update `deploy.yml` to reference `frontend/` instead of `book/`
   - Change cache dependency path from `book/package-lock.json` to `frontend/package-lock.json`
   - Change build path from `book/build` to `frontend/build`

2. **Deployment Strategy**:
   - Either update workflow to trigger on `gh-pages` branch
   - Or use Docusaurus' built-in `deploy` command which pushes build output to `gh-pages`

3. **Alternative Manual Deployment**:
   - Build site locally: `cd frontend && npm run build`
   - Copy contents of `frontend/build/` to `gh-pages` branch
   - Ensure `gh-pages` branch contains only static files, not source code

### Decision: Update GitHub Actions workflow to properly deploy from frontend directory to gh-pages branch

### Rationale:
Using GitHub Actions provides automated deployment that ensures consistency. The workflow needs to be updated to match the actual project structure where Docusaurus is in the `frontend/` directory rather than a `book/` directory.

### Alternatives Considered:
1. **Manual deployment**: Requires manual build and push, error-prone
2. **Change directory structure**: Would require significant refactoring
3. **Use Docusaurus deploy command**: Would still need to fix configuration to point to `frontend/` directory