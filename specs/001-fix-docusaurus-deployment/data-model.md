# Data Model: Docusaurus GitHub Pages Deployment

## Entities

### GitHub Actions Workflow Configuration
- **Name**: deploy.yml
- **Location**: .github/workflows/deploy.yml
- **Fields**:
  - trigger_branches: [string array] - branches that trigger deployment
  - build_directory: [string] - directory containing Docusaurus source
  - cache_dependency_path: [string] - path to package-lock.json for caching
  - artifact_path: [string] - path to built static files
- **Relationships**: References Docusaurus source directory and build output

### Docusaurus Configuration
- **Name**: docusaurus.config.ts
- **Location**: frontend/docusaurus.config.ts
- **Fields**:
  - url: [string] - base URL for the site
  - baseUrl: [string] - path under which site is served
  - organizationName: [string] - GitHub organization/user name
  - projectName: [string] - GitHub repository name
- **Relationships**: Determines how assets are loaded and routed

### GitHub Pages Branch
- **Name**: gh-pages
- **Fields**:
  - content_type: [enum: source_code, static_build] - type of content stored
  - directory_structure: [string] - files and folders in the branch
- **Validation**: Must contain static build output, not source files

## State Transitions

### Workflow State Transition
- **From**: Incorrect workflow (references 'book/')
- **To**: Correct workflow (references 'frontend/')
- **Action**: Update workflow configuration
- **Validation**: Workflow successfully builds and deploys from correct directory

### Branch Content Transition
- **From**: gh-pages contains source code
- **To**: gh-pages contains built static files
- **Action**: Deploy built output to gh-pages branch
- **Validation**: GitHub Pages serves proper Docusaurus UI

## Validation Rules

1. GitHub Actions workflow must reference correct Docusaurus directory ('frontend/')
2. Build artifact path must point to correct build output ('frontend/build/')
3. gh-pages branch must contain only static files, not source code
4. Docusaurus configuration must match GitHub Pages URL structure
5. All assets (CSS, JS) must load correctly from the deployed site