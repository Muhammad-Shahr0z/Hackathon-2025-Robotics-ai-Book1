# Research: Urdu Translation Implementation

## Decision: Docusaurus Built-in i18n with RTL Support
**Rationale**: Using Docusaurus built-in internationalization features provides the most robust and maintainable solution for multilingual support. This approach follows Docusaurus best practices and ensures compatibility with future updates.

## Technical Approach
- Configure `docusaurus.config.js` with Urdu locale settings
- Set up `i18n/ur/` directory structure for Urdu translations
- Implement RTL (right-to-left) layout support for Urdu
- Maintain all existing English content without modification

## Implementation Details

### 1. Configuration Changes
- Update `i18n` section in `docusaurus.config.js` to include Urdu locale
- Configure locale labels: 'English' for English, 'اردو' for Urdu
- Set text direction: LTR for English, RTL for Urdu
- Add locale dropdown to navbar

### 2. Translation File Structure
- Mirror English documentation structure in `i18n/ur/docusaurus-plugin-content-docs/current/`
- Each English markdown file has corresponding Urdu translation
- Preserve code blocks in English (as per constitution principle)
- Translate text content and explanations only

### 3. RTL Support Implementation
- Docusaurus automatically applies RTL styling when locale direction is set to 'rtl'
- CSS will adapt for right-to-left reading flow
- Navigation and layout elements will mirror for Urdu users

### 4. URL Structure
- English: `/docs/intro`
- Urdu: `/ur/docs/intro`
- SEO-friendly with language-specific paths

## Alternatives Considered

### Alternative 1: Custom Translation System
- **Rejected**: Would require building custom infrastructure, not leveraging Docusaurus strengths
- **Issues**: More complex, harder to maintain, potential SEO problems

### Alternative 2: Client-Side Translation API
- **Rejected**: Would require external API calls, potential latency issues
- **Issues**: Less reliable, doesn't work offline, violates "no external API" requirement

### Alternative 3: Pre-built Translation Plugins
- **Rejected**: Less control over translation quality and RTL implementation
- **Issues**: Potential compatibility issues, vendor lock-in

## Dependencies and Best Practices

### Docusaurus i18n Best Practices
- Follow official Docusaurus i18n documentation
- Use proper locale codes ('ur' for Urdu)
- Maintain consistent file structure between locales
- Test with RTL language support

### RTL Implementation Best Practices
- Proper CSS direction property handling
- Text alignment adjustments
- Navigation flow reversal
- Icon and UI element positioning

## Risks and Mitigation

### Risk 1: Layout Issues in RTL Mode
- **Mitigation**: Thorough testing of all page layouts in Urdu
- **Verification**: Manual testing of all major page types

### Risk 2: Incomplete Translations
- **Mitigation**: Plan for gradual translation with fallback to English
- **Verification**: Ensure fallback mechanism works properly

### Risk 3: Performance Impact
- **Mitigation**: Optimize translation files and build process
- **Verification**: Monitor build times and page load speeds