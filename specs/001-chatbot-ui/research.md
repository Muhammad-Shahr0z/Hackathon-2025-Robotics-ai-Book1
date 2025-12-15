# Research: Chatbot UI Component

## Decision: Technology Stack
**Rationale**: Using React + TypeScript as specified in requirements. No external UI libraries to maintain minimal dependencies and follow the requirement of using React hooks only. CSS-in-JS for styling to keep everything in one file.

## Decision: State Management
**Rationale**: Using React's useState hook for managing component state as it's the standard approach for functional components and meets the requirement of using React hooks only.

## Decision: Animation Approach
**Rationale**: Using CSS animations for the slide-in/slide-out transitions and the animated dots loading indicator. This approach is lightweight, performant, and doesn't require additional dependencies.

## Decision: API Structure
**Rationale**: Created a mock API function (`callApi`) that follows the required signature (accepts string prompt, returns string response) and can be easily replaced with a real backend endpoint later.

## Decision: Responsive Design
**Rationale**: Using CSS media queries to ensure the chat component works well on different screen sizes, with special consideration for mobile devices where the virtual keyboard might affect layout.

## Alternatives Considered

### UI Libraries
- **Alternative**: Use Material-UI or Chakra UI
- **Rejected**: Violates requirement to not use external UI libraries unless absolutely necessary

### State Management
- **Alternative**: Use Redux or Context API
- **Rejected**: Overkill for this simple component; useState is sufficient and meets requirements

### Animation Libraries
- **Alternative**: Use Framer Motion or React Spring
- **Rejected**: Additional dependency not needed; CSS animations meet performance requirements

### Styling Approach
- **Alternative**: Styled-components or CSS modules
- **Rejected**: CSS-in-JS (via style jsx) keeps everything in one file as required