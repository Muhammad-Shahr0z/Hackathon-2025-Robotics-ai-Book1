# Quickstart: Chatbot UI Component

## Installation

The Chatbot UI component is a standalone React component that requires no additional dependencies beyond a standard React + TypeScript setup.

## Usage

1. Place the `ChatbotUI.tsx` file in your project's components directory
2. Import and render the component in your desired location:

```tsx
import ChatbotUI from './ChatbotUI';

function App() {
  return (
    <div className="App">
      {/* Your other components */}
      <ChatbotUI />
    </div>
  );
}
```

## Configuration

The component is self-contained with no external configuration needed. All state is managed internally.

## API Integration

To connect to a real backend:

1. Locate the `callApi` function in the component
2. Replace the mock implementation with your actual API call:

```tsx
const callApi = async (prompt: string): Promise<string> => {
  // Replace with your actual API endpoint
  const response = await fetch('/api/chat', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({ prompt }),
  });

  const data = await response.json();
  return data.response;
};
```

## Styling

The component uses CSS-in-JS for styling with a professional, minimal design:
- Blur effect: `backdrop-filter: blur(10px)`
- White background with transparency
- Soft borders and subtle shadows
- Rounded corners
- Responsive design for all screen sizes

## Key Features

- Fixed position chat icon at bottom-right corner
- Smooth slide-in/slide-out animation
- Message history display
- Animated dots loading indicator
- Responsive design for mobile/desktop
- TypeScript type safety
- Keyboard support (Enter to send)