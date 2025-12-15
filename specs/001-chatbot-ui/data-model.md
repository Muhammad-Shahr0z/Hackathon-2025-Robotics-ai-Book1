# Data Model: Chatbot UI Component

## Entities

### ChatMessage
- **id**: string (unique identifier for the message)
- **content**: string (the text content of the message)
- **sender**: 'user' | 'bot' (indicates whether the message was sent by user or bot)
- **timestamp**: Date (when the message was created/sent)

### ChatState
- **isOpen**: boolean (whether the chat window is currently open)
- **messages**: ChatMessage[] (array of all messages in the current conversation)
- **inputText**: string (the current value of the input field)
- **isLoading**: boolean (whether the bot is currently "typing"/processing)

## Relationships
- ChatState contains multiple ChatMessage instances in the messages array
- Each ChatMessage belongs to a single ChatState (through the messages array)

## Validation Rules
- ChatMessage.content must not be empty when sent by user
- ChatMessage.sender must be either 'user' or 'bot'
- ChatMessage.id must be unique within the conversation
- inputText in ChatState must be trimmed before sending

## State Transitions
- isOpen toggles between true/false when the chat icon is clicked
- isLoading transitions to true when user sends message, back to false when response is received
- Messages array grows as new messages are added to the conversation