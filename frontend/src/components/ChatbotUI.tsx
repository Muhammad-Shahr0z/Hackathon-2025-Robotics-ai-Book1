import React, { useState, useRef, useEffect } from "react";

// Define TypeScript interfaces
interface ChatMessage {
  id: string;
  content: string;
  sender: "user" | "bot";
  timestamp: Date;
}

interface ChatState {
  isOpen: boolean;
  messages: ChatMessage[];
  inputText: string;
  isLoading: boolean;
}

const ChatbotUI: React.FC = () => {
  // Initialize state
  const [state, setState] = useState<ChatState>({
    isOpen: false,
    messages: [],
    inputText: "",
    isLoading: false,
  });

  // Reference for auto-scrolling to the latest message
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const chatWindowRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Function to toggle the chat window open/closed
  const toggleChat = () => {
    setState((prev) => ({
      ...prev,
      isOpen: !prev.isOpen,
    }));
  };

  // Function to clear all chat messages
  const clearChat = () => {
    setState((prev) => ({
      ...prev,
      messages: [],
    }));
  };

  // Function to handle input changes
  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setState((prev) => ({
      ...prev,
      inputText: e.target.value,
    }));
  };

  // Function to simulate API call (will be replaced with real API in the future)
  const callApi = async (prompt: string): Promise<string> => {
    // Simulate network delay
    await new Promise((resolve) => setTimeout(resolve, 1000));

    // Return a mock response
    return `I received your message: "${prompt}". This is a mock response from the chatbot.`;
  };

  // Function to handle sending a message
  const handleSendMessage = async () => {
    if (state.inputText.trim() === "") return;

    const userMessage: ChatMessage = {
      id: Date.now().toString(),
      content: state.inputText,
      sender: "user",
      timestamp: new Date(),
    };

    // Add user message to the chat
    setState((prev) => ({
      ...prev,
      messages: [...prev.messages, userMessage],
      inputText: "",
      isLoading: true,
    }));

    try {
      // Get response from API
      const response = await callApi(state.inputText);

      // Add bot response to the chat
      const botMessage: ChatMessage = {
        id: (Date.now() + 1).toString(),
        content: response,
        sender: "bot",
        timestamp: new Date(),
      };

      setState((prev) => ({
        ...prev,
        messages: [...prev.messages, botMessage],
        isLoading: false,
      }));
    } catch (error) {
      // Handle error case
      const errorMessage: ChatMessage = {
        id: (Date.now() + 1).toString(),
        content: "Sorry, I encountered an error processing your request.",
        sender: "bot",
        timestamp: new Date(),
      };

      setState((prev) => ({
        ...prev,
        messages: [...prev.messages, errorMessage],
        isLoading: false,
      }));
    }
  };

  // Function to handle sending message when Enter key is pressed
  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  // Auto-scroll to the bottom when new messages are added
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [state.messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (state.isOpen && inputRef.current) {
      setTimeout(() => {
        inputRef.current?.focus();
      }, 100);
    }
  }, [state.isOpen]);

  // Close chat when clicking outside on mobile
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (
        state.isOpen &&
        chatWindowRef.current &&
        !chatWindowRef.current.contains(event.target as Node) &&
        window.innerWidth <= 768
      ) {
        // Check if click is on toggle button (don't close in that case)
        const toggleButton = document.querySelector('.chat-toggle-button');
        if (toggleButton && !toggleButton.contains(event.target as Node)) {
          toggleChat();
        }
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [state.isOpen]);

  return (
    <>
      {/* Chat Window */}
      {state.isOpen && (
        <div 
          style={{ zIndex: 9999999 }} 
          className="chat-window"
          ref={chatWindowRef}
        >
     {/* New Blue Header with AI Icon */}
<div className="chat-header-blue">
  <div className="header-left">
    <div className="ai-robot-icon">
      <svg
        xmlns="http://www.w3.org/2000/svg"
        viewBox="0 0 24 24"
        fill="currentColor"
      >
        <path d="M13 2.05v2.02c3.95.49 7 3.85 7 7.93 0 3.21-1.92 6-4.72 7.28L13 17v5h5l-1.22-2.33C19.91 17.81 22 14.64 22 11c0-5.18-3.95-9.45-9-9.95zM11 2.05C6.05 2.55 2 6.82 2 11c0 3.64 2.09 6.81 5.22 8.67L6 22h5v-5l-2.28 1.28C5.92 17.99 4 15.2 4 12c0-4.08 3.05-7.44 7-7.93V2.05z" />
      </svg>
    </div>
    <div className="header-content">
      <h3>AI Assistant</h3>
      <p className="header-subtitle">Online • Ready to help</p>
    </div>
  </div>
  <div className="header-right">
    {/* Clear button with trash icon */}
    <button
      className="header-icon-button"
      onClick={clearChat}
      aria-label="Clear chat"
      title="Clear chat"
    >
      <svg
        xmlns="http://www.w3.org/2000/svg"
        viewBox="0 0 24 24"
        fill="currentColor"
        width="20"
        height="20"
      >
        <path d="M6 19c0 1.1.9 2 2 2h8c1.1 0 2-.9 2-2V7H6v12zM19 4h-3.5l-1-1h-5l-1 1H5v2h14V4z" />
      </svg>
    </button>
    
    {/* Close button with X icon */}
    <button
      className="header-icon-button"
      onClick={toggleChat}
      aria-label="Close chat"
      title="Close chat"
    >
      <svg
        xmlns="http://www.w3.org/2000/svg"
        viewBox="0 0 24 24"
        fill="currentColor"
        width="20"
        height="20"
      >
        <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z" />
      </svg>
    </button>
  </div>
</div>
          <div className="chat-messages">
            {/* Welcome Message */}
            {state.messages.length === 0 && (
              <div className="welcome-message">
                <div className="welcome-icon">
                  <svg
                    xmlns="http://www.w3.org/2000/svg"
                    viewBox="0 0 24 24"
                    fill="currentColor"
                  >
                    <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm0 3c1.66 0 3 1.34 3 3s-1.34 3-3 3-3-1.34-3-3 1.34-3 3-3zm0 14.2c-2.5 0-4.71-1.28-6-3.22.03-1.99 4-3.08 6-3.08 1.99 0 5.97 1.09 6 3.08-1.29 1.94-3.5 3.22-6 3.22z" />
                  </svg>
                </div>
                <h4>Hello! 👋</h4>
                <p>I'm your AI assistant. How can I help you today?</p>
                <div className="suggestions">
                  <button
                    className="suggestion-chip"
                    onClick={() => {
                      setState((prev) => ({
                        ...prev,
                        inputText: "What can you do?",
                      }));
                      inputRef.current?.focus();
                    }}
                  >
                    Capabilities
                  </button>

                  <button
                    className="suggestion-chip"
                    onClick={() => {
                      setState((prev) => ({
                        ...prev,
                        inputText: "Curious about robotics?",
                      }));
                      inputRef.current?.focus();
                    }}
                  >
                    Robotics
                  </button>

                  <button
                    className="suggestion-chip"
                    onClick={() => {
                      setState((prev) => ({
                        ...prev,
                        inputText: "Tell me about humanoid AI",
                      }));
                      inputRef.current?.focus();
                    }}
                  >
                    Humanoid AI
                  </button>
                </div>
              </div>
            )}

            {state.messages.map((message) => (
              <div
                key={message.id}
                className={`message ${message.sender}-message`}
              >
                {message.sender === "bot" && (
                  <div className="message-sender-icon">🤖</div>
                )}
                {message.sender === "user" && (
                  <div className="message-sender-icon">👤</div>
                )}
                <div className="message-content-wrapper">
                  <div className="message-content">
                    {message.content.split(' ').map((word, index, array) => (
                      <React.Fragment key={index}>
                        <span className="message-word">{word}</span>
                        {index < array.length - 1 && ' '}
                      </React.Fragment>
                    ))}
                  </div>
                  <div className="message-timestamp">
                    {message.timestamp.toLocaleTimeString([], {
                      hour: "2-digit",
                      minute: "2-digit",
                    })}
                  </div>
                </div>
              </div>
            ))}

            {state.isLoading && (
              <div className="message bot-message">
                <div className="message-sender-icon">🤖</div>
                <div className="message-content-wrapper">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          <div className="chat-input-area">
            <div className="input-wrapper">
              <input
                ref={inputRef}
                type="text"
                value={state.inputText}
                onChange={handleInputChange}
                onKeyPress={handleKeyPress}
                placeholder="Type your message here..."
                aria-label="Type your message"
                disabled={state.isLoading}
              />
              <div className="input-icons">
                <button className="icon-button" aria-label="Attach file">
                  <svg
                    xmlns="http://www.w3.org/2000/svg"
                    viewBox="0 0 24 24"
                    fill="currentColor"
                  >
                    <path d="M16.5 6v11.5c0 2.21-1.79 4-4 4s-4-1.79-4-4V5c0-1.38 1.12-2.5 2.5-2.5s2.5 1.12 2.5 2.5v10.5c0 .55-.45 1-1 1s-1-.45-1-1V6H10v9.5c0 1.38 1.12 2.5 2.5 2.5s2.5-1.12 2.5-2.5V5c0-2.21-1.79-4-4-4S7 2.79 7 5v12.5c0 3.04 2.46 5.5 5.5 5.5s5.5-2.46 5.5-5.5V6h-1.5z" />
                  </svg>
                </button>
              </div>
            </div>
            <button
              className="send-button"
              onClick={handleSendMessage}
              disabled={state.isLoading || state.inputText.trim() === ""}
              aria-label="Send message"
            >
              <svg
                xmlns="http://www.w3.org/2000/svg"
                viewBox="0 0 24 24"
                fill="currentColor"
              >
                <path d="M2.01 21L23 12 2.01 3 2 10l15 2-15 2z" />
              </svg>
            </button>
          </div>
        </div>
      )}

      {/* Chat Toggle Button */}
      <button
        className={`chat-toggle-button ${state.isOpen ? "hidden" : ""}`}
        onClick={toggleChat}
        aria-label={state.isOpen ? "Close chat" : "Open chat"}
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          viewBox="0 0 24 24"
          fill="currentColor"
        >
          <path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm0 14H5.17L4 17.17V4h16v12z" />
          <path d="M7 9h10v2H7zm0 4h7v2H7z" />
        </svg>
        <span className="notification-dot"></span>
      </button>

      {/* Add styles */}
      <style>{`
        /* Base Styles */
        * {
          box-sizing: border-box;
        }
        
        .chat-toggle-button {
          position: fixed;
          bottom: 30px;
          right: 30px;
          width: 70px;
          height: 70px;
          border-radius: 50%;
          background: linear-gradient(135deg, #4a6cf7 0%, #6a8cff 100%);
          color: white;
          border: none;
          font-size: 28px;
          cursor: pointer;
          box-shadow: 0 8px 32px rgba(74, 108, 247, 0.3);
          display: flex;
          align-items: center;
          justify-content: center;
          z-index: 1000;
          transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
        }

        .chat-toggle-button svg {
          width: 32px;
          height: 32px;
        }

        .chat-toggle-button:hover {
          transform: scale(1.1) rotate(5deg);
          box-shadow: 0 12px 40px rgba(74, 108, 247, 0.4);
        }

        .chat-toggle-button.hidden {
          opacity: 0;
          visibility: hidden;
          transform: scale(0.5);
        }

        .notification-dot {
          position: absolute;
          top: 12px;
          right: 12px;
          width: 12px;
          height: 12px;
          background: #ff4757;
          border-radius: 50%;
          border: 2px solid white;
          animation: pulse 2s infinite;
        }

        @keyframes pulse {
          0% {
            box-shadow: 0 0 0 0 rgba(255, 71, 87, 0.7);
          }
          70% {
            box-shadow: 0 0 0 10px rgba(255, 71, 87, 0);
          }
          100% {
            box-shadow: 0 0 0 0 rgba(255, 71, 87, 0);
          }
        }

        .chat-window {
          position: fixed;
          bottom: 30px;
          right: 30px;
          width: 420px;
          max-width: calc(100vw - 40px);
          height: 620px;
          max-height: calc(100vh - 100px);
          background: white;
          border-radius: 20px;
          box-shadow: 0 20px 60px rgba(0, 0, 0, 0.2);
          display: flex;
          flex-direction: column;
          z-index: 1001;
          overflow: hidden;
          animation: slideIn 0.4s cubic-bezier(0.34, 1.56, 0.64, 1);
        }

        @keyframes slideIn {
          from {
            opacity: 0;
            transform: translateY(100px) scale(0.9);
          }
          to {
            opacity: 1;
            transform: translateY(0) scale(1);
          }
        }

        /* New Blue Header */
        .chat-header-blue {
          background: linear-gradient(135deg, #4a6cf7 0%, #6a8cff 100%);
          padding: 20px 24px;
          display: flex;
          justify-content: space-between;
          align-items: center;
          color: white;
          min-height: 80px;
          flex-shrink: 0;
        }

        .header-left {
          display: flex;
          align-items: center;
          gap: 16px;
          min-width: 0;
        }

        .ai-robot-icon {
          width: 48px;
          height: 48px;
          background: rgba(255, 255, 255, 0.2);
          border-radius: 12px;
          display: flex;
          align-items: center;
          justify-content: center;
          backdrop-filter: blur(10px);
          flex-shrink: 0;
        }

        .ai-robot-icon svg {
          width: 28px;
          height: 28px;
        }

        .header-content {
          min-width: 0;
        }

        .header-content h3 {
          margin: 0;
          font-size: 20px;
          font-weight: 600;
          letter-spacing: -0.5px;
          white-space: nowrap;
          overflow: hidden;
          text-overflow: ellipsis;
        }

        .header-subtitle {
          margin: 4px 0 0 0;
          font-size: 13px;
          opacity: 0.9;
          display: flex;
          align-items: center;
          gap: 4px;
          white-space: nowrap;
          overflow: hidden;
          text-overflow: ellipsis;
        }

        .header-subtitle:before {
          content: '';
          width: 8px;
          height: 8px;
          background: #4cd964;
          border-radius: 50%;
          display: inline-block;
          flex-shrink: 0;
        }

        .header-right {
          display: flex;
          gap: 8px;
          flex-shrink: 0;
        }

        .header-icon-button {
          background: rgba(255, 255, 255, 0.2);
          border: none;
          width: 36px;
          height: 36px;
          border-radius: 10px;
          display: flex;
          align-items: center;
          justify-content: center;
          cursor: pointer;
          color: white;
          transition: all 0.2s ease;
          flex-shrink: 0;
        }

        .header-icon-button:hover {
          background: rgba(255, 255, 255, 0.3);
          transform: scale(1.05);
        }

        .header-icon-button.close:hover {
          background: rgba(255, 71, 87, 0.3);
        }

        .header-icon-button svg {
          width: 20px;
          height: 20px;
        }

        .chat-messages {
          flex: 1;
          padding: 24px;
          overflow-y: auto;
          display: flex;
          flex-direction: column;
          gap: 20px;
          background: #f8fafd;
          min-height: 0;
        }

        /* Welcome Message */
        .welcome-message {
          background: white;
          border-radius: 16px;
          padding: 28px;
          text-align: center;
          box-shadow: 0 4px 20px rgba(0, 0, 0, 0.05);
          margin-bottom: 20px;
          animation: fadeInUp 0.5s ease;
        }

        .welcome-icon {
          width: 64px;
          height: 64px;
          background: linear-gradient(135deg, #4a6cf7 0%, #6a8cff 100%);
          border-radius: 50%;
          display: flex;
          align-items: center;
          justify-content: center;
          margin: 0 auto 20px;
        }

        .welcome-icon svg {
          width: 32px;
          height: 32px;
          color: white;
        }

        .welcome-message h4 {
          margin: 0 0 8px 0;
          font-size: 22px;
          color: #2d3436;
          word-break: break-word;
        }

        .welcome-message p {
          margin: 0 0 24px 0;
          color: #636e72;
          font-size: 15px;
          line-height: 1.5;
          word-break: break-word;
        }

        .suggestions {
          display: flex;
          flex-wrap: wrap;
          gap: 10px;
          justify-content: center;
        }

        .suggestion-chip {
          background: #f1f5ff;
          border: 1px solid #e0e7ff;
          border-radius: 20px;
          padding: 10px 18px;
          font-size: 13px;
          color: #4a6cf7;
          cursor: pointer;
          transition: all 0.2s ease;
          white-space: nowrap;
        }

        .suggestion-chip:hover {
          background: #4a6cf7;
          color: white;
          transform: translateY(-1px);
        }

        .message {
          display: flex;
          gap: 12px;
          max-width: 100%;
          animation: fadeIn 0.3s ease;
          word-wrap: break-word;
          overflow-wrap: break-word;
        }

        @keyframes fadeIn {
          from {
            opacity: 0;
            transform: translateY(10px);
          }
          to {
            opacity: 1;
            transform: translateY(0);
          }
        }

        .user-message {
          align-self: flex-end;
          flex-direction: row-reverse;
        }

        .bot-message {
          align-self: flex-start;
        }

        .message-sender-icon {
          width: 36px;
          height: 36px;
          background: linear-gradient(135deg, #4a6cf7 0%, #6a8cff 100%);
          border-radius: 10px;
          display: flex;
          align-items: center;
          justify-content: center;
          color: white;
          font-size: 18px;
          flex-shrink: 0;
        }

        .user-message .message-sender-icon {
          background: #10b981;
        }

        .message-content-wrapper {
          max-width: calc(100% - 48px);
          min-width: 0;
        }

        .message-content {
          padding: 14px 18px;
          border-radius: 18px;
          font-size: 14.5px;
          line-height: 1.5;
          position: relative;
          word-break: break-word;
          overflow-wrap: break-word;
          white-space: pre-wrap;
        }

        .bot-message .message-content {
          background: white;
          color: #2d3436;
          border-bottom-left-radius: 4px;
          box-shadow: 0 2px 12px rgba(0, 0, 0, 0.05);
        }

        .user-message .message-content {
          background: linear-gradient(135deg, #4a6cf7 0%, #6a8cff 100%);
          color: white;
          border-bottom-right-radius: 4px;
        }

        /* Word-by-word styling for better text wrapping */
        .message-word {
          display: inline-block;
          min-width: 0;
          word-break: break-word;
        }

        .message-timestamp {
          font-size: 11px;
          color: #a0a0a0;
          margin-top: 6px;
          text-align: left;
          white-space: nowrap;
        }

        .user-message .message-timestamp {
          text-align: right;
        }

        .typing-indicator {
          display: flex;
          align-items: center;
          padding: 8px 0;
        }

        .typing-indicator span {
          height: 8px;
          width: 8px;
          background: #b8b8b8;
          border-radius: 50%;
          display: inline-block;
          margin: 0 2px;
          animation: typing 1.4s infinite ease-in-out;
        }

        .typing-indicator span:nth-child(1) {
          animation-delay: 0s;
        }

        .typing-indicator span:nth-child(2) {
          animation-delay: 0.2s;
        }

        .typing-indicator span:nth-child(3) {
          animation-delay: 0.4s;
        }

        @keyframes typing {
          0%, 60%, 100% {
            transform: translateY(0);
          }
          30% {
            transform: translateY(-5px);
          }
        }

        .chat-input-area {
          padding: 20px;
          background: white;
          border-top: 1px solid #f0f0f0;
          display: flex;
          gap: 12px;
          align-items: center;
          flex-shrink: 0;
        }

        .input-wrapper {
          flex: 1;
          position: relative;
          min-width: 0;
        }

        .chat-input-area input {
          width: 100%;
          padding: 16px 52px 16px 20px;
          border: 2px solid #e8e8e8;
          border-radius: 16px;
          font-size: 14.5px;
          outline: none;
          transition: all 0.3s ease;
          background: #fafafa;
          min-width: 0;
        }

        .chat-input-area input:focus {
          border-color: #4a6cf7;
          background: white;
          box-shadow: 0 0 0 4px rgba(74, 108, 247, 0.1);
        }

        .chat-input-area input::placeholder {
          color: #a0a0a0;
        }

        .input-icons {
          position: absolute;
          right: 12px;
          top: 50%;
          transform: translateY(-50%);
          display: flex;
          gap: 8px;
        }

        .icon-button {
          background: none;
          border: none;
          width: 32px;
          height: 32px;
          border-radius: 8px;
          display: flex;
          align-items: center;
          justify-content: center;
          cursor: pointer;
          color: #666;
          transition: all 0.2s ease;
          flex-shrink: 0;
        }

        .icon-button:hover {
          background: #f0f0f0;
          color: #4a6cf7;
        }

        .icon-button svg {
          width: 18px;
          height: 18px;
        }

        .send-button {
          width: 52px;
          height: 52px;
          background: linear-gradient(135deg, #4a6cf7 0%, #6a8cff 100%);
          border: none;
          border-radius: 16px;
          display: flex;
          align-items: center;
          justify-content: center;
          cursor: pointer;
          color: white;
          transition: all 0.3s ease;
          flex-shrink: 0;
        }

        .send-button:hover:not(:disabled) {
          transform: scale(1.05);
          box-shadow: 0 8px 20px rgba(74, 108, 247, 0.3);
        }

        .send-button:disabled {
          background: #cccccc;
          cursor: not-allowed;
          transform: none;
        }

        .send-button svg {
          width: 22px;
          height: 22px;
        }

        /* Scrollbar Styling */
        .chat-messages::-webkit-scrollbar {
          width: 6px;
        }

        .chat-messages::-webkit-scrollbar-track {
          background: transparent;
        }

        .chat-messages::-webkit-scrollbar-thumb {
          background: rgba(0, 0, 0, 0.1);
          border-radius: 3px;
        }

        .chat-messages::-webkit-scrollbar-thumb:hover {
          background: rgba(0, 0, 0, 0.2);
        }

        /* Responsive design */
        
        /* Large Tablets and Small Laptops */
        @media (max-width: 1024px) {
          .chat-window {
            width: 380px;
          }
          
          .message {
            max-width: 90%;
          }
        }
        
        /* Tablets */
        @media (max-width: 768px) {
          .chat-window {
            width: 350px;
            right: 20px;
            bottom: 20px;
          }
          
          .chat-toggle-button {
            bottom: 20px;
            right: 20px;
            width: 60px;
            height: 60px;
          }
          
          .header-content h3 {
            font-size: 18px;
          }
          
          .ai-robot-icon {
            width: 40px;
            height: 40px;
          }
          
          .ai-robot-icon svg {
            width: 24px;
            height: 24px;
          }
        }
        
        /* Mobile Phones */
        @media (max-width: 480px) {
          .chat-window {
            width: calc(100vw - 40px);
            height: 70vh;
            bottom: 20px;
            right: 20px;
            left: 20px;
            margin: 0;
            max-height: 80vh;
          }
          
          .chat-messages {
            padding: 16px;
          }
          
          .chat-input-area {
            padding: 16px;
          }
          
          .chat-header-blue {
            padding: 16px 20px;
            min-height: 70px;
          }
          
          .header-left {
            gap: 12px;
          }
          
          .ai-robot-icon {
            width: 36px;
            height: 36px;
          }
          
          .ai-robot-icon svg {
            width: 20px;
            height: 20px;
          }
          
          .header-content h3 {
            font-size: 16px;
          }
          
          .header-subtitle {
            font-size: 11px;
          }
          
          .header-icon-button {
            width: 32px;
            height: 32px;
          }
          
          .header-icon-button svg {
            width: 18px;
            height: 18px;
          }
          
          .message {
            max-width: 95%;
            gap: 8px;
          }
          
          .message-sender-icon {
            width: 32px;
            height: 32px;
            font-size: 16px;
          }
          
          .message-content {
            padding: 12px 16px;
            font-size: 14px;
          }
          
          .welcome-message {
            padding: 20px;
          }
          
          .welcome-message h4 {
            font-size: 18px;
          }
          
          .welcome-message p {
            font-size: 14px;
          }
          
          .suggestion-chip {
            padding: 8px 14px;
            font-size: 12px;
          }
          
          .chat-input-area input {
            padding: 14px 48px 14px 16px;
            font-size: 14px;
          }
          
          .send-button {
            width: 48px;
            height: 48px;
          }
          
          .icon-button {
            width: 28px;
            height: 28px;
          }
          
          .icon-button svg {
            width: 16px;
            height: 16px;
          }
          
          .send-button svg {
            width: 20px;
            height: 20px;
          }
        }
        
        /* Very Small Phones */
        @media (max-width: 360px) {
          .chat-window {
            width: calc(100vw - 30px);
            right: 15px;
            left: 15px;
            bottom: 15px;
          }
          
          .header-left {
            gap: 8px;
          }
          
          .ai-robot-icon {
            width: 32px;
            height: 32px;
          }
          
          .header-content h3 {
            font-size: 15px;
          }
          
          .suggestions {
            gap: 6px;
          }
          
          .suggestion-chip {
            padding: 6px 12px;
            font-size: 11px;
          }
        }

        @keyframes fadeInUp {
          from {
            opacity: 0;
            transform: translateY(20px);
          }
          to {
            opacity: 1;
            transform: translateY(0);
          }
        }
        
        /* Landscape Mode for Mobile */
        @media (max-height: 600px) and (orientation: landscape) {
          .chat-window {
            height: 80vh;
            max-height: 80vh;
          }
          
          .chat-header-blue {
            padding: 12px 16px;
            min-height: 60px;
          }
          
          .chat-messages {
            padding: 12px;
          }
        }
      `}</style>
    </>
  );
};

export default ChatbotUI;