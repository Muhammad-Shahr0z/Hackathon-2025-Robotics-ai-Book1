---
id: "001"
title: "create-chatbot-ui"
stage: "general"
date: "2025-12-15"
surface: "agent"
model: "claude-haiku-4-5-20251001"
feature: "none"
branch: "001-chatbot-ui"
user: "user"
command: "/sp.specify You are a senior Frontend Engineer and UI/UX specialist. Your task is to design and implement a **professional, clean, and modern Chatbot UI** using **React + TypeScript**."
labels: ["ui", "react", "typescript", "chatbot"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "ChatbotUI.tsx"
  - "specs/001-chatbot-ui/spec.md"
  - "specs/001-chatbot-ui/checklists/requirements.md"
tests: []
---

## PROMPT_TEXT

You are a senior Frontend Engineer and UI/UX specialist.

Your task is to design and implement a **professional, clean, and modern Chatbot UI** using **React + TypeScript**.

## Core Requirements

- Create **only one component** named **`ChatbotUI.tsx`**
- All chatbot-related logic, UI, and state must live **inside this single component**
- This component will later be imported and rendered on the **Home page**
- No backend is required initially (use simulated behavior)

---

## Position & Behavior

- The chatbot must be **fixed at the bottom-right corner** of the screen with a small margin
- When **closed**, only a small circular chat icon/button is visible
- When the icon is clicked:
  - A **side chat window opens smoothly** (slide / fade animation)
  - The window opens **from the right side**, not full screen
- When closed again, it collapses back into the icon

---

## Chat Window UI

- Design a **minimal, professional, non-colorful UI**
- Use:
  - **Blur + white background**
  - Soft borders
  - Subtle shadows
  - Rounded corners
- Avoid strong colors; keep the interface clean and corporate-looking

---

## Chat Functionality (Phase 1 – No Backend)

- User can type a question and send it
- When a message is sent:
  1. Show the user's message in the chat
  2. Display a **typing-style loading indicator** using animated dots (`...`)
     - No text like "typing…"
     - Dots animation only
  3. Wait **1 second**
  4. Show a **static mock response** from the bot

---

## Chat Functionality (Phase 2 – API Ready)

Inside the **same component**, structure the code so that:

- An API function exists that:
  - Accepts the user prompt as a string
  - Returns a string response
- When API call starts:
  - Show the **animated dots loading indicator** in the chat
- When API responds:
  - Replace the loader with the response text
- The API logic should be easily replaceable later with a real backend endpoint

---

## Technical Guidelines

- Use **React hooks only** (`useState`, `useEffect`, etc.)
- No external UI libraries unless absolutely necessary
- Clean, readable, production-quality code
- Use TypeScript types for messages and state
- Ensure smooth animations and proper spacing
- Component should be responsive and not break on smaller screens

---

## Deliverables

- `ChatbotUI.tsx` containing:
  - UI
  - State management
  - Mock logic
  - API-ready structure
- The component must be easily importable into the Home page
- Focus on **UX polish**, smooth interactions, and professional appearance

---

## Important Note

- **Do NOT remove, modify, or delete any existing files or content inside the `specs` folder**
- All old specs must remain intact

## RESPONSE_TEXT

I have successfully implemented the Chatbot UI component as requested. The implementation includes:

1. A `ChatbotUI.tsx` component with:
   - Fixed position at the bottom-right corner with a circular chat icon
   - Smooth slide-in/slide-out animation for the chat window
   - Professional UI with blur effects, white background, soft borders, subtle shadows, and rounded corners
   - Minimal, corporate styling without strong colors
   - Message sending and receiving functionality
   - Animated dots loading indicator that shows after sending a message
   - Mock response functionality with 1-second delay
   - API-ready structure that can be easily replaced with a real backend
   - Responsive design for different screen sizes
   - TypeScript interfaces for type safety
   - Keyboard support (Enter key to send)

2. A complete specification file (spec.md) that outlines:
   - User scenarios and testing requirements
   - Functional requirements
   - Success criteria
   - Edge cases

3. A quality checklist to validate the specification

The component is fully functional and meets all the requirements specified in the user's request. It uses React hooks only, implements proper TypeScript typing, and has a clean, professional appearance with smooth animations.

## OUTCOME

Successfully created a professional, clean, and modern Chatbot UI component that meets all specified requirements. The component is ready to be imported and rendered on the Home page.