import React from 'react';
import { AuthProvider } from '../contexts/AuthContext';
import AuthModal from '../components/AuthModal';
import ChatbotUI from '../components/ChatbotUI';


export default function Root({ children }) {
  return (
    <AuthProvider>
      {children}
      <AuthModal />
         <ChatbotUI/>
    </AuthProvider>
  );
}
