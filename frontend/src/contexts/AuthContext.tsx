import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import {
  saveAuthToStorage,
  getAuthFromStorage,
  removeAuthFromStorage,
  isStoredAuthValid,
  StoredAuthData
} from '@site/src/utils/localStorage';

export interface User {
  id: string;
  email: string;
  name?: string;
  first_name?: string;
  last_name?: string;
}

interface AuthContextType {
  user: User | null;
  setUser: (user: User | null) => void;
  setAuthWithToken: (userData: User, token: string, expiresAt: string) => void;
  isAuthModalOpen: boolean;
  openAuthModal: () => void;
  closeAuthModal: () => void;
  isLoading: boolean;
  checkSession: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

// Backend API URL - points to Better Auth backend
const getApiUrl = () => {
  if (typeof window === 'undefined') return 'http://localhost:8000';
  return window.location.hostname === 'localhost'
    ? 'http://localhost:8000'
    : 'https://hackathon-2025-robotics-ai-book1.vercel.app/'; // Update this to your deployed backend URL
};

export function AuthProvider({ children }: { children: ReactNode }) {
  const [user, setUserState] = useState<User | null>(null);

  // Custom setUser that also saves to local storage
  const setUser = (newUser: User | null) => {
    setUserState(newUser);

    if (newUser) {
      // Save to local storage for persistence
      const authData: StoredAuthData = {
        token: '', // We don't have the token here, will be handled in auth flows
        userId: newUser.id,
        email: newUser.email,
        firstName: newUser.first_name || newUser.name?.split(' ')[0] || '',
        lastName: newUser.last_name || (newUser.name?.split(' ').slice(1).join(' ') || ''),
        expiresAt: new Date(Date.now() + 30*24*60*60*1000).toISOString() // 30 days from now
      };
      saveAuthToStorage(authData);
    } else {
      // Clear local storage when user is set to null
      removeAuthFromStorage();
    }
  };

  // Function to handle full auth response (with token) from sign-in/sign-up
  const setAuthWithToken = (userData: User, token: string, expiresAt: string) => {
    setUserState(userData);

    // Save full auth data including token to local storage
    const authData: StoredAuthData = {
      token: token,
      userId: userData.id,
      email: userData.email,
      firstName: userData.first_name || userData.name?.split(' ')[0] || '',
      lastName: userData.last_name || (userData.name?.split(' ').slice(1).join(' ') || ''),
      expiresAt: expiresAt
    };
    saveAuthToStorage(authData);
  };
  const [isAuthModalOpen, setIsAuthModalOpen] = useState(false);
  const [isLoading, setIsLoading] = useState(true);

  const checkSession = async () => {
    try {
      // First, check if we have valid auth data in local storage
      if (isStoredAuthValid()) {
        const storedData = getAuthFromStorage();
        if (storedData) {
          // Create a user object from stored data
          const storedUser: User = {
            id: storedData.userId,
            email: storedData.email,
            first_name: storedData.firstName,
            last_name: storedData.lastName,
            name: `${storedData.firstName} ${storedData.lastName}`.trim()
          };
          setUser(storedUser);
          setIsLoading(false);
          return; // Use stored data, no need to call backend
        }
      }

      // If no valid stored data, check with backend
      const response = await fetch(`${getApiUrl()}/api/auth/get-session`, {
        credentials: 'include',
      });

      if (!response.ok) {
        // If backend session is invalid, clear any stored data
        removeAuthFromStorage();
        setUser(null);
        setIsLoading(false);
        return;
      }

      const data = await response.json();
      if (data && data.user) {
        setUser(data.user);
        // The get-session endpoint returns user info but not the token
        // So we'll update the stored token if it exists, but keep other data
        const storedData = getAuthFromStorage();
        if (storedData) {
          // Update user data but keep the existing token and expiration
          const updatedAuthData: StoredAuthData = {
            ...storedData,
            userId: data.user.id,
            email: data.user.email,
            firstName: data.user.first_name || storedData.firstName,
            lastName: data.user.last_name || storedData.lastName
          };
          saveAuthToStorage(updatedAuthData);
        } else {
          // If no stored data exists, create new entry (without token since get-session doesn't return it)
          const authData: StoredAuthData = {
            token: '', // Token is not returned by get-session endpoint
            userId: data.user.id,
            email: data.user.email,
            firstName: data.user.first_name || '',
            lastName: data.user.last_name || '',
            expiresAt: new Date(Date.now() + 30*24*60*60*1000).toISOString() // 30 days from now
          };
          saveAuthToStorage(authData);
        }
      } else {
        setUser(null);
      }
    } catch (error) {
      console.log('Session check failed:', error);
      setUser(null);
    } finally {
      setIsLoading(false);
    }
  };

  useEffect(() => {
    checkSession();

    // Add event listener for storage changes (for cross-tab sync)
    const handleStorageChange = (e: StorageEvent) => {
      if (e.key === 'better_auth_session') {
        // If auth data changed in another tab, update our state
        if (isStoredAuthValid()) {
          const storedData = getAuthFromStorage();
          if (storedData) {
            const storedUser: User = {
              id: storedData.userId,
              email: storedData.email,
              first_name: storedData.firstName,
              last_name: storedData.lastName,
              name: `${storedData.firstName} ${storedData.lastName}`.trim()
            };
            setUserState(storedUser);
          }
        } else {
          // If no valid stored data, clear user state
          setUserState(null);
        }
      }
    };

    window.addEventListener('storage', handleStorageChange);

    // Cleanup function
    return () => {
      window.removeEventListener('storage', handleStorageChange);
    };
  }, []);

  const openAuthModal = () => setIsAuthModalOpen(true);
  const closeAuthModal = () => setIsAuthModalOpen(false);

  return (
    <AuthContext.Provider
      value={{
        user,
        setUser,
        setAuthWithToken,
        isAuthModalOpen,
        openAuthModal,
        closeAuthModal,
        isLoading,
        checkSession,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
}

export function useAuth() {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}

