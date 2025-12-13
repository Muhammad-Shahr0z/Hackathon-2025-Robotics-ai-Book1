/**
 * Utility functions for managing local storage operations
 * Specifically designed for Better Auth session data
 */

// Key used to store auth session data in local storage
const AUTH_STORAGE_KEY = 'better_auth_session';

// Define the structure of auth data that will be stored
export interface StoredAuthData {
  token: string;
  userId: string;
  email: string;
  firstName: string;
  lastName: string;
  expiresAt: string; // ISO string format
}

/**
 * Save auth data to local storage
 */
export const saveAuthToStorage = (authData: StoredAuthData): void => {
  try {
    localStorage.setItem(AUTH_STORAGE_KEY, JSON.stringify(authData));
  } catch (error) {
    console.error('Error saving auth data to local storage:', error);
    // Handle storage quota exceeded or other storage errors
    if (error instanceof DOMException && error.name === 'QuotaExceededError') {
      console.error('Local storage quota exceeded');
    }
  }
};

/**
 * Retrieve auth data from local storage
 */
export const getAuthFromStorage = (): StoredAuthData | null => {
  try {
    const stored = localStorage.getItem(AUTH_STORAGE_KEY);
    if (!stored) {
      return null;
    }
    return JSON.parse(stored);
  } catch (error) {
    console.error('Error retrieving auth data from local storage:', error);
    return null;
  }
};

/**
 * Remove auth data from local storage
 */
export const removeAuthFromStorage = (): void => {
  try {
    localStorage.removeItem(AUTH_STORAGE_KEY);
  } catch (error) {
    console.error('Error removing auth data from local storage:', error);
  }
};

/**
 * Check if auth data exists in storage
 */
export const hasStoredAuth = (): boolean => {
  try {
    return localStorage.getItem(AUTH_STORAGE_KEY) !== null;
  } catch (error) {
    console.error('Error checking for stored auth data:', error);
    return false;
  }
};

/**
 * Validate if the stored token is still valid (not expired)
 */
export const isStoredAuthValid = (): boolean => {
  try {
    const storedData = getAuthFromStorage();
    if (!storedData) {
      return false;
    }

    // Check if the token expiration time is in the future
    const expirationTime = new Date(storedData.expiresAt).getTime();
    const currentTime = new Date().getTime();

    return expirationTime > currentTime;
  } catch (error) {
    console.error('Error validating stored auth data:', error);
    return false;
  }
};

/**
 * Clear all auth-related data from storage
 */
export const clearAuthStorage = (): void => {
  try {
    localStorage.removeItem(AUTH_STORAGE_KEY);
  } catch (error) {
    console.error('Error clearing auth storage:', error);
  }
};