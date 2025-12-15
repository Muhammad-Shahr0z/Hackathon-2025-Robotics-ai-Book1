import React, { useState } from 'react';
import styles from './AuthModal.module.css';
import { useAuth, User } from '@site/src/contexts/AuthContext';
import { Lock, UserPlus, User as UserIcon, Mail, Key, Briefcase, Code, Target, LogOut, CheckCircle, XCircle, Info, Eye, EyeOff, ArrowRight, Sparkles, Shield } from 'lucide-react';

interface AuthModalProps {
  isOpen?: boolean;
  onClose?: () => void;
  onAuthSuccess?: (user: User) => void;
}

// Backend API URL - points to Better Auth backend
const getApiUrl = () => {
 // Update this to your deployed backend URL
  return 'https://hackathon-2025-robotics-ai-book1-sr.vercel.app';
          // 'http://localhost:8000';
};

export default function AuthModal({ isOpen: propIsOpen, onClose: propOnClose, onAuthSuccess }: AuthModalProps) {
  const { user, setUser, setAuthWithToken, isAuthModalOpen, closeAuthModal, checkSession } = useAuth();

  // Support both controlled (props) and context-based usage
  const isOpen = propIsOpen !== undefined ? propIsOpen : isAuthModalOpen;
  const onClose = propOnClose || closeAuthModal;

  const [mode, setMode] = useState<'signin' | 'signup'>('signin');
  const [loading, setLoading] = useState(false);
  const [message, setMessage] = useState('');
  const [messageType, setMessageType] = useState<'success' | 'error' | 'info'>('info');
  const [showPassword, setShowPassword] = useState(false);

  // Form states - Basic Info
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');

  // Removed background/profile preference fields for minimal signup

  const API_URL = getApiUrl();

  const showMessage = (msg: string, type: 'success' | 'error' | 'info') => {
    setMessage(msg);
    setMessageType(type);
  };

  // Signup now uses a single compact form (name, email, password)

  const handleSignUp = async (e: React.FormEvent) => {
    e.preventDefault();

    // Validate minimum password length
    if (password.length < 6) {
      showMessage('Password must be at least 6 characters long', 'error');
      return;
    }

    setLoading(true);

    try {
      console.log('Attempting sign up to:', `${API_URL}/api/auth/sign-up/email`);
      const response = await fetch(`${API_URL}/api/auth/sign-up/email`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({
            email,
            password,
            name
          }),
      });

      console.log('Sign up response status:', response.status);

      if (!response.ok) {
        const errorText = await response.text();
        console.error('Sign up error:', errorText);
        throw new Error(errorText || 'Sign up failed');
      }

      let data;
      try {
        const responseText = await response.text();
        console.log('Response text:', responseText.substring(0, 500)); // Log first 500 chars
        data = JSON.parse(responseText);
      } catch (parseError) {
        console.error('JSON parse error:', parseError);
        // If JSON parsing fails but response was 200, consider it a success
        data = { user: { email } };
      }

      if (response.ok) {
        showMessage('Account created! Setting up your profile...', 'success');

        // Handle the full auth response with token
        if (data.session && data.session.token && data.session.expires_at) {
          setAuthWithToken(data.user, data.session.token, data.session.expires_at);
        } else {
          // Fallback to setUser if token is not in response
          setUser(data.user);
        }

        // Profile creation with extra background fields removed for minimal signup

        setTimeout(() => {
          onAuthSuccess?.(data.user);
          onClose();
        }, 600);
      } else {
        showMessage(`${data.message || 'Sign up failed'}`, 'error');
      }
    } catch (error) {
      showMessage(`Error: ${error instanceof Error ? error.message : 'Unknown error'}`, 'error');
    } finally {
      setLoading(false);
    }
  };

  const handleSignIn = async (e: React.FormEvent) => {
    e.preventDefault();

    // Validate minimum password length
    if (password.length < 6) {
      showMessage('Password must be at least 6 characters long', 'error');
      return;
    }

    setLoading(true);

    try {
      console.log('Attempting sign in to:', `${API_URL}/api/auth/sign-in/email`);
      const response = await fetch(`${API_URL}/api/auth/sign-in/email`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({ email, password }),
      });

      console.log('Sign in response status:', response.status);

      if (!response.ok) {
        const errorText = await response.text();
        console.error('Sign in error:', errorText);
        throw new Error(errorText || 'Sign in failed');
      }

      const data = await response.json();

      if (response.ok) {
        showMessage('Signed in successfully!', 'success');

        // Handle the full auth response with token
        if (data.session && data.session.token && data.session.expires_at) {
          setAuthWithToken(data.user, data.session.token, data.session.expires_at);
        } else {
          // Fallback to setUser if token is not in response
          setUser(data.user);
        }

        setTimeout(() => {
          onAuthSuccess?.(data.user);
          onClose();
        }, 1000);
      } else {
        showMessage(`${data.message || 'Sign in failed'}`, 'error');
      }
    } catch (error) {
      showMessage(`Error: ${error instanceof Error ? error.message : 'Unknown error'}`, 'error');
    } finally {
      setLoading(false);
    }
  };

  const handleSignOut = async () => {
    setLoading(true);
    try {
      // Call custom sign-out endpoint that clears HTTP-only cookies
      const response = await fetch(`${API_URL}/api/auth/sign-out-clear`, {
        method: 'POST',
        credentials: 'include',
      });

      if (!response.ok) {
        const errText = await response.text().catch(() => '');
        console.error('Sign out returned non-OK:', response.status, errText);
      }

      // Clear user state and local storage
      setUser(null);
      showMessage('Signed out successfully', 'success');

      // Close modal and reload page to clear all state
      setTimeout(() => {
        onClose();
        window.location.reload();
      }, 500);
    } catch (error) {
      console.error('Sign out error:', error);
      // Ensure user state and local storage are cleared even if API call fails
      setUser(null);
      showMessage('Signed out', 'success');
      setTimeout(() => {
        onClose();
        window.location.reload();
      }, 500);
    } finally {
      setLoading(false);
    }
  };

  if (!isOpen) return null;

  return (
    <div className={styles.modal}>
      <div className={styles.overlay} onClick={onClose} />
      <div className={styles.content}>
        <button className={styles.closeBtn} onClick={onClose}>×</button>

        {user ? (
          // Logged In View - Minimal profile
          <div className={styles.container}>
            <div className={styles.profileHeader}>
              <div className={styles.avatarCircle}>
                <UserIcon size={28} />
              </div>
              <h2 className={styles.welcomeTitle}>{user.name || user.email.split('@')[0]}</h2>
              <p className={styles.userEmail}>{user.email}</p>
            </div>

            <div className={styles.simpleInfo}>
              <div className={styles.infoRow}>
                <Mail size={16} className={styles.infoIcon} />
                <div className={styles.infoContent}>
                  <span className={styles.infoLabel}>Email</span>
                  <span className={styles.infoValue}>{user.email}</span>
                </div>
              </div>
              <div className={styles.infoRow}>
                <Shield size={16} className={styles.infoIcon} />
                <div className={styles.infoContent}>
                  <span className={styles.infoLabel}>Account Status</span>
                  <span className={styles.statusBadge}><CheckCircle size={12} /> Active</span>
                </div>
              </div>
            </div>

            <div className={styles.modalDivider} />

            <button
              className={styles.signOutBtn}
              onClick={handleSignOut}
              disabled={loading}
            >
              {loading ? (
                <>
                  <div className={styles.spinner}></div>
                  Signing out...
                </>
              ) : (
                <>
                  <LogOut size={16} />
                  Sign Out
                </>
              )}
            </button>
          </div>
        ) : (
            // Sign In / Sign Up View
          <div className={styles.container}>
            {message && (
              <div className={`${styles.message} ${styles[messageType]}`}>
                {message}
              </div>
            )}
            {mode === 'signin' ? (
              // Sign In Form - Modern Design
              <>
                <div className={styles.formHeader}>
                  <div className={styles.iconWrapper}>
                    <Lock size={24} />
                  </div>
                  <h3 className={styles.formTitle}>Welcome Back</h3>
                  <p className={styles.formSubtitle}>Sign in to continue your learning journey</p>
                </div>

                <form onSubmit={handleSignIn}>
                  <div className={styles.formGroup}>
                    <label>Email Address</label>
                    <div className={styles.inputWrapper}>
                      <Mail size={18} className={styles.inputIcon} />
                      <input
                        type="email"
                        value={email}
                        onChange={(e) => setEmail(e.target.value)}
                        placeholder="you@example.com"
                        className={styles.inputWithIcon}
                        required
                      />
                    </div>
                  </div>

                  <div className={styles.formGroup}>
                    <label>Password</label>
                    <div className={styles.inputWrapper}>
                      <Key size={18} className={styles.inputIcon} />
                      <input
                        type={showPassword ? "text" : "password"}
                        value={password}
                        onChange={(e) => setPassword(e.target.value)}
                        placeholder="Enter your password"
                        className={styles.inputWithIcon}
                        required
                      />
                      <button
                        type="button"
                        className={styles.passwordToggle}
                        onClick={() => setShowPassword(!showPassword)}
                      >
                        {showPassword ? <EyeOff size={18} /> : <Eye size={18} />}
                      </button>
                    </div>
                  </div>

                  <button type="submit" className={styles.submitBtn} disabled={loading || password.length < 6}>
                    {loading ? (
                      <>
                        <div className={styles.spinner}></div>
                        Signing in...
                      </>
                    ) : (
                      <>
                        Sign In
                        <ArrowRight size={18} />
                      </>
                    )}
                  </button>
                </form>
              </>
            ) : (
              // Simplified Sign Up Form - only name, email, password
              <>
                <div className={styles.formHeader}>
                  <div className={styles.iconWrapper}>
                    <UserPlus size={24} />
                  </div>
                  <h3 className={styles.formTitle}>Create Account</h3>
                  <p className={styles.formSubtitle}>One quick step to get started</p>
                </div>

                <form onSubmit={handleSignUp}>
                  <div className={styles.formGroup}>
                    <label>Full Name</label>
                    <div className={styles.inputWrapper}>
                      <UserIcon size={18} className={styles.inputIcon} />
                      <input
                        type="text"
                        value={name}
                        onChange={(e) => setName(e.target.value)}
                        placeholder="Enter your full name"
                        className={styles.inputWithIcon}
                        required
                      />
                    </div>
                  </div>

                  <div className={styles.formGroup}>
                    <label>Email Address</label>
                    <div className={styles.inputWrapper}>
                      <Mail size={18} className={styles.inputIcon} />
                      <input
                        type="email"
                        value={email}
                        onChange={(e) => setEmail(e.target.value)}
                        placeholder="you@example.com"
                        className={styles.inputWithIcon}
                        required
                      />
                    </div>
                  </div>

                  <div className={styles.formGroup}>
                    <label>Password</label>
                    <div className={styles.inputWrapper}>
                      <Key size={18} className={styles.inputIcon} />
                      <input
                        type={showPassword ? "text" : "password"}
                        value={password}
                        onChange={(e) => setPassword(e.target.value)}
                        placeholder="Create a strong password (min 8 characters)"
                        className={styles.inputWithIcon}
                        required
                      />
                      <button
                        type="button"
                        className={styles.passwordToggle}
                        onClick={() => setShowPassword(!showPassword)}
                      >
                        {showPassword ? <EyeOff size={18} /> : <Eye size={18} />}
                      </button>
                    </div>
                    <p className={styles.passwordHint}>Use at least 6 characters</p>
                  </div>

                  <button type="submit" className={styles.submitBtn} disabled={password.length < 6 || !name}>
                    {loading ? (
                      <>
                        <div className={styles.spinner}></div>
                        Creating Account...
                      </>
                    ) : (
                      <>
                        Create Account
                        <CheckCircle size={18} />
                      </>
                    )}
                  </button>
                </form>
              </>
            )}

            <div className={styles.toggle}>
              <p>
                {mode === 'signin' ? "Don't have an account? " : 'Already have an account? '}
                <button
                  type="button"
                  onClick={() => {
                    setMode(mode === 'signin' ? 'signup' : 'signin');
                    setMessage('');
                  }}
                  className={styles.toggleBtn}
                >
                  {mode === 'signin' ? 'Sign Up' : 'Sign In'}
                </button>
              </p>
            </div>
            {/* compact hint removed to keep modal minimal */}
          </div>
        )}
      </div>
    </div>
  );
}
