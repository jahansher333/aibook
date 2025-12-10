/**
 * Google OAuth Button Component
 *
 * Allows users to sign in with Google as an alternative to email/password
 */

import React from 'react';
import styles from '../Auth/Auth.module.css';
import { initiateGoogleOAuth } from '@site/src/lib/authAdapter';

interface GoogleOAuthButtonProps {
  mode?: 'signup' | 'login';
  disabled?: boolean;
}

export default function GoogleOAuthButton({ mode = 'signup', disabled = false }: GoogleOAuthButtonProps) {
  const handleGoogleLogin = async () => {
    if (disabled) return;

    try {
      const { auth_url } = await initiateGoogleOAuth();
      // Redirect to Google consent page
      window.location.href = auth_url;
    } catch (error) {
      console.error('Google OAuth error:', error);
      alert('Failed to initiate Google login. Please try again.');
    }
  };

  return (
    <button
      type="button"
      onClick={handleGoogleLogin}
      disabled={disabled}
      className={`${styles.googleButton} ${styles.primaryButton}`}
      style={{
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        gap: '10px',
        width: '100%',
        padding: '10px',
        border: '1px solid #dadce0',
        backgroundColor: 'white',
        color: 'rgba(0,0,0,.6)',
        borderRadius: '4px',
        cursor: disabled ? 'not-allowed' : 'pointer',
        opacity: disabled ? 0.6 : 1,
      }}
    >
      <svg width="18" height="18" viewBox="0 0 24 24">
        <path
          d="M22.56 12.25c0-.78-.07-1.53-.2-2.25H12v4.26h5.92c-.26 1.37-1.04 2.53-2.21 3.31v2.77h3.57c2.08-1.92 3.28-4.74 3.28-8.09z"
          fill="#4285F4"
        />
        <path
          d="M12 23c2.97 0 5.46-.98 7.28-2.66l-3.57-2.77c-.98.66-2.23 1.06-3.71 1.06-2.86 0-5.29-1.93-6.16-4.53H2.18v2.84C3.99 20.53 7.7 23 12 23z"
          fill="#34A853"
        />
        <path
          d="M5.84 14.09c-.22-.66-.35-1.36-.35-2.09s.13-1.43.35-2.09V7.07H2.18C1.43 8.55 1 10.22 1 12s.43 3.45 1.18 4.93l2.85-2.22.81-.62z"
          fill="#FBBC05"
        />
        <path
          d="M12 5.38c1.62 0 3.06.56 4.21 1.64l3.15-3.15C17.45 2.09 14.97 1 12 1 7.7 1 3.99 3.47 2.18 7.07l3.66 2.84c.87-2.6 3.3-4.53 6.16-4.53z"
          fill="#EA4335"
        />
      </svg>
      {mode === 'signup' ? 'Sign up with Google' : 'Sign in with Google'}
    </button>
  );
}