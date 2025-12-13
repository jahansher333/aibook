import React, { JSX, useEffect } from 'react';
import { handleGoogleCallback } from '@site/src/lib/authAdapter';
import Layout from '@theme/Layout';

export default function GoogleCallbackPage(): JSX.Element {
  useEffect(() => {
    const urlParams = new URLSearchParams(window.location.search);
    const code = urlParams.get('code');
    const error = urlParams.get('error');

    if (error) {
      console.error('Google OAuth error:', error);
      alert(`Google OAuth error: ${error}`);
      window.location.href = '/login';
      return;
    }

    if (code) {
      handleGoogleCallback(code)
        .then((data) => {
          // Redirect based on profile completion
          if (data.profile_completed) {
            // User has completed profile, go to first chapter
            window.location.href = '/docs/ch01-physical-ai-intro/ch01';
          } else {
            // User needs to complete profile, redirect to signup to finish onboarding
            window.location.href = '/signup';
          }
        })
        .catch((error) => {
          console.error('Google OAuth callback error:', error);
          alert('Failed to complete Google login. Please try again.');
          window.location.href = '/login';
        });
    } else {
      // No code in URL, redirect to login
      window.location.href = '/login';
    }
  }, []);

  return (
    <Layout title="Google Login" description="Completing Google authentication">
      <main style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: '80vh',
        padding: '2rem 0'
      }}>
        <div style={{ maxWidth: '600px', width: '100%', padding: '0 20px' }}>
          <div style={{
            textAlign: 'center',
            padding: '2rem',
            border: '1px solid #ddd',
            borderRadius: '8px',
            backgroundColor: '#f9f9f9'
          }}>
            <h2>Completing Google Login...</h2>
            <p>Please wait while we complete your authentication.</p>
          </div>
        </div>
      </main>
    </Layout>
  );
}