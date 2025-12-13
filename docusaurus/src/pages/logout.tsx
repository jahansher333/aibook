import React, { JSX, useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import { logout } from '@site/src/lib/authAdapter';

export default function LogoutPage(): JSX.Element {
  const [isLoggingOut, setIsLoggingOut] = useState(true);
  const [message, setMessage] = useState('Logging out...');

  useEffect(() => {
    // Perform logout
    logout();

    // Update message
    setMessage('You have been signed out successfully.');
    setIsLoggingOut(false);

    // Redirect to home page after a brief delay
    const timer = setTimeout(() => {
      window.location.href = '/';
    }, 2000);

    // Cleanup timeout if component unmounts
    return () => clearTimeout(timer);
  }, []);

  return (
    <Layout title="Signing Out" description="Logging out of your account">
      <main style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: '80vh',
        padding: '2rem 0'
      }}>
        <div style={{ maxWidth: '600px', width: '100%', padding: '0 20px', textAlign: 'center' }}>
          <div style={{
            backgroundColor: '#f8f9fa',
            borderRadius: '8px',
            padding: '2rem',
            boxShadow: '0 4px 6px rgba(0,0,0,0.1)'
          }}>
            <h1 style={{ color: '#495057', marginBottom: '1rem' }}>
              {isLoggingOut ? 'Signing Out...' : 'Signed Out'}
            </h1>
            <p style={{ color: '#6c757d', fontSize: '1.1rem' }}>
              {message}
            </p>
            {isLoggingOut && (
              <div style={{ marginTop: '1.5rem' }}>
                <div style={{
                  display: 'inline-block',
                  width: '40px',
                  height: '40px',
                  border: '4px solid #e9ecef',
                  borderTop: '4px solid #0d6efd',
                  borderRadius: '50%',
                  animation: 'spin 1s linear infinite'
                }}></div>
              </div>
            )}
            {!isLoggingOut && (
              <div style={{ marginTop: '1.5rem' }}>
                <Link
                  to="/login"
                  style={{
                    display: 'inline-block',
                    padding: '0.5rem 1.5rem',
                    backgroundColor: '#0d6efd',
                    color: 'white',
                    textDecoration: 'none',
                    borderRadius: '4px',
                    fontWeight: '500'
                  }}
                >
                  Sign In Again
                </Link>
              </div>
            )}
          </div>
        </div>
      </main>

      <style jsx>{`
        @keyframes spin {
          0% { transform: rotate(0deg); }
          100% { transform: rotate(360deg); }
        }
      `}</style>
    </Layout>
  );
}