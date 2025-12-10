import React from 'react';
import LoginForm from '@site/src/components/Auth/LoginForm';
import Layout from '@theme/Layout';

export default function LoginPage(): JSX.Element {
  return (
    <Layout title="Sign In" description="Sign in to access your personalized learning experience">
      <main style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: '80vh',
        padding: '2rem 0'
      }}>
        <div style={{ maxWidth: '600px', width: '100%', padding: '0 20px' }}>
          <LoginForm />
        </div>
      </main>
    </Layout>
  );
}