import React from 'react';
import SignUpForm from '@site/src/components/Auth/SignUpForm';
import Layout from '@theme/Layout';

export default function SignupPage(): JSX.Element {
  return (
    <Layout title="Sign Up" description="Create your account to personalize your learning experience">
      <main style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: '80vh',
        padding: '2rem 0'
      }}>
        <div style={{ maxWidth: '600px', width: '100%', padding: '0 20px' }}>
          <SignUpForm />
        </div>
      </main>
    </Layout>
  );
}