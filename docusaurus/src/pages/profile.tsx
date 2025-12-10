import React from 'react';
import ProfileDashboard from '@site/src/components/Auth/ProfileDashboard';
import Layout from '@theme/Layout';

export default function ProfilePage(): JSX.Element {
  return (
    <Layout title="Your Profile" description="View and update your profile information">
      <main style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: '80vh',
        padding: '2rem 0'
      }}>
        <div style={{ maxWidth: '600px', width: '100%', padding: '0 20px' }}>
          <ProfileDashboard />
        </div>
      </main>
    </Layout>
  );
}