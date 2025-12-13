import React from 'react';
import RagChatbot from '@site/src/components/RagChatbot';
import { authClient } from '@site/src/lib/authAdapter';
import { UserProvider } from '@site/src/contexts/UserContext';

// This Root component wraps the entire application
// The UserProvider wraps the entire app to provide personalization context
// The chatbot will appear on every page
// Note: Better Auth v1+ doesn't require a provider wrapper like previous versions
export default function Root({children}: {children: React.ReactNode}) {
  return (
    <UserProvider>
      <>
        {children}
        <RagChatbot />
      </>
    </UserProvider>
  );
}
