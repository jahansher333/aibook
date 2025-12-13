/**
 * Auth Adapter for FastAPI Backend
 *
 * Provides authentication utilities connecting to FastAPI endpoints
 */

const FASTAPI_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://jahansher-aibook.hf.space'  // Hugging Face Space backend
  : 'http://127.0.0.1:8003';

// Export helper to manually logout (clear local storage)
export function logout() {
  localStorage.removeItem('access_token');
  localStorage.removeItem('user_id');
  localStorage.removeItem('profile_hash');
}

// Export helper to get current token
export function getAccessToken(): string | null {
  return localStorage.getItem('access_token');
}

// Export helper to get profile hash (for personalization cache key)
export function getProfileHash(): string | null {
  return localStorage.getItem('profile_hash');
}

// Google OAuth functions
export async function initiateGoogleOAuth(): Promise<{ auth_url: string }> {
  const response = await fetch(`${FASTAPI_BASE_URL}/api/auth/google/initiate`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
  });

  if (!response.ok) {
    throw new Error('Failed to initiate Google OAuth');
  }

  return response.json();
}

export async function handleGoogleCallback(code: string): Promise<any> {
  const response = await fetch(`${FASTAPI_BASE_URL}/api/auth/google/callback?code=${code}`, {
    method: 'GET',
    headers: {
      'Content-Type': 'application/json',
    },
  });

  if (!response.ok) {
    const errorData = await response.json();
    throw new Error(errorData.detail || 'Failed to handle Google OAuth callback');
  }

  const data = await response.json();

  // Store the token and user info
  if (data.access_token) {
    localStorage.setItem('access_token', data.access_token);
    localStorage.setItem('user_id', data.user_id);
    if (data.profile_completed) {
      // Get user profile to get profile hash if completed
      const profileResponse = await fetch(`${FASTAPI_BASE_URL}/api/auth/me`, {
        headers: {
          'Authorization': `Bearer ${data.access_token}`,
        },
      });

      if (profileResponse.ok) {
        const profileData = await profileResponse.json();
        if (profileData.profile && profileData.profile.profile_hash) {
          localStorage.setItem('profile_hash', profileData.profile.profile_hash);
        }
      }
    }
  }

  return data;
}
