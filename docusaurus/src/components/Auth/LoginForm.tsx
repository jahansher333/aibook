/**
 * LoginForm Component
 *
 * Uses Better-Auth React's useSignIn hook to authenticate users
 * Simple email/password form
 */

import React, { useState } from 'react';
import styles from './Auth.module.css';
import GoogleOAuthButton from './GoogleOAuthButton';

interface LoginFormData {
  email: string;
  password: string;
}

export default function LoginForm() {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const [formData, setFormData] = useState<LoginFormData>({
    email: '',
    password: '',
  });

  const [validationError, setValidationError] = useState<string>('');

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData((prev) => ({ ...prev, [name]: value }));
  };

  const validate = (): boolean => {
    if (!formData.email || !formData.password) {
      setValidationError('Email and password are required');
      return false;
    }

    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!emailRegex.test(formData.email)) {
      setValidationError('Invalid email address');
      return false;
    }

    setValidationError('');
    return true;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!validate()) {
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      // Submit to our FastAPI backend directly
      const response = await fetch('http://127.0.0.1:8003/api/auth/sign-in', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email: formData.email,
          password: formData.password,
        })
      });

      const result = await response.json();

      if (response.ok) {
        // Store the token in localStorage as our system expects
        localStorage.setItem('access_token', result.access_token);
        localStorage.setItem('user_id', result.user_id);
        if (result.profile_hash) {
          localStorage.setItem('profile_hash', result.profile_hash);
        }

        // Fetch user profile from backend after successful login
        try {
          const profileResponse = await fetch('http://127.0.0.1:8003/api/auth/me', {
            method: 'GET',
            headers: {
              'Authorization': `Bearer ${result.access_token}`,
              'Content-Type': 'application/json',
            }
          });

          if (profileResponse.ok) {
            const profileData = await profileResponse.json();
            if (profileData.user && profileData.profile) {
              // Map backend profile fields to UserContext fields
              const userProfile = {
                id: profileData.user.id,
                learning_environment: profileData.profile.sim_preference === 'cloud' ? 'cloud_only' : (profileData.profile.sim_preference === 'local' ? 'local_only' : 'cloud_preferred'),
                hardware_experience: profileData.profile.rtx_gpu ? 'proficient' : 'none',
                ros2_knowledge: profileData.profile.ros2_knowledge as 'none' | 'basic' | 'intermediate' | 'advanced',
                learning_goal: profileData.profile.learning_goal === 'learn_basics' ? 'academic' : (profileData.profile.learning_goal === 'build_humanoid' ? 'hobby' : 'professional'),
                python_level: profileData.profile.ubuntu_experience === 'expert' ? 'expert' : (profileData.profile.ubuntu_experience === 'intermediate' ? 'intermediate' : 'beginner'),
                gpu_access: profileData.profile.rtx_gpu ? 'midrange' : 'none',
                profile_hash: profileData.profile.profile_hash,
                timestamp: new Date().toISOString(),
              };
              localStorage.setItem('userProfile', JSON.stringify(userProfile));
            }
          }
        } catch (profileErr) {
          console.error('Failed to fetch profile:', profileErr);
          // Continue anyway - use default profile
        }

        // Redirect to first chapter after successful login
        window.location.href =
          "/docs/ch01-physical-ai-intro/";
      } else {
        setError(result.detail || 'Login failed. Please check your credentials.');
      }
    } catch (err) {
      console.error('Login error:', err);
      setError('An error occurred during login. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.authContainer}>
      <form onSubmit={handleSubmit} className={styles.authForm}>
        <h2>Sign In</h2>

        {error && (
          <div className={styles.errorMessage}>
            {error}
          </div>
        )}

        {validationError && (
          <div className={styles.errorMessage}>{validationError}</div>
        )}

        <div className={styles.formGroup}>
          <label htmlFor="email">Email</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleInputChange}
            placeholder="you@example.com"
            required
            disabled={isLoading}
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="password">Password</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleInputChange}
            placeholder="Your password"
            required
            disabled={isLoading}
          />
        </div>

        <button
          type="submit"
          className={styles.primaryButton}
          disabled={isLoading}
        >
          {isLoading ? 'Signing In...' : 'Sign In'}
        </button>

        <div style={{ marginTop: '20px', textAlign: 'center' }}>
          <p style={{ margin: '15px 0', color: '#666' }}>or</p>
          <GoogleOAuthButton mode="login" disabled={isLoading} />
        </div>

        <div className={styles.formFooter}>
          <p>
            Don't have an account?{' '}
            <a href="/signup" className={styles.link}>
              Create Account
            </a>
          </p>
        </div>
      </form>
    </div>
  );
}
