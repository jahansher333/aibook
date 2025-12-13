/**
 * SignUpForm Component with 7 Background Questions
 *
 * Uses Better-Auth React's useSignUp hook to register users
 * Collects email, password, and 7 custom questions for personalization
 */

import React, { useState } from 'react';
import Link from '@docusaurus/Link';
// import { authClient } from '@site/src/lib/authAdapter';
import styles from './Auth.module.css';
import GoogleOAuthButton from './GoogleOAuthButton';

interface SignUpFormData {
  email: string;
  password: string;
  confirmPassword: string;
  name: string;
  // 7 background questions
  rtx_gpu: boolean;
  rtx_model: string;
  jetson_board: 'none' | 'nano' | 'nx' | 'agx';
  ubuntu_experience: 'beginner' | 'intermediate' | 'expert';
  ros2_knowledge: 'none' | 'basic' | 'advanced';
  sim_preference: 'cloud' | 'local' | 'both';
  learning_goal: 'learn_basics' | 'build_humanoid' | 'research';
  preferred_language: 'english' | 'urdu';
}

export default function SignUpForm() {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const [formData, setFormData] = useState<SignUpFormData>({
    email: '',
    password: '',
    confirmPassword: '',
    name: '',
    rtx_gpu: false,
    rtx_model: '',
    jetson_board: 'none',
    ubuntu_experience: 'beginner',
    ros2_knowledge: 'none',
    sim_preference: 'cloud',
    learning_goal: 'learn_basics',
    preferred_language: 'english',
  });

  const [step, setStep] = useState<1 | 2>(1); // Step 1: Credentials, Step 2: Background Questions
  const [validationError, setValidationError] = useState<string>('');

  const handleInputChange = (
    e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>
  ) => {
    const { name, value, type } = e.target;

    if (type === 'checkbox') {
      const checked = (e.target as HTMLInputElement).checked;
      setFormData((prev) => ({ ...prev, [name]: checked }));
    } else {
      setFormData((prev) => ({ ...prev, [name]: value }));
    }
  };

  const validateStep1 = (): boolean => {
    if (!formData.email || !formData.password || !formData.confirmPassword) {
      setValidationError('All fields are required');
      return false;
    }

    if (formData.password.length < 8) {
      setValidationError('Password must be at least 8 characters');
      return false;
    }

    if (formData.password !== formData.confirmPassword) {
      setValidationError('Passwords do not match');
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

  const handleNext = () => {
    if (validateStep1()) {
      setStep(2);
    }
  };

  const handleBack = () => {
    setStep(1);
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (step === 1) {
      handleNext();
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      // Submit to our FastAPI backend directly
      const apiUrl = process.env.NODE_ENV === 'production'
        ? 'https://jahansher-aibook.hf.space/api/auth/sign-up'
        : 'http://127.0.0.1:8003/api/auth/sign-up';
      const response = await fetch(apiUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email: formData.email,
          password: formData.password,
          name: formData.name || null,
          // Include the 7 background questions
          background: {
            rtx_gpu: formData.rtx_gpu,
            rtx_model: formData.rtx_gpu ? formData.rtx_model : null,
            jetson_board: formData.jetson_board,
            ubuntu_experience: formData.ubuntu_experience,
            ros2_knowledge: formData.ros2_knowledge,
            sim_preference: formData.sim_preference,
            learning_goal: formData.learning_goal,
            preferred_language: formData.preferred_language,
          }
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

        // Save the user profile to localStorage for the UserContext to load
        const userProfile = {
          id: result.user_id,
          // Map the backend fields to UserContext fields
          learning_environment: formData.sim_preference === 'cloud' ? 'cloud_only' : (formData.sim_preference === 'local' ? 'local_only' : 'cloud_preferred'),
          hardware_experience: formData.rtx_gpu ? 'proficient' : 'none',
          ros2_knowledge: formData.ros2_knowledge as 'none' | 'basic' | 'intermediate' | 'advanced',
          learning_goal: formData.learning_goal === 'learn_basics' ? 'academic' : (formData.learning_goal === 'build_humanoid' ? 'hobby' : 'professional'),
          python_level: formData.ubuntu_experience === 'expert' ? 'expert' : (formData.ubuntu_experience === 'intermediate' ? 'intermediate' : 'beginner'),
          gpu_access: formData.rtx_gpu ? 'midrange' : 'none',
          profile_hash: result.profile_hash,
          timestamp: new Date().toISOString(),
        };
        localStorage.setItem('userProfile', JSON.stringify(userProfile));

        // Redirect to first chapter after successful signup
        window.location.href = '/aibook/ch01-physical-ai-intro/ch01';
      } else {
        // Show detailed error message
        const errorMsg = result.detail || result.message || JSON.stringify(result) || 'Signup failed. Please try again.';
        console.error('Signup failed:', errorMsg);
        setError(errorMsg);
      }
    } catch (err) {
      console.error('Signup error:', err);
      const errorMessage = err instanceof Error ? err.message : 'Network error. Please check if backend is running.';
      setError(`Error: ${errorMessage}`);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.authContainer}>
      <form onSubmit={handleSubmit} className={styles.authForm}>
        <h2>Create Account</h2>

        {error && (
          <div className={styles.errorMessage}>
            {error}
          </div>
        )}

        {validationError && (
          <div className={styles.errorMessage}>{validationError}</div>
        )}

        {step === 1 && (
          <>
            {/* Step 1: Account Credentials */}
            <div className={styles.formGroup}>
              <label htmlFor="email">Email *</label>
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
              <label htmlFor="password">Password *</label>
              <input
                type="password"
                id="password"
                name="password"
                value={formData.password}
                onChange={handleInputChange}
                placeholder="Min 8 characters"
                required
                disabled={isLoading}
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="confirmPassword">Confirm Password *</label>
              <input
                type="password"
                id="confirmPassword"
                name="confirmPassword"
                value={formData.confirmPassword}
                onChange={handleInputChange}
                placeholder="Re-enter password"
                required
                disabled={isLoading}
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="name">Name (Optional)</label>
              <input
                type="text"
                id="name"
                name="name"
                value={formData.name}
                onChange={handleInputChange}
                placeholder="Your name"
                disabled={isLoading}
              />
            </div>

            <button
              type="button"
              onClick={handleNext}
              className={styles.primaryButton}
              disabled={isLoading}
            >
              Next: Background Questions
            </button>
          </>
        )}

        {step === 2 && (
          <>
            {/* Step 2: 7 Background Questions */}
            <p className={styles.stepDescription}>
              Help us personalize your learning experience (7 questions)
            </p>

            <div className={styles.formGroup}>
              <label>
                <input
                  type="checkbox"
                  name="rtx_gpu"
                  checked={formData.rtx_gpu}
                  onChange={handleInputChange}
                  disabled={isLoading}
                />
                Do you have an NVIDIA RTX GPU?
              </label>
            </div>

            {formData.rtx_gpu && (
              <div className={styles.formGroup}>
                <label htmlFor="rtx_model">RTX GPU Model</label>
                <input
                  type="text"
                  id="rtx_model"
                  name="rtx_model"
                  value={formData.rtx_model}
                  onChange={handleInputChange}
                  placeholder="e.g., RTX 4090, RTX 3060"
                  disabled={isLoading}
                />
              </div>
            )}

            <div className={styles.formGroup}>
              <label htmlFor="jetson_board">Jetson Board</label>
              <select
                id="jetson_board"
                name="jetson_board"
                value={formData.jetson_board}
                onChange={handleInputChange}
                disabled={isLoading}
              >
                <option value="none">None</option>
                <option value="nano">Jetson Nano</option>
                <option value="nx">Jetson Xavier NX</option>
                <option value="agx">Jetson AGX Orin</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="ubuntu_experience">Ubuntu Experience</label>
              <select
                id="ubuntu_experience"
                name="ubuntu_experience"
                value={formData.ubuntu_experience}
                onChange={handleInputChange}
                disabled={isLoading}
              >
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="expert">Expert</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="ros2_knowledge">ROS2 Knowledge</label>
              <select
                id="ros2_knowledge"
                name="ros2_knowledge"
                value={formData.ros2_knowledge}
                onChange={handleInputChange}
                disabled={isLoading}
              >
                <option value="none">None</option>
                <option value="basic">Basic</option>
                <option value="advanced">Advanced</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="sim_preference">Simulation Preference</label>
              <select
                id="sim_preference"
                name="sim_preference"
                value={formData.sim_preference}
                onChange={handleInputChange}
                disabled={isLoading}
              >
                <option value="cloud">Cloud (Isaac Sim Cloud)</option>
                <option value="local">Local (Own Hardware)</option>
                <option value="both">Both</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="learning_goal">Learning Goal</label>
              <select
                id="learning_goal"
                name="learning_goal"
                value={formData.learning_goal}
                onChange={handleInputChange}
                disabled={isLoading}
              >
                <option value="learn_basics">Learn Basics</option>
                <option value="build_humanoid">Build Humanoid Robot</option>
                <option value="research">Research / Academic</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="preferred_language">Preferred Language</label>
              <select
                id="preferred_language"
                name="preferred_language"
                value={formData.preferred_language}
                onChange={handleInputChange}
                disabled={isLoading}
              >
                <option value="english">English</option>
                <option value="urdu">Urdu</option>
              </select>
            </div>

            <div className={styles.buttonGroup}>
              <button
                type="button"
                onClick={handleBack}
                className={styles.secondaryButton}
                disabled={isLoading}
              >
                Back
              </button>
              <button
                type="submit"
                className={styles.primaryButton}
                disabled={isLoading}
              >
                {isLoading ? 'Creating Account...' : 'Create Account'}
              </button>
            </div>
          </>
        )}

        <div style={{ marginTop: '20px', textAlign: 'center' }}>
          <p style={{ margin: '15px 0', color: '#666' }}>or</p>
          <GoogleOAuthButton mode="signup" disabled={isLoading} />
        </div>

        <div className={styles.formFooter}>
          <p>
            Already have an account?{' '}
            <Link to="/login" className={styles.link}>
              Sign In
            </Link>
          </p>
        </div>
      </form>
    </div>
  );
}
