/**
 * Profile Dashboard Component
 *
 * Allows users to view and edit their profile information
 * Uses Better-Auth React's useSession hook to get current user data
 */

import React, { useState, useEffect } from 'react';
import styles from '../Auth/Auth.module.css';
import { useUserContext } from '../../contexts/UserContext';

interface ProfileData {
  rtx_gpu: boolean;
  rtx_model: string | null;
  jetson_board: 'none' | 'nano' | 'nx' | 'agx';
  ubuntu_experience: 'beginner' | 'intermediate' | 'expert';
  ros2_knowledge: 'none' | 'basic' | 'advanced';
  sim_preference: 'cloud' | 'local' | 'both';
  learning_goal: 'learn_basics' | 'build_humanoid' | 'research';
  preferred_language: 'english' | 'urdu';
}

export default function ProfileDashboard() {
  const { setUserProfile, setIsPersonalized } = useUserContext();
  const [profile, setProfile] = useState<ProfileData | null>(null);
  const [isEditing, setIsEditing] = useState(false);
  const [formData, setFormData] = useState<ProfileData | null>(null);
  const [message, setMessage] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [isLoggedIn, setIsLoggedIn] = useState(false);

  useEffect(() => {
    // Check if user is logged in by checking for access token
    const token = localStorage.getItem('access_token');
    if (token) {
      setIsLoggedIn(true);
      fetchProfile();
    } else {
      setIsLoading(false);
    }
  }, []);

  const fetchProfile = async () => {
    try {
      const token = localStorage.getItem('access_token');
      if (!token) {
        setIsLoggedIn(false);
        setIsLoading(false);
        return;
      }

      const apiUrl = process.env.NODE_ENV === 'production'
        ? 'https://jahansher-aibook.hf.space/api/auth/me'
        : 'http://127.0.0.1:8003/api/auth/me';
      const response = await fetch(apiUrl, {
        headers: {
          'Authorization': `Bearer ${token}`,
        },
      });

      if (response.ok) {
        const data = await response.json();
        if (data.profile) {
          setProfile(data.profile);
          setFormData(data.profile);
        }
      } else if (response.status === 401) {
        // Token is invalid, remove it
        localStorage.removeItem('access_token');
        localStorage.removeItem('user_id');
        localStorage.removeItem('profile_hash');
        setIsLoggedIn(false);
      }
    } catch (error) {
      console.error('Error fetching profile:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const handleEdit = () => {
    if (formData) {
      setIsEditing(true);
    }
  };

  const handleCancel = () => {
    if (profile) {
      setFormData({ ...profile });
    }
    setIsEditing(false);
    setMessage(null);
  };

  const handleChange = (
    e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>
  ) => {
    const { name, value, type } = e.target;

    if (type === 'checkbox') {
      const checked = (e.target as HTMLInputElement).checked;
      setFormData((prev: any) => ({ ...prev, [name]: checked }));
    } else {
      setFormData((prev: any) => ({ ...prev, [name]: value }));
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!formData) return;

    try {
      const token = localStorage.getItem('access_token');
      if (!token) {
        setMessage('You must be logged in to update your profile');
        return;
      }

      // Update local state
      setProfile({ ...formData });
      setIsEditing(false);

      // Convert ProfileData to UserProfile format and save to localStorage
      // Match the exact conversion logic from SignUpForm and LoginForm
      const userProfile = {
        learning_environment: formData.sim_preference === 'cloud' ? 'cloud_only' : (formData.sim_preference === 'local' ? 'local_only' : 'cloud_preferred'),
        hardware_experience: formData.rtx_gpu ? 'proficient' : 'none',
        ros2_knowledge: formData.ros2_knowledge as 'none' | 'basic' | 'intermediate' | 'advanced',
        learning_goal: formData.learning_goal === 'learn_basics' ? 'academic' : (formData.learning_goal === 'build_humanoid' ? 'hobby' : 'professional'),
        python_level: formData.ubuntu_experience === 'expert' ? 'expert' : (formData.ubuntu_experience === 'intermediate' ? 'intermediate' : 'beginner'),
        gpu_access: formData.rtx_gpu ? 'midrange' : 'none',
        timestamp: new Date().toISOString(),
      };

      // Save to localStorage
      localStorage.setItem('userProfile', JSON.stringify(userProfile));

      // Also update UserContext directly (for same-tab updates)
      setUserProfile(userProfile as any);

      // Reset personalization when profile changes
      setIsPersonalized(false);

      setMessage('Profile updated successfully!');

      // Clear message after 3 seconds
      setTimeout(() => setMessage(null), 3000);
    } catch (error) {
      console.error('Error updating profile:', error);
      setMessage('Failed to update profile. Please try again.');
    }
  };

  if (isLoading) {
    return <div>Loading...</div>;
  }

  if (!isLoggedIn) {
    return (
      <div className={styles.authContainer}>
        <div className={styles.authForm}>
          <h2>Profile Access</h2>
          <p>Please sign in to view your profile.</p>
          <a href="/login" className={styles.primaryButton}>Sign In</a>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.authContainer}>
      <div className={styles.authForm}>
        <h2>Your Profile</h2>

        {message && (
          <div className={styles.successMessage}>
            {message}
          </div>
        )}

        {isEditing ? (
          <form onSubmit={handleSubmit}>
            <div className={styles.formGroup}>
              <label>
                <input
                  type="checkbox"
                  name="rtx_gpu"
                  checked={formData?.rtx_gpu || false}
                  onChange={handleChange}
                />
                Do you have an NVIDIA RTX GPU?
              </label>
            </div>

            {formData?.rtx_gpu && (
              <div className={styles.formGroup}>
                <label htmlFor="rtx_model">RTX GPU Model</label>
                <input
                  type="text"
                  id="rtx_model"
                  name="rtx_model"
                  value={formData.rtx_model || ''}
                  onChange={handleChange}
                  placeholder="e.g., RTX 4090, RTX 3060"
                />
              </div>
            )}

            <div className={styles.formGroup}>
              <label htmlFor="jetson_board">Jetson Board</label>
              <select
                id="jetson_board"
                name="jetson_board"
                value={formData?.jetson_board || 'none'}
                onChange={handleChange}
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
                value={formData?.ubuntu_experience || 'beginner'}
                onChange={handleChange}
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
                value={formData?.ros2_knowledge || 'none'}
                onChange={handleChange}
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
                value={formData?.sim_preference || 'cloud'}
                onChange={handleChange}
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
                value={formData?.learning_goal || 'learn_basics'}
                onChange={handleChange}
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
                value={formData?.preferred_language || 'english'}
                onChange={handleChange}
              >
                <option value="english">English</option>
                <option value="urdu">Urdu</option>
              </select>
            </div>

            <div className={styles.buttonGroup}>
              <button
                type="button"
                onClick={handleCancel}
                className={styles.secondaryButton}
              >
                Cancel
              </button>
              <button
                type="submit"
                className={styles.primaryButton}
              >
                Save Changes
              </button>
            </div>
          </form>
        ) : (
          <div>
            {profile ? (
              <>
                <div className={styles.profileCard}>
                  <h3>Hardware & Setup</h3>
                  <p><strong>RTX GPU:</strong> {profile.rtx_gpu ? `Yes (${profile.rtx_model || 'Model not specified'})` : 'No'}</p>
                  <p><strong>Jetson Board:</strong> {profile.jetson_board === 'none' ? 'None' : profile.jetson_board}</p>
                  <p><strong>Simulation Preference:</strong> {profile.sim_preference}</p>
                </div>

                <div className={styles.profileCard}>
                  <h3>Experience & Goals</h3>
                  <p><strong>Ubuntu Experience:</strong> {profile.ubuntu_experience}</p>
                  <p><strong>ROS2 Knowledge:</strong> {profile.ros2_knowledge}</p>
                  <p><strong>Learning Goal:</strong> {profile.learning_goal}</p>
                  <p><strong>Preferred Language:</strong> {profile.preferred_language}</p>
                </div>

                <button
                  onClick={handleEdit}
                  className={styles.primaryButton}
                >
                  Edit Profile
                </button>
              </>
            ) : (
              <div>
                <p>You haven't completed your profile yet.</p>
                <p>Complete the 7-question quiz to personalize your learning experience.</p>
                <a href="/signup" className={styles.primaryButton}>Complete Profile</a>
              </div>
            )}
          </div>
        )}

        <div className={styles.formFooter}>
          <p>
            <a href="/" className={styles.link}>
              ← Back to Home
            </a>
          </p>
        </div>
      </div>
    </div>
  );
}