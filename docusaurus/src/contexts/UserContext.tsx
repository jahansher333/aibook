import React, { createContext, useContext, useState, ReactNode, useEffect } from 'react';

// Define the user profile type based on the spec
interface UserProfile {
  id?: string;
  hardware_experience: 'none' | 'some' | 'proficient' | 'expert';
  gpu_access: 'none' | 'consumer' | 'midrange' | 'highend';
  ros2_knowledge: 'none' | 'basic' | 'intermediate' | 'advanced';
  learning_goal: 'academic' | 'hobby' | 'career_transition' | 'professional';
  python_level: 'beginner' | 'intermediate' | 'advanced' | 'expert';
  learning_environment: 'cloud_only' | 'cloud_preferred' | 'local_preferred' | 'local_only';
  profile_hash?: string;
  timestamp?: string;
}

// Define the context type
interface UserContextType {
  userProfile: UserProfile | null;
  isPersonalized: boolean;
  setIsPersonalized: (isPersonalized: boolean) => void;
  setUserProfile: (profile: UserProfile | null) => void;
  isLoading: boolean;
  setIsLoading: (loading: boolean) => void;
  profileSummary: string;
}

// Create the context with default values
const UserContext = createContext<UserContextType | undefined>(undefined);

// Define default profile for anonymous users or when profile is not loaded
const DEFAULT_PROFILE: UserProfile = {
  hardware_experience: 'none',
  gpu_access: 'none',
  ros2_knowledge: 'none',
  learning_goal: 'academic',
  python_level: 'beginner',
  learning_environment: 'cloud_only',
  timestamp: new Date().toISOString(),
};

interface UserProviderProps {
  children: ReactNode;
}

// UserProvider component
export const UserProvider: React.FC<UserProviderProps> = ({ children }) => {
  const [userProfile, setUserProfile] = useState<UserProfile | null>(null);
  const [isPersonalized, setIsPersonalized] = useState<boolean>(false);
  const [isLoading, setIsLoading] = useState<boolean>(true);

  // Load user profile from localStorage or API on mount
  useEffect(() => {
    const loadUserProfile = async () => {
      try {
        // In a real implementation, this would fetch from Better-Auth API
        // For now, we'll simulate loading from localStorage
        const savedProfile = localStorage.getItem('userProfile');
        if (savedProfile) {
          setUserProfile(JSON.parse(savedProfile));
        } else {
          // Use default profile if none is saved
          setUserProfile(DEFAULT_PROFILE);
        }
      } catch (error) {
        console.error('Error loading user profile:', error);
        setUserProfile(DEFAULT_PROFILE);
      } finally {
        setIsLoading(false);
      }
    };

    loadUserProfile();
  }, []);

  // Listen for changes to userProfile in localStorage (from other components)
  useEffect(() => {
    const handleStorageChange = (e: StorageEvent) => {
      if (e.key === 'userProfile' && e.newValue) {
        try {
          const updatedProfile = JSON.parse(e.newValue);
          setUserProfile(updatedProfile);
          // Reset personalization when profile changes
          setIsPersonalized(false);
        } catch (error) {
          console.error('Error parsing updated profile:', error);
        }
      }
    };

    window.addEventListener('storage', handleStorageChange);
    return () => window.removeEventListener('storage', handleStorageChange);
  }, []);

  // Calculate profile summary for display
  const profileSummary = userProfile
    ? `${userProfile.learning_environment.replace('_', '-')} ${userProfile.hardware_experience} ${userProfile.ros2_knowledge}`
    : 'Not logged in';

  const value: UserContextType = {
    userProfile,
    isPersonalized,
    setIsPersonalized,
    setUserProfile,
    isLoading,
    setIsLoading,
    profileSummary,
  };

  return <UserContext.Provider value={value}>{children}</UserContext.Provider>;
};

// Custom hook to use the UserContext
export const useUserContext = (): UserContextType => {
  const context = useContext(UserContext);
  if (context === undefined) {
    throw new Error('useUserContext must be used within a UserProvider');
  }
  return context;
};

export default UserContext;