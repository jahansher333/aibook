# Quickstart: Personalize Button System

## Overview
Quick setup guide for implementing the personalization system that allows users to customize chapter content based on their profile (hardware access, experience level, learning goals).

## Prerequisites
- Docusaurus 3.x site with existing chapters
- Better-Auth authentication system configured
- User profiles with 7 background questions stored in Neon Postgres
- TypeScript/JavaScript development environment

## Installation Steps

### 1. Install Personalization Components
```bash
# Navigate to your Docusaurus src directory
cd docusaurus/src

# Create the components directory structure
mkdir -p components/PersonalizeButton components/PersonalizedContent components/ProfileProvider
mkdir -p utils styles
```

### 2. Create Core Components

#### ProfileProvider (Context Management)
Create `docusaurus/src/components/ProfileProvider/ProfileProvider.tsx`:
```tsx
import React, { createContext, useContext, useEffect, useState, ReactNode } from 'react';
import { useAuth } from 'use-better-auth/react'; // Assuming Better-Auth integration

interface UserProfile {
  id: string;
  hardware_experience: 'none' | 'some' | 'proficient' | 'expert';
  gpu_access: 'none' | 'consumer' | 'midrange' | 'highend';
  ros2_knowledge: 'none' | 'basic' | 'intermediate' | 'advanced';
  learning_goal: 'academic' | 'hobby' | 'career_transition' | 'professional';
  python_level: 'beginner' | 'intermediate' | 'advanced' | 'expert';
  learning_environment: 'cloud_only' | 'cloud_preferred' | 'local_preferred' | 'local_only';
  profile_hash: string;
  timestamp: string;
}

interface ProfileContextType {
  profile: UserProfile | null;
  loading: boolean;
  error: string | null;
  refreshProfile: () => Promise<void>;
}

const ProfileContext = createContext<ProfileContextType | undefined>(undefined);

export const ProfileProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [profile, setProfile] = useState<UserProfile | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const { session } = useAuth();

  useEffect(() => {
    loadProfile();
  }, [session?.user.id]);

  const loadProfile = async () => {
    if (!session?.user.id) {
      setLoading(false);
      return;
    }

    try {
      // Load profile from localStorage cache first
      const cachedProfile = localStorage.getItem(`profile_${session.user.id}`);
      if (cachedProfile) {
        setProfile(JSON.parse(cachedProfile));
        setLoading(false);
        return;
      }

      // Fetch from API if not in cache
      const response = await fetch(`/api/profile/${session.user.id}`);
      if (!response.ok) throw new Error('Failed to fetch profile');

      const profileData = await response.json();
      setProfile(profileData);

      // Cache in localStorage
      localStorage.setItem(`profile_${session.user.id}`, JSON.stringify(profileData));
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unknown error');
      console.error('Error loading profile:', err);
    } finally {
      setLoading(false);
    }
  };

  const refreshProfile = async () => {
    setLoading(true);
    setError(null);
    await loadProfile();
  };

  return (
    <ProfileContext.Provider value={{ profile, loading, error, refreshProfile }}>
      {children}
    </ProfileContext.Provider>
  );
};

export const useProfile = () => {
  const context = useContext(ProfileContext);
  if (context === undefined) {
    throw new Error('useProfile must be used within a ProfileProvider');
  }
  return context;
};
```

#### PersonalizeButton Component
Create `docusaurus/src/components/PersonalizeButton/PersonalizeButton.tsx`:
```tsx
import React, { useState } from 'react';
import { useProfile } from '../ProfileProvider/useProfile';
import { usePersonalization } from './usePersonalization';
import './PersonalizeButton.module.css';

interface PersonalizeButtonProps {
  chapterId: string;
  onPersonalize?: (isPersonalized: boolean) => void;
}

export const PersonalizeButton: React.FC<PersonalizeButtonProps> = ({
  chapterId,
  onPersonalize
}) => {
  const { profile, loading: profileLoading } = useProfile();
  const {
    isPersonalized,
    personalizeContent,
    revertToOriginal,
    loading: personalizationLoading
  } = usePersonalization(chapterId);

  const [hovered, setHovered] = useState(false);

  const handleClick = async () => {
    if (!profile) {
      alert('Please sign in to personalize content');
      return;
    }

    if (isPersonalized) {
      await revertToOriginal();
    } else {
      await personalizeContent(profile);
    }

    onPersonalize?.(!isPersonalized);
  };

  if (!profile && !profileLoading) {
    return (
      <button
        className="personalize-btn disabled"
        title="Sign in to personalize this chapter based on your background"
      >
        Sign in to Personalize
      </button>
    );
  }

  const isLoading = profileLoading || personalizationLoading;
  const buttonText = isPersonalized ? 'Show Original Content' : 'Personalize this chapter';

  return (
    <button
      className={`personalize-btn ${isPersonalized ? 'active' : ''} ${isLoading ? 'loading' : ''}`}
      onClick={handleClick}
      onMouseEnter={() => setHovered(true)}
      onMouseLeave={() => setHovered(false)}
      disabled={isLoading}
      title={hovered ?
        (isPersonalized ? 'Revert to original content' : 'Personalize based on your profile') :
        undefined
      }
    >
      {isLoading ? (
        <>
          <span className="spinner"></span>
          {isPersonalized ? 'Reverting...' : 'Personalizing...'}
        </>
      ) : (
        buttonText
      )}
    </button>
  );
};
```

#### Personalization Hook
Create `docusaurus/src/components/PersonalizeButton/usePersonalization.ts`:
```tsx
import { useState, useCallback } from 'react';
import { UserProfile } from '../ProfileProvider/ProfileProvider';
import { applyPersonalizationRules } from '../utils/content-transformer';
import { personalizationRules } from '../utils/personalization-rules';

export interface PersonalizationState {
  isPersonalized: boolean;
  originalContent: string;
  transformedContent: string;
  profileSummary: string;
  appliedRules: string[];
  loading: boolean;
  error: string | null;
}

export const usePersonalization = (chapterId: string) => {
  const [state, setState] = useState<PersonalizationState>({
    isPersonalized: false,
    originalContent: '',
    transformedContent: '',
    profileSummary: '',
    appliedRules: [],
    loading: false,
    error: null
  });

  const personalizeContent = useCallback(async (profile: UserProfile) => {
    try {
      setState(prev => ({ ...prev, loading: true, error: null }));

      // Get original content (in a real implementation, this would come from the DOM or props)
      const originalContent = document.body.innerHTML; // Simplified for example

      // Apply personalization rules
      const { transformedContent, appliedRules } = applyPersonalizationRules(
        originalContent,
        profile,
        personalizationRules
      );

      // Create profile summary
      const profileSummary = `${profile.learning_environment.replace('_', '-')}-${
        profile.ros2_knowledge
      }`;

      setState({
        isPersonalized: true,
        originalContent,
        transformedContent,
        profileSummary,
        appliedRules,
        loading: false,
        error: null
      });

      // Apply transformed content to DOM
      document.body.innerHTML = transformedContent;

    } catch (error) {
      setState(prev => ({
        ...prev,
        loading: false,
        error: error instanceof Error ? error.message : 'Personalization failed'
      }));
    }
  }, [chapterId]);

  const revertToOriginal = useCallback(async () => {
    try {
      setState(prev => ({ ...prev, loading: true, error: null }));

      // Revert to original content
      if (state.originalContent) {
        document.body.innerHTML = state.originalContent;
      }

      setState(prev => ({
        ...prev,
        isPersonalized: false,
        loading: false,
        error: null
      }));
    } catch (error) {
      setState(prev => ({
        ...prev,
        loading: false,
        error: error instanceof Error ? error.message : 'Revert failed'
      }));
    }
  }, [state.originalContent]);

  return {
    ...state,
    personalizeContent,
    revertToOriginal
  };
};
```

### 3. Add Personalization Rules Configuration
Create `docusaurus/src/utils/personalization-rules.ts`:
```ts
export interface PersonalizationRule {
  rule_id: string;
  profile_key: string;
  profile_value: string | string[];
  content_selector: string;
  transformation_type: 'show' | 'hide' | 'emphasize' | 'replace_text' | 'add_content';
  parameters?: Record<string, any>;
}

export const personalizationRules: PersonalizationRule[] = [
  // Learning Environment Rules
  {
    rule_id: "env-cloud-only-hide-local",
    profile_key: "learning_environment",
    profile_value: ["cloud_only", "cloud_preferred"],
    content_selector: ".local-installation-section",
    transformation_type: "hide"
  },
  {
    rule_id: "env-cloud-show-cloud",
    profile_key: "learning_environment",
    profile_value: ["cloud_only", "cloud_preferred"],
    content_selector: ".cloud-instructions",
    transformation_type: "show"
  },
  {
    rule_id: "env-local-show-local",
    profile_key: "learning_environment",
    profile_value: ["local_preferred", "local_only"],
    content_selector: ".local-installation-section",
    transformation_type: "show"
  },

  // ROS2 Knowledge Rules
  {
    rule_id: "ros2-none-add-primer",
    profile_key: "ros2_knowledge",
    profile_value: "none",
    content_selector: "body",
    transformation_type: "add_content",
    parameters: {
      content: `
        <div class="primer-section">
          <h3>ROS2 Primer</h3>
          <p>For beginners, here are the key concepts you should understand before proceeding...</p>
        </div>
      `,
      position: "prepend"
    }
  },
  {
    rule_id: "ros2-none-hide-advanced",
    profile_key: "ros2_knowledge",
    profile_value: "none",
    content_selector: ".advanced-ros2-section",
    transformation_type: "hide"
  },

  // Hardware Experience Rules
  {
    rule_id: "hw-none-simplify",
    profile_key: "hardware_experience",
    profile_value: ["none", "some"],
    content_selector: ".technical-specs",
    transformation_type: "hide"
  },
  {
    rule_id: "hw-expert-show-advanced",
    profile_key: "hardware_experience",
    profile_value: ["proficient", "expert"],
    content_selector: ".advanced-config",
    transformation_type: "show"
  },

  // GPU Access Rules
  {
    rule_id: "gpu-none-prioritize-cloud",
    profile_key: "gpu_access",
    profile_value: "none",
    content_selector: ".cloud-alternatives",
    transformation_type: "show"
  },
  {
    rule_id: "gpu-none-hide-local-gpu",
    profile_key: "gpu_access",
    profile_value: "none",
    content_selector: ".gpu-intensive-section",
    transformation_type: "hide"
  },

  // Python Level Rules
  {
    rule_id: "python-beginner-add-explanations",
    profile_key: "python_level",
    profile_value: "beginner",
    content_selector: "code",
    transformation_type: "add_content",
    parameters: {
      content: "<div class='python-explanation'>This code example demonstrates...</div>",
      position: "after"
    }
  }
];
```

### 4. Add CSS Styling
Create `docusaurus/src/components/PersonalizeButton/PersonalizeButton.module.css`:
```css
.personalize-btn {
  display: inline-flex;
  align-items: center;
  gap: 8px;
  padding: 8px 16px;
  background-color: #4a6cf7;
  color: white;
  border: none;
  border-radius: 6px;
  cursor: pointer;
  font-size: 14px;
  font-weight: 500;
  transition: all 0.2s ease;
  position: relative;
  min-height: 44px; /* For accessibility */
}

.personalize-btn:hover:not(:disabled) {
  background-color: #3a5ce5;
  transform: translateY(-1px);
}

.personalize-btn:active:not(:disabled) {
  transform: translateY(0);
}

.personalize-btn:disabled {
  background-color: #cccccc;
  cursor: not-allowed;
  opacity: 0.6;
}

.personalize-btn.active {
  background-color: #28a745;
}

.personalize-btn.active:hover:not(:disabled) {
  background-color: #218838;
}

.personalize-btn.loading {
  pointer-events: none;
}

.spinner {
  width: 16px;
  height: 16px;
  border: 2px solid transparent;
  border-top: 2px solid currentColor;
  border-radius: 50%;
  animation: spin 1s linear infinite;
  display: inline-block;
  vertical-align: middle;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}

/* Visual indicator for personalized content */
.personalized-indicator {
  display: inline-block;
  padding: 4px 8px;
  background-color: #d4edda;
  color: #155724;
  border-radius: 4px;
  font-size: 12px;
  margin-left: 8px;
  border: 1px solid #c3e6cb;
}
```

### 5. Integrate with Docusaurus Layout
Update your Docusaurus layout to include the ProfileProvider and add the PersonalizeButton to chapters.

In your Docusaurus config or layout wrapper, wrap your app with the ProfileProvider:

```tsx
// In your main layout or App component
import { ProfileProvider } from './src/components/ProfileProvider/ProfileProvider';

function App() {
  return (
    <ProfileProvider>
      {/* Your existing Docusaurus content */}
    </ProfileProvider>
  );
}
```

### 6. Add to Chapter Pages
Add the PersonalizeButton to each chapter by modifying the MDX files:

```mdx
---
title: Chapter 1 - Introduction to AI Robotics
---

import { PersonalizeButton } from '@site/src/components/PersonalizeButton/PersonalizeButton';

<div className="chapter-header">
  <h1>Chapter 1 - Introduction to AI Robotics</h1>
  <PersonalizeButton chapterId="ch01-intro" />
</div>

<!-- Rest of your chapter content -->
```

## Configuration

### Profile Fields
The system uses the following profile fields from Better-Auth:
- `hardware_experience`: none, some, proficient, expert
- `gpu_access`: none, consumer, midrange, highend
- `ros2_knowledge`: none, basic, intermediate, advanced
- `learning_goal`: academic, hobby, career_transition, professional
- `python_level`: beginner, intermediate, advanced, expert
- `learning_environment`: cloud_only, cloud_preferred, local_preferred, local_only

### Customization
- Add new personalization rules by extending the `personalizationRules` array
- Modify CSS classes in the module file to match your design
- Adjust the profile summary format in the hook

## Testing

### Manual Testing
1. Log in with different profile types
2. Navigate to a chapter with the PersonalizeButton
3. Click the button and verify content transforms appropriately
4. Verify "Show Original Content" reverts the changes
5. Test with different screen sizes and devices

### Automated Testing
```bash
# Run unit tests
npm run test PersonalizeButton

# Run E2E tests
npm run e2e personalization
```

## Troubleshooting

### Profile Not Loading
- Verify Better-Auth is properly configured
- Check that profile data is being stored in Neon Postgres
- Confirm the API endpoint for fetching profiles is working

### Content Not Transforming
- Check that CSS selectors in rules match actual content elements
- Verify profile data is being passed correctly to transformation logic
- Review browser console for JavaScript errors

### Performance Issues
- Ensure transformation rules are optimized
- Verify profile caching is working correctly
- Check for unnecessary re-renders