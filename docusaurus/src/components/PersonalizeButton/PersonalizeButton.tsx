import React, { useState } from 'react';
import { useUserContext } from '../../contexts/UserContext';
import { personalizeChapter } from '../../utils/personalizeChapter';

interface PersonalizeButtonProps {
  chapterId: string;
  chapterContent: string;
}

const PersonalizeButton: React.FC<PersonalizeButtonProps> = ({ chapterId, chapterContent }) => {
  const { userProfile, isPersonalized, setIsPersonalized, isLoading } = useUserContext();
  const [isProcessing, setIsProcessing] = useState(false);

  const handlePersonalizeClick = async () => {
    if (!userProfile) {
      alert('Please sign in to personalize this chapter');
      return;
    }

    setIsProcessing(true);

    try {
      // Simulate the personalization process
      // In a real implementation, this would transform the content based on the user's profile
      await new Promise(resolve => setTimeout(resolve, 300)); // Simulate processing time

      // Toggle personalization state
      setIsPersonalized(!isPersonalized);
    } catch (error) {
      console.error('Error during personalization:', error);
      alert('Failed to personalize content. Please try again.');
    } finally {
      setIsProcessing(false);
    }
  };

  // If user is not logged in, show a different message
  if (!userProfile && !isLoading) {
    return (
      <div className="personalize-button-container">
        <button
          className="personalize-button disabled"
          disabled
          title="Sign in to personalize this chapter"
        >
          Sign In to Personalize
        </button>
      </div>
    );
  }

  return (
    <div className="personalize-button-container">
      {isLoading ? (
        <button className="personalize-button loading" disabled>
          Loading...
        </button>
      ) : (
        <button
          className={`personalize-button ${isProcessing ? 'processing' : ''} ${isPersonalized ? 'personalized' : ''}`}
          onClick={handlePersonalizeClick}
          disabled={isProcessing}
          title={isPersonalized ? 'Show Original Content' : 'Personalize this chapter based on your profile'}
        >
          {isProcessing ? (
            <>
              <span className="spinner"></span> {isPersonalized ? 'Reverting...' : 'Personalizing...'}
            </>
          ) : isPersonalized ? (
            'Show Original Content'
          ) : (
            'Personalize this chapter'
          )}
        </button>
      )}
      {isPersonalized && (
        <div className="personalization-indicator">
          Personalized for: {userProfile?.learning_environment.replace('_', ' ')} {userProfile?.hardware_experience}
        </div>
      )}
    </div>
  );
};

export default PersonalizeButton;