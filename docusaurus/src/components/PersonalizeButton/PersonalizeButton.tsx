import React, { useState, useEffect, useRef } from 'react';
import { useUserContext } from '../../contexts/UserContext';
import styles from './PersonalizeButton.module.css';

interface PersonalizeButtonProps {
  chapterId: string;
  chapterContent?: string;
}

// Get API URL based on environment
const getAPIUrl = (): string => {
  if (typeof window === 'undefined') {
    return 'http://localhost:8003';
  }
  const hostname = window.location.hostname;
  if (hostname === 'localhost' || hostname === '127.0.0.1') {
    return 'http://localhost:8003';
  }
  return 'https://your-api-domain.com';
};

// Extract text content from HTML, preserving structure for markdown
const extractTextContent = (element: Element): string => {
  // Clone the element to avoid modifying the DOM
  const clone = element.cloneNode(true) as Element;

  // Remove script and style elements
  clone.querySelectorAll('script, style, nav, .navbar, .footer, .pagination-nav').forEach(el => el.remove());

  // Get text content with some structure preserved
  let text = '';

  const processNode = (node: Node, depth: number = 0): void => {
    if (node.nodeType === Node.TEXT_NODE) {
      const content = node.textContent?.trim();
      if (content) {
        text += content + ' ';
      }
    } else if (node.nodeType === Node.ELEMENT_NODE) {
      const el = node as Element;
      const tagName = el.tagName.toLowerCase();

      // Add markdown-like structure
      if (tagName === 'h1') text += '\n# ';
      else if (tagName === 'h2') text += '\n## ';
      else if (tagName === 'h3') text += '\n### ';
      else if (tagName === 'h4') text += '\n#### ';
      else if (tagName === 'p') text += '\n\n';
      else if (tagName === 'li') text += '\n- ';
      else if (tagName === 'pre' || tagName === 'code') {
        text += '\n```\n' + el.textContent + '\n```\n';
        return; // Don't process children of code blocks
      }
      else if (tagName === 'br') text += '\n';

      // Process children
      node.childNodes.forEach(child => processNode(child, depth + 1));

      // Add line breaks after block elements
      if (['h1', 'h2', 'h3', 'h4', 'p', 'div', 'ul', 'ol', 'blockquote'].includes(tagName)) {
        text += '\n';
      }
    }
  };

  processNode(clone);

  // Clean up multiple newlines and spaces
  return text
    .replace(/\n{3,}/g, '\n\n')
    .replace(/ {2,}/g, ' ')
    .trim();
};

const PersonalizeButton: React.FC<PersonalizeButtonProps> = ({ chapterId, chapterContent = '' }) => {
  const { userProfile, isPersonalized, setIsPersonalized, isLoading } = useUserContext();
  const [isProcessing, setIsProcessing] = useState(false);
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const [originalContent, setOriginalContent] = useState<string | null>(null);
  const [adaptations, setAdaptations] = useState<string[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [duration, setDuration] = useState<number | null>(null);
  const [isCached, setIsCached] = useState(false);
  const articleRef = useRef<Element | null>(null);

  // Store reference to article element
  useEffect(() => {
    if (typeof window !== 'undefined') {
      articleRef.current = document.querySelector('article') || document.querySelector('.markdown') || document.querySelector('main');
    }
  }, []);

  const handlePersonalizeClick = async () => {
    if (!userProfile) {
      setError('Please complete your profile in Settings to personalize content');
      return;
    }

    // If already personalized, revert to original
    if (isPersonalized && originalContent && articleRef.current) {
      articleRef.current.innerHTML = originalContent;
      setIsPersonalized(false);
      setPersonalizedContent(null);
      setAdaptations([]);
      setError(null);
      return;
    }

    // Get content from DOM
    let contentToPersonalize = chapterContent;
    if (!contentToPersonalize || contentToPersonalize.length < 100) {
      const mainContent = articleRef.current;
      if (mainContent) {
        // Store original HTML for reverting
        setOriginalContent(mainContent.innerHTML);
        // Extract text content for API
        contentToPersonalize = extractTextContent(mainContent);
      }
    }

    if (!contentToPersonalize || contentToPersonalize.length < 50) {
      setError('Content too short to personalize. Navigate to a chapter page first.');
      return;
    }

    // Limit content length to avoid API issues
    if (contentToPersonalize.length > 15000) {
      contentToPersonalize = contentToPersonalize.substring(0, 15000) + '\n\n[Content truncated for processing...]';
    }

    setIsProcessing(true);
    setError(null);

    try {
      const API_BASE_URL = getAPIUrl();
      console.log('[Personalize] Sending request to:', `${API_BASE_URL}/api/v1/personalize`);
      console.log('[Personalize] Content length:', contentToPersonalize.length);
      console.log('[Personalize] User profile:', userProfile);

      const response = await fetch(`${API_BASE_URL}/api/v1/personalize`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content: contentToPersonalize,
          chapterId: chapterId,
          userProfile: {
            hardware_experience: userProfile.hardware_experience || 'none',
            gpu_access: userProfile.gpu_access || 'none',
            ros2_knowledge: userProfile.ros2_knowledge || 'none',
            learning_goal: userProfile.learning_goal || 'academic',
            python_level: userProfile.python_level || 'beginner',
            learning_environment: userProfile.learning_environment || 'cloud_only',
          }
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({ detail: 'Unknown error' }));
        throw new Error(errorData.detail || `Server error: ${response.status}`);
      }

      const data = await response.json();

      setPersonalizedContent(data.personalizedContent);
      setAdaptations(data.adaptations || []);
      setDuration(data.duration);
      setIsCached(data.cached || false);
      setIsPersonalized(true);

      console.log(`[Personalize] Completed in ${data.duration}ms (cached: ${data.cached})`);
      console.log(`[Personalize] Adaptations:`, data.adaptations);

    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Personalization failed. Please try again.';
      console.error('[Personalize] Error:', err);
      setError(errorMessage);
      // Restore original content if we saved it
      if (originalContent && articleRef.current) {
        articleRef.current.innerHTML = originalContent;
      }
    } finally {
      setIsProcessing(false);
    }
  };

  // If user profile is not loaded yet
  if (isLoading) {
    return (
      <div className={styles.container}>
        <button className={`${styles.button} ${styles.loading}`} disabled>
          <span className={styles.spinner}></span>
          Loading Profile...
        </button>
      </div>
    );
  }

  // Show profile summary if available
  const profileSummary = userProfile
    ? `${userProfile.python_level} Python, ${userProfile.gpu_access === 'none' ? 'No GPU' : 'Has GPU'}, ${userProfile.learning_environment.replace('_', ' ')}`
    : null;

  return (
    <div className={styles.container}>
      <button
        className={`${styles.button} ${isProcessing ? styles.processing : ''} ${isPersonalized ? styles.personalized : ''}`}
        onClick={handlePersonalizeClick}
        disabled={isProcessing}
        title={isPersonalized ? 'Click to show original content' : `Personalize for: ${profileSummary || 'your profile'}`}
      >
        {isProcessing ? (
          <>
            <span className={styles.spinner}></span>
            Personalizing with AI...
          </>
        ) : isPersonalized ? (
          <>
            <span className={styles.icon}>✓</span>
            Show Original Content
          </>
        ) : (
          <>
            <span className={styles.icon}>✨</span>
            Personalize for Me
          </>
        )}
      </button>

      {profileSummary && !isPersonalized && !isProcessing && (
        <div className={styles.profileHint}>
          Profile: {profileSummary}
        </div>
      )}

      {error && (
        <div className={styles.error} role="alert">
          <strong>Error:</strong> {error}
        </div>
      )}

      {isPersonalized && adaptations.length > 0 && (
        <div className={styles.adaptationsBox}>
          <strong>✨ Content personalized for your profile:</strong>
          <ul>
            {adaptations.map((adaptation, idx) => (
              <li key={idx}>{adaptation}</li>
            ))}
          </ul>
          {duration !== null && (
            <span className={styles.duration}>
              {isCached ? 'From cache' : `Processed in ${duration}ms`}
            </span>
          )}
        </div>
      )}

      {isPersonalized && personalizedContent && (
        <div
          className={styles.personalizedContent}
          dangerouslySetInnerHTML={{ __html: personalizedContent }}
        />
      )}
    </div>
  );
};

export default PersonalizeButton;
