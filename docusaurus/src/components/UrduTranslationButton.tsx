/**
 * Urdu Translation Button Component
 * Provides a button to translate chapter content to Urdu using backend API
 */

import React, { useState, useCallback } from 'react';
import './UrduTranslationButton.module.css';

interface UrduTranslationButtonProps {
  chapterId: string;
  chapterContent?: string;
  children?: React.ReactNode;
}

interface TranslationState {
  isUrdu: boolean;
  isLoading: boolean;
  error: string | null;
  translatedContent: string | null;
  duration: number | null;
}

// Get API URL - automatically detect based on environment
const getAPIUrl = (): string => {
  if (typeof window === 'undefined') {
    return 'http://localhost:8003';
  }

  const hostname = window.location.hostname;

  // Development
  if (hostname === 'localhost' || hostname === '127.0.0.1') {
    return 'http://localhost:8003';
  }

  // Production - update this with your actual backend URL
  return 'https://your-api-domain.com';
};

export const UrduTranslationButton: React.FC<UrduTranslationButtonProps> = ({
  chapterId,
  chapterContent = '',
  children,
}) => {
  const [state, setState] = useState<TranslationState>({
    isUrdu: false,
    isLoading: false,
    error: null,
    translatedContent: null,
    duration: null,
  });

  const handleTranslate = useCallback(async () => {
    // If already in Urdu, revert to English
    if (state.isUrdu) {
      setState((prev) => ({
        ...prev,
        isUrdu: false,
        error: null,
      }));
      return;
    }

    // Get content from DOM if not provided
    let contentToTranslate = chapterContent;
    if (!contentToTranslate) {
      const mainContent = document.querySelector('main') || document.querySelector('article');
      if (mainContent) {
        contentToTranslate = mainContent.textContent || '';
      }
    }

    // Validate content
    if (!contentToTranslate || contentToTranslate.length < 100) {
      setState((prev) => ({
        ...prev,
        error: 'Content too short to translate',
      }));
      return;
    }

    // Start loading
    setState((prev) => ({
      ...prev,
      isLoading: true,
      error: null,
    }));

    try {
      const API_BASE_URL = getAPIUrl();
      const response = await fetch(`${API_BASE_URL}/api/v1/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapterId,
          content: contentToTranslate,
          skipCache: false,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        const errorMessage = errorData.detail?.message || errorData.message || 'Translation failed';
        throw new Error(errorMessage);
      }

      const data = await response.json();

      setState((prev) => ({
        ...prev,
        isUrdu: true,
        isLoading: false,
        error: null,
        translatedContent: data.urduContent,
        duration: data.duration,
      }));

      // Log performance
      console.log(`Translation for ${chapterId}: ${data.duration}ms (cached: ${data.cached})`);
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Translation failed. Please try again.';
      console.error('Translation error:', error);

      setState((prev) => ({
        ...prev,
        isLoading: false,
        error: errorMessage,
        isUrdu: false,
      }));
    }
  }, [chapterId, chapterContent, state.isUrdu]);

  const buttonLabel = state.isLoading
    ? 'Translating...'
    : state.isUrdu
      ? 'Show in English'
      : 'اردو میں دیکھیں';

  return (
    <div className="urdu_translation_container">
      <button
        onClick={handleTranslate}
        disabled={state.isLoading}
        className={`urdu_translation_button ${state.isUrdu ? 'active' : ''} ${state.isLoading ? 'loading' : ''}`}
        title={state.isUrdu ? 'Click to show English' : 'Click to view in Urdu'}
        aria-label={buttonLabel}
      >
        {state.isLoading && <span className="spinner"></span>}
        <span className="button_text">{buttonLabel}</span>
      </button>

      {state.error && (
        <div className="error_message" role="alert">
          <span className="error_icon">⚠️</span>
          <span>{state.error}</span>
          {state.error.includes('rate limit') && (
            <span className="retry_hint"> (Retry after 60 seconds)</span>
          )}
        </div>
      )}

      {state.isUrdu && state.translatedContent && (
        <div
          className="translated_content"
          dir="rtl"
          lang="ur"
          dangerouslySetInnerHTML={{ __html: state.translatedContent }}
        />
      )}

      {!state.isUrdu && children && (
        <div className="original_content">
          {children}
        </div>
      )}
    </div>
  );
};

export default UrduTranslationButton;
