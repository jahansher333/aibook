/**
 * Urdu Translation Button Component
 * Provides a button to translate chapter content to Urdu using backend API
 */

import React, { useState, useCallback } from 'react';
import styles from './UrduTranslationButton.module.css';

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

// Check if running locally (backend available) or on GitHub Pages (no backend)
const isLocalEnvironment = (): boolean => {
  if (typeof window === 'undefined') return true;
  const hostname = window.location.hostname;
  return hostname === 'localhost' || hostname === '127.0.0.1';
};

// Get API URL - automatically detect based on environment
const getAPIUrl = (): string => {
  if (typeof window === 'undefined') return 'http://localhost:8003';
  const hostname = window.location.hostname;
  if (hostname === 'localhost' || hostname === '127.0.0.1') {
    return 'http://localhost:8003';
  }
  return 'https://jahansher-aibook.hf.space';
};

// Simple markdown to HTML converter for basic formatting
const markdownToHtml = (markdown: string): string => {
  return markdown
    // Remove leading/trailing markdown delimiters
    .replace(/^---\n?/, '')
    .replace(/\n?---$/, '')
    // Code blocks (```code```)
    .replace(/```(\w*)\n([\s\S]*?)```/g, '<pre><code class="language-$1">$2</code></pre>')
    // Inline code (`code`)
    .replace(/`([^`]+)`/g, '<code>$1</code>')
    // Headers (# to h1, ## to h2, etc.)
    .replace(/^### (.*$)/gm, '<h3>$1</h3>')
    .replace(/^## (.*$)/gm, '<h2>$1</h2>')
    .replace(/^# (.*$)/gm, '<h1>$1</h1>')
    // Bold (**text** or __text__)
    .replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>')
    .replace(/__([^_]+)__/g, '<strong>$1</strong>')
    // Italic (*text* or _text_)
    .replace(/\*([^*]+)\*/g, '<em>$1</em>')
    .replace(/_([^_]+)_/g, '<em>$1</em>')
    // Links [text](url)
    .replace(/\[([^\]]+)\]\(([^)]+)\)/g, '<a href="$2">$1</a>')
    // Unordered lists (- item)
    .replace(/^- (.+)$/gm, '<li>$1</li>')
    // Wrap list items in ul
    .replace(/(<li>.*<\/li>\n?)+/g, '<ul>$&</ul>')
    // Line breaks (double newline = paragraph)
    .replace(/\n\n/g, '</p><p>')
    // Single newline = br
    .replace(/\n/g, '<br/>')
    // Wrap in paragraph if needed
    .replace(/^(.+)$/s, '<p>$1</p>')
    // Clean up empty paragraphs
    .replace(/<p><\/p>/g, '')
    .replace(/<p><br\/><\/p>/g, '');
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

  // Translation now works in production via Hugging Face Space backend

  return (
    <div className={styles.urdu_translation_container}>
      <button
        onClick={handleTranslate}
        disabled={state.isLoading}
        className={`${styles.urdu_translation_button} ${state.isUrdu ? styles.active : ''} ${state.isLoading ? styles.loading : ''}`}
        title={state.isUrdu ? 'Click to show English' : 'Click to view in Urdu'}
        aria-label={buttonLabel}
      >
        {state.isLoading && <span className={styles.spinner}></span>}
        <span className={styles.button_text}>{buttonLabel}</span>
      </button>

      {state.error && (
        <div className={styles.error_message} role="alert">
          <span className={styles.error_icon}>⚠️</span>
          <span>{state.error}</span>
          {state.error.includes('rate limit') && (
            <span className={styles.retry_hint}> (Retry after 60 seconds)</span>
          )}
        </div>
      )}

      {state.isUrdu && state.translatedContent && (
        <div
          className={styles.translated_content}
          dir="rtl"
          lang="ur"
          dangerouslySetInnerHTML={{ __html: markdownToHtml(state.translatedContent) }}
        />
      )}

      {!state.isUrdu && children && (
        <div className={styles.original_content}>
          {children}
        </div>
      )}
    </div>
  );
};

export default UrduTranslationButton;
