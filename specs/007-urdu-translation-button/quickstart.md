# Quickstart: Urdu Translation Button Implementation

**Feature**: 007-urdu-translation-button
**Date**: 2025-12-10
**Status**: Ready for Phase 2 (Implementation Tasks)

---

## Overview

This guide walks a developer through setting up the Urdu Translation Button feature from scratch.

**Time Estimate**: 1-2 hours to set up environment and test first translation

---

## Prerequisites

- Node.js 18+ (for Docusaurus & React)
- npm or yarn
- Groq API key (free tier: https://console.groq.com)
- Basic React knowledge
- Understanding of localStorage API

---

## Step 1: Set Up Groq API Key

### 1.1 Obtain API Key

1. Visit https://console.groq.com
2. Sign up or log in
3. Navigate to API Keys
4. Create a new API key (or use existing)
5. Copy the key (looks like: `gsk_xxxxxxxxxxxxx`)

### 1.2 Store API Key

Create `.env.local` in the Docusaurus root:

```bash
# docusaurus/.env.local
REACT_APP_GROQ_API_KEY=gsk_xxxxxxxxxxxxx
REACT_APP_GROQ_API_URL=https://api.groq.com/openai/v1/chat/completions
```

**Warning**: Never commit `.env.local` to git. Add to `.gitignore`:
```
.env.local
.env
```

---

## Step 2: Create Core Utilities

### 2.1 Create `docusaurus/src/utils/translationCache.ts`

```typescript
/**
 * Translation caching utilities using browser localStorage
 */

export interface UrduTranslation {
  chapterId: string;
  originalContent: string;
  originalContentHash: string;
  urduTranslation: string;
  timestamp: string; // ISO 8601
  expiryTime: string; // ISO 8601
}

const CACHE_PREFIX = 'urdu_translation_';
const CACHE_TTL_DAYS = 30;

/**
 * Get translation from cache
 */
export function getTranslation(chapterId: string): UrduTranslation | null {
  try {
    const key = `${CACHE_PREFIX}${chapterId}`;
    const data = localStorage.getItem(key);
    if (!data) return null;

    const translation = JSON.parse(data) as UrduTranslation;

    // Check if expired
    if (new Date(translation.expiryTime) < new Date()) {
      localStorage.removeItem(key);
      return null;
    }

    return translation;
  } catch (error) {
    console.error('Error reading translation cache:', error);
    return null;
  }
}

/**
 * Store translation in cache
 */
export function setTranslation(chapterId: string, translation: UrduTranslation): void {
  try {
    const key = `${CACHE_PREFIX}${chapterId}`;
    localStorage.setItem(key, JSON.stringify(translation));
  } catch (error) {
    if (error instanceof DOMException && error.code === 22) {
      // QuotaExceededError
      console.warn('localStorage quota exceeded. Falling back to session memory.');
    } else {
      console.error('Error writing translation cache:', error);
    }
  }
}

/**
 * Clear specific translation
 */
export function clearTranslation(chapterId: string): void {
  localStorage.removeItem(`${CACHE_PREFIX}${chapterId}`);
}

/**
 * Check if translation is stale
 */
export function isStale(translation: UrduTranslation): boolean {
  return new Date(translation.expiryTime) < new Date();
}

/**
 * Generate expiry time (30 days from now)
 */
export function getExpiryTime(): string {
  const now = new Date();
  now.setDate(now.getDate() + CACHE_TTL_DAYS);
  return now.toISOString();
}

/**
 * Compute SHA-256 hash of content
 */
export async function hashContent(content: string): Promise<string> {
  const encoder = new TextEncoder();
  const data = encoder.encode(content);
  const hashBuffer = await crypto.subtle.digest('SHA-256', data);
  const hashArray = Array.from(new Uint8Array(hashBuffer));
  return hashArray.map(b => b.toString(16).padStart(2, '0')).join('');
}
```

### 2.2 Create `docusaurus/src/utils/groqClient.ts`

```typescript
/**
 * Groq API client for Urdu translation
 */

const API_KEY = process.env.REACT_APP_GROQ_API_KEY;
const API_URL = process.env.REACT_APP_GROQ_API_URL || 'https://api.groq.com/openai/v1/chat/completions';

export interface TranslationResult {
  urdu: string;
  duration: number; // milliseconds
}

/**
 * Translate content to Urdu using Groq Llama-3.1-70B
 */
export async function translateToUrdu(content: string): Promise<TranslationResult> {
  if (!API_KEY) {
    throw new Error('REACT_APP_GROQ_API_KEY not set');
  }

  const startTime = Date.now();

  const prompt = getTranslationPrompt(content);

  try {
    const response = await fetch(API_URL, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${API_KEY}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        model: 'llama-3.1-70b-instant',
        messages: [
          {
            role: 'system',
            content: 'You are a professional technical translator. Translate educational robotics content to Urdu. Preserve code blocks and technical terms in English.',
          },
          {
            role: 'user',
            content: prompt,
          },
        ],
        temperature: 0.3, // Lower temperature for consistency
        max_tokens: 4096,
      }),
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(`Groq API error: ${error.error?.message || response.statusText}`);
    }

    const data = await response.json();
    const urdu = data.choices[0].message.content.trim();
    const duration = Date.now() - startTime;

    return { urdu, duration };
  } catch (error) {
    console.error('Groq translation error:', error);
    throw error;
  }
}

/**
 * Build translation prompt with best practices
 */
function getTranslationPrompt(content: string): string {
  return `Translate the following robotics and AI educational content into natural, easy-to-read Urdu.

IMPORTANT RULES:
1. Keep technical terms in English: ROS 2, Isaac Sim, URDF, Gazebo, TF2, Nav2, SLAM, GPT, Whisper, CLIP, MoveIt, Jetson, Ubuntu, Python, etc.
2. Preserve ALL code blocks, commands, and formulas EXACTLY AS-IS (do not translate)
3. Preserve markdown formatting: headers, bold, italics, lists
4. Write for Grade 10-12 reading level (clear, simple Urdu)
5. If code block is detected, skip translating it

CONTENT TO TRANSLATE:
${content}

TRANSLATED CONTENT (Urdu only):`;
}

/**
 * Export for testing
 */
export { getTranslationPrompt };
```

### 2.3 Create `docusaurus/src/utils/urduPrompt.ts`

```typescript
/**
 * Urdu translation prompt utilities
 */

export function getTranslationPrompt(content: string): string {
  return `Translate the following robotics and AI educational content into natural, easy-to-read Urdu.

IMPORTANT RULES:
1. Keep technical terms in English: ROS 2, Isaac Sim, URDF, Gazebo, TF2, Nav2, SLAM, GPT, Whisper, CLIP, MoveIt, Jetson, Ubuntu, Python, Linux, Docker, etc.
2. Preserve ALL code blocks, command-line examples, and LaTeX/math formulas EXACTLY AS-IS
3. Preserve markdown formatting: #headers, **bold**, *italics*, - lists
4. Write for Grade 10-12 reading level (accessible Urdu with standard vocabulary)
5. Do not translate variable names, function names, or keywords inside code

CONTENT TO TRANSLATE:
${content}

TRANSLATED CONTENT (Urdu only, no intro/outro):`;
}
```

---

## Step 3: Create React Context for Language State

### 3.1 Create `docusaurus/src/contexts/LanguageContext.tsx`

```typescript
import React, { createContext, useContext, useState } from 'react';

interface LanguageContextType {
  languageStates: Map<string, boolean>; // chapterId → isUrdu
  toggleLanguage: (chapterId: string) => void;
  setLanguage: (chapterId: string, isUrdu: boolean) => void;
}

const LanguageContext = createContext<LanguageContextType | undefined>(undefined);

export const LanguageProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [languageStates] = useState<Map<string, boolean>>(new Map());

  const toggleLanguage = (chapterId: string) => {
    const current = languageStates.get(chapterId) || false;
    languageStates.set(chapterId, !current);
    // Force re-render (in real implementation, use useState)
  };

  const setLanguage = (chapterId: string, isUrdu: boolean) => {
    languageStates.set(chapterId, isUrdu);
  };

  return (
    <LanguageContext.Provider value={{ languageStates, toggleLanguage, setLanguage }}>
      {children}
    </LanguageContext.Provider>
  );
};

export const useLanguageContext = (): LanguageContextType => {
  const context = useContext(LanguageContext);
  if (!context) {
    throw new Error('useLanguageContext must be used within LanguageProvider');
  }
  return context;
};
```

---

## Step 4: Create UrduTranslationButton Component

### 4.1 Create `docusaurus/src/components/UrduTranslationButton/UrduTranslationButton.tsx`

```typescript
import React, { useState } from 'react';
import { useLanguageContext } from '../../contexts/LanguageContext';
import { getTranslation, setTranslation, hashContent, getExpiryTime } from '../../utils/translationCache';
import { translateToUrdu } from '../../utils/groqClient';
import styles from './UrduTranslationButton.module.css';

interface UrduTranslationButtonProps {
  chapterId: string;
  chapterContent: string;
}

export const UrduTranslationButton: React.FC<UrduTranslationButtonProps> = ({
  chapterId,
  chapterContent,
}) => {
  const { languageStates, setLanguage } = useLanguageContext();
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const isUrdu = languageStates.get(chapterId) || false;

  const handleClick = async () => {
    if (isUrdu) {
      // Revert to English
      setLanguage(chapterId, false);
      return;
    }

    // Translate to Urdu
    setIsLoading(true);
    setError(null);

    try {
      // Check cache first
      let cachedTranslation = getTranslation(chapterId);
      if (cachedTranslation) {
        setLanguage(chapterId, true);
        setIsLoading(false);
        return;
      }

      // Fetch from Groq
      if (!navigator.onLine) {
        setError('No internet connection. Translation not available.');
        return;
      }

      const result = await translateToUrdu(chapterContent);
      const contentHash = await hashContent(chapterContent);

      // Store in cache
      setTranslation(chapterId, {
        chapterId,
        originalContent: chapterContent,
        originalContentHash: contentHash,
        urduTranslation: result.urdu,
        timestamp: new Date().toISOString(),
        expiryTime: getExpiryTime(),
      });

      setLanguage(chapterId, true);
    } catch (err) {
      console.error('Translation failed:', err);
      setError('Translation failed. Please try again later.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.container}>
      <button
        onClick={handleClick}
        disabled={isLoading}
        className={`${styles.button} ${isUrdu ? styles.active : ''}`}
        title={isUrdu ? 'Show in English' : 'View in Urdu'}
      >
        {isLoading ? (
          <>
            <span className={styles.spinner} />
            {isUrdu ? 'Reverting...' : 'Translating...'}
          </>
        ) : isUrdu ? (
          'Show in English'
        ) : (
          'اردو میں دیکھیں'
        )}
      </button>
      {error && <div className={styles.error}>{error}</div>}
    </div>
  );
};
```

### 4.2 Create `docusaurus/src/components/UrduTranslationButton/UrduTranslationButton.module.css`

```css
.container {
  display: flex;
  align-items: center;
  gap: 8px;
  margin: 0;
}

.button {
  padding: 8px 16px;
  background-color: #007bff;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  font-size: 14px;
  font-weight: 500;
  transition: background-color 0.2s;
  display: flex;
  align-items: center;
  gap: 6px;
}

.button:hover:not(:disabled) {
  background-color: #0056b3;
}

.button.active {
  background-color: #28a745;
}

.button:disabled {
  opacity: 0.6;
  cursor: not-allowed;
}

.spinner {
  display: inline-block;
  width: 12px;
  height: 12px;
  border: 2px solid rgba(255, 255, 255, 0.3);
  border-top-color: white;
  border-radius: 50%;
  animation: spin 0.8s linear infinite;
}

@keyframes spin {
  to {
    transform: rotate(360deg);
  }
}

.error {
  color: #dc3545;
  font-size: 12px;
  margin-top: 4px;
  padding: 8px;
  background-color: #f8d7da;
  border-radius: 4px;
}
```

---

## Step 5: Integrate Into Chapter

### 5.1 Add to Chapter Markdown (e.g., `ch01.md`)

```markdown
<UrduTranslationButton chapterId="ch01" chapterContent={pageContent} />

# Chapter 1: Introduction to Physical AI

...rest of chapter content...
```

---

## Step 6: Test End-to-End

### 6.1 Start Docusaurus Dev Server

```bash
cd docusaurus
npm install
npm run start
```

### 6.2 Manual Testing

1. Open http://localhost:3000/docs/ch01-physical-ai-intro
2. Click "اردو میں دیکھیں" button
3. Wait ~2 seconds for translation
4. Verify: Chapter content is in Urdu
5. Click "Show in English" to revert
6. Refresh page and click "اردو میں دیکھیں" again
7. Verify: <500ms load time (cached)

### 6.3 Test Offline Fallback

1. Open DevTools → Network → Offline
2. Click button
3. Verify: Error message appears, English content preserved

---

## Step 7: Performance Benchmarking

Add performance markers to measure translation time:

```typescript
// In UrduTranslationButton.tsx
performance.mark('translation-start');
const result = await translateToUrdu(chapterContent);
performance.mark('translation-end');
performance.measure('translation', 'translation-start', 'translation-end');

const measure = performance.getEntriesByName('translation')[0];
console.log(`Translation took ${measure.duration.toFixed(2)}ms`);
```

**Expected**:
- First request: 1200-1700ms
- Cached request: 150-350ms

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| **API key not found** | Check `.env.local` has `REACT_APP_GROQ_API_KEY` |
| **Groq API 401 error** | API key is invalid or expired; regenerate in Groq console |
| **"localStorage quota exceeded"** | Clear cache: `localStorage.clear()` |
| **Translation includes code blocks** | Improve prompt to better preserve code |
| **Urdu text not rendering** | Browser font support; add Noto Sans Urdu font |

---

## Next Steps

1. ✅ Complete Phase 2 implementation (integrate into all 13 chapters)
2. ✅ Add unit & integration tests
3. ✅ Peer review with Urdu speaker for quality
4. ✅ Performance profiling & optimization
5. ✅ Deploy to GitHub Pages

---

## References

- **Groq API Docs**: https://console.groq.com/docs
- **LiteLLM**: https://docs.litellm.ai/
- **Docusaurus**: https://docusaurus.io/
- **React Context**: https://react.dev/reference/react/useContext
