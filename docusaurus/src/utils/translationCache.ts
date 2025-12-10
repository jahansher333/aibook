/**
 * Translation caching utilities using browser localStorage
 * Stores translated chapters with TTL validation and content hash checking
 */

export interface UrduTranslation {
  chapterId: string;
  originalContent: string;
  originalContentHash: string;
  urduTranslation: string;
  timestamp: string; // ISO 8601
  expiryTime: string; // ISO 8601
  translationDuration?: number; // milliseconds
}

const CACHE_PREFIX = 'urdu_translation_';
const CACHE_TTL_DAYS = 30;

/**
 * Get translation from cache
 * @param chapterId - Chapter identifier (ch01, ch02, etc.)
 * @returns Cached translation or null if not found/expired/invalid
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
    console.error(`Error reading translation cache for ${chapterId}:`, error);
    return null;
  }
}

/**
 * Store translation in cache
 * @param chapterId - Chapter identifier
 * @param translation - Translation object to cache
 */
export function setTranslation(
  chapterId: string,
  translation: UrduTranslation
): void {
  try {
    const key = `${CACHE_PREFIX}${chapterId}`;
    localStorage.setItem(key, JSON.stringify(translation));
  } catch (error) {
    if (error instanceof DOMException && error.code === 22) {
      // QuotaExceededError
      console.warn(
        'localStorage quota exceeded. Falling back to session memory.'
      );
      // Could implement session memory fallback here
    } else {
      console.error(`Error writing translation cache for ${chapterId}:`, error);
    }
  }
}

/**
 * Clear specific translation from cache
 * @param chapterId - Chapter identifier
 */
export function clearTranslation(chapterId: string): void {
  try {
    localStorage.removeItem(`${CACHE_PREFIX}${chapterId}`);
  } catch (error) {
    console.error(`Error clearing translation cache for ${chapterId}:`, error);
  }
}

/**
 * Check if translation is stale (expired)
 * @param translation - Translation object to check
 * @returns true if expired, false otherwise
 */
export function isStale(translation: UrduTranslation): boolean {
  return new Date(translation.expiryTime) < new Date();
}

/**
 * Generate expiry time (30 days from now)
 * @returns ISO 8601 string representing expiry time
 */
export function getExpiryTime(): string {
  const now = new Date();
  now.setDate(now.getDate() + CACHE_TTL_DAYS);
  return now.toISOString();
}

/**
 * Compute SHA-256 hash of content
 * Uses SubtleCrypto for secure hashing
 * @param content - Content to hash
 * @returns Promise<string> - Hex string of SHA-256 hash
 */
export async function hashContent(content: string): Promise<string> {
  try {
    const encoder = new TextEncoder();
    const data = encoder.encode(content);
    const hashBuffer = await crypto.subtle.digest('SHA-256', data);
    const hashArray = Array.from(new Uint8Array(hashBuffer));
    return hashArray.map((b) => b.toString(16).padStart(2, '0')).join('');
  } catch (error) {
    console.error('Error hashing content:', error);
    // Fallback: return a simple hash if SubtleCrypto fails
    return simpleHash(content);
  }
}

/**
 * Simple hash fallback (not cryptographically secure, for fallback only)
 * @param content - Content to hash
 * @returns Hash string
 */
function simpleHash(content: string): string {
  let hash = 0;
  for (let i = 0; i < content.length; i++) {
    const char = content.charCodeAt(i);
    hash = (hash << 5) - hash + char;
    hash = hash & hash; // Convert to 32bit integer
  }
  return Math.abs(hash).toString(16);
}

/**
 * Validate that cached content matches current content
 * Compares SHA-256 hash to detect if chapter has been updated
 * @param cachedTranslation - Cached translation with originalContentHash
 * @param currentContent - Current chapter content
 * @returns Promise<boolean> - true if hashes match, false if content changed
 */
export async function validateContentHash(
  cachedTranslation: UrduTranslation,
  currentContent: string
): Promise<boolean> {
  try {
    const currentHash = await hashContent(currentContent);
    return cachedTranslation.originalContentHash === currentHash;
  } catch (error) {
    console.error('Error validating content hash:', error);
    return false; // Conservative: assume invalid on error
  }
}

/**
 * Check if cached translation is valid (not expired and content unchanged)
 * @param cachedTranslation - Cached translation
 * @param currentContent - Current chapter content
 * @returns Promise<boolean> - true if valid and fresh, false if stale or invalid
 */
export async function isValidCache(
  cachedTranslation: UrduTranslation,
  currentContent: string
): Promise<boolean> {
  // Check expiry
  if (isStale(cachedTranslation)) {
    return false;
  }

  // Check content hash
  const hashValid = await validateContentHash(
    cachedTranslation,
    currentContent
  );
  return hashValid;
}

/**
 * Clear all translation cache entries
 * Useful for debugging or manual cache reset
 */
export function clearAllTranslations(): void {
  try {
    const keys = Object.keys(localStorage);
    keys.forEach((key) => {
      if (key.startsWith(CACHE_PREFIX)) {
        localStorage.removeItem(key);
      }
    });
    console.log('All translation cache cleared');
  } catch (error) {
    console.error('Error clearing all translations:', error);
  }
}

/**
 * Get cache statistics (for debugging)
 * @returns Object with cache size, entry count, etc.
 */
export function getCacheStats(): {
  totalEntries: number;
  totalSize: number;
  entries: Array<{
    chapterId: string;
    size: number;
    expiryTime: string;
  }>;
} {
  try {
    const keys = Object.keys(localStorage);
    const cacheKeys = keys.filter((key) => key.startsWith(CACHE_PREFIX));

    let totalSize = 0;
    const entries = cacheKeys.map((key) => {
      const data = localStorage.getItem(key);
      const size = data ? data.length : 0;
      totalSize += size;
      const translation = JSON.parse(data || '{}') as UrduTranslation;
      return {
        chapterId: translation.chapterId,
        size,
        expiryTime: translation.expiryTime,
      };
    });

    return {
      totalEntries: cacheKeys.length,
      totalSize,
      entries,
    };
  } catch (error) {
    console.error('Error getting cache stats:', error);
    return { totalEntries: 0, totalSize: 0, entries: [] };
  }
}
