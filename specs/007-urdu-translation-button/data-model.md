# Data Model: Urdu Translation Button

**Feature**: 007-urdu-translation-button
**Date**: 2025-12-10
**Purpose**: Define entities, relationships, and state transitions for translation caching and language management

---

## Entity Definitions

### 1. UrduTranslation

Represents a translated chapter in Urdu, cached locally.

**Storage**: Browser localStorage (key: `urdu_translation_<chapterId>`)

**Fields**:

| Field | Type | Required | Notes |
|-------|------|----------|-------|
| `chapterId` | string | ✅ | Primary identifier: "ch01", "ch02", ... "ch13" |
| `originalContent` | string | ✅ | Full chapter HTML before translation (for reference) |
| `originalContentHash` | string (SHA-256) | ✅ | Hash of originalContent; detect changes |
| `urduTranslation` | string | ✅ | Full chapter translated to Urdu |
| `timestamp` | ISO 8601 | ✅ | When translation was created (e.g., "2025-12-10T18:30:00Z") |
| `expiryTime` | ISO 8601 | ✅ | When cache expires, TTL = 30 days (e.g., "2026-01-09T18:30:00Z") |
| `translationDuration` | number (ms) | ❌ | How long Groq API took to translate (metadata for performance tracking) |

**Example**:
```json
{
  "chapterId": "ch01",
  "originalContent": "<h1>Chapter 1: Introduction to Physical AI</h1>\n<p>Physical AI...</p>",
  "originalContentHash": "sha256_abc123def456",
  "urduTranslation": "<h1>باب ۱: جسمانی AI کا تعارف</h1>\n<p>جسمانی AI...</p>",
  "timestamp": "2025-12-10T18:30:00Z",
  "expiryTime": "2026-01-09T18:30:00Z",
  "translationDuration": 1200
}
```

**Constraints**:
- `chapterId` must be one of: "ch01" through "ch13" (validated)
- `urduTranslation` must not be empty
- `expiryTime` must be > `timestamp`
- `originalContentHash` used to detect chapter updates (if chapter content changes, hash changes, cache is invalidated)

---

### 2. LanguageState

Represents the current display language preference for a chapter (session-level, not persisted).

**Storage**: React Context (LanguageContext)
**Scope**: Session (reset on page reload)

**Structure**:
```typescript
Map<chapterId: string, isUrdu: boolean>
```

**Example**:
```typescript
{
  "ch01": true,   // ch01 is in Urdu
  "ch02": false,  // ch02 is in English
  "ch03": true    // ch03 is in Urdu
}
```

**Behavior**:
- Default: All chapters start as `isUrdu = false` (English)
- Update: When user clicks UrduTranslationButton, toggle `isUrdu` for that `chapterId`
- Scope: Per-session (localStorage NOT used; resets on page reload)
- Rationale: Users navigate between chapters; each chapter can independently show English or Urdu

**Constraints**:
- `chapterId` must exist (one of ch01-ch13)
- `isUrdu` is boolean (no other values)

---

### 3. CacheEntry

Internal representation of cached translation in localStorage.

**Storage**: Browser localStorage
**Key Format**: `urdu_translation_<chapterId>` (e.g., `urdu_translation_ch01`)
**Value**: Serialized UrduTranslation (JSON)

**Operations**:

| Operation | Input | Output | Notes |
|-----------|-------|--------|-------|
| **Get** | `chapterId` | `UrduTranslation \| null` | Returns null if key doesn't exist or is expired |
| **Set** | `chapterId`, `UrduTranslation` | `void` | Writes to localStorage; throws QuotaExceededError if quota exceeded |
| **Delete** | `chapterId` | `void` | Removes from localStorage (cleanup) |
| **IsStale** | `chapterId` | `boolean` | Returns true if expiryTime < now |

**Error Handling**:
- **QuotaExceededError**: localStorage is full; fall back to session memory
- **JSON.parse() error**: Corrupted cache entry; delete and re-fetch from API
- **Missing key**: Return null; treat as cache miss

---

## Relationships

**Entity Relationship Diagram**:

```
┌──────────────┐
│   Chapter    │
│  (ch01-13)   │
└──────┬───────┘
       │
       │ 1:1 (zero or one translation)
       │
       ▼
┌──────────────────────────┐
│  UrduTranslation         │
│  (stored in localStorage)│
└──────┬───────────────────┘
       │
       │ maps to via LanguageState
       │
       ▼
┌──────────────┐
│ LanguageState│
│ (isUrdu flag)│
└──────────────┘
```

**Cardinality**:
- **Chapter → UrduTranslation**: 1:0..1 (every chapter may have 0 or 1 cached translation)
- **Chapter → LanguageState**: 1:1 (every chapter has exactly one language state during session)

**Data Flow**:
1. User opens chapter (English displayed by default)
2. User clicks UrduTranslationButton
3. Check cache for `urdu_translation_ch01`:
   - **Cache hit** (and not stale): Retrieve UrduTranslation, set `LanguageState[ch01] = true`, display Urdu
   - **Cache miss** or **stale**: Call Groq API, store result in cache, set LanguageState, display Urdu
   - **API error**: Set LanguageState to false, display error notification, keep English

---

## State Transitions

### Translation Lifecycle

```
START (English displayed)
  │
  ├─ User clicks button
  │   │
  │   ├─ Check cache for chapterId
  │   │   │
  │   │   ├─ Cache hit + not stale
  │   │   │   └─→ RENDERING (display loading spinner)
  │   │   │       └─→ Retrieve UrduTranslation from localStorage (~10-50ms)
  │   │   │           └─→ Set LanguageState[chapterId] = true
  │   │   │               └─→ Render Urdu content (~100-300ms)
  │   │   │                   └─→ END (Urdu displayed) [<500ms total]
  │   │   │
  │   │   ├─ Cache miss or stale
  │   │   │   └─→ API_CALLING
  │   │   │       └─→ Show loading spinner, disable button
  │   │   │           └─→ Call Groq API with prompt + chapter content (~1200-1700ms)
  │   │   │               ├─ Success (receive Urdu translation)
  │   │   │               │   └─→ Store UrduTranslation in localStorage
  │   │   │               │       └─→ Set LanguageState[chapterId] = true
  │   │   │               │           └─→ Render Urdu content
  │   │   │               │               └─→ END (Urdu displayed) [~2s total]
  │   │   │               │
  │   │   │               └─ Failure (API error, network down)
  │   │   │                   └─→ Log error, show error notification
  │   │   │                       └─→ LanguageState[chapterId] remains false
  │   │   │                           └─→ END (English displayed, error message)
```

### Language Toggle State Machine

```
┌─────────────┐
│   English   │   (isUrdu = false)
│ (Initial)   │
└──────┬──────┘
       │ [button click]
       │ [if Urdu translation available]
       │
       ▼
┌──────────────┐
│   Urdu       │   (isUrdu = true)
│ (Translated) │
└──────┬───────┘
       │ [button click]
       │
       ▼
┌─────────────┐
│   English   │   (isUrdu = false)
│ (Reverted)  │
└─────────────┘
```

**Rules**:
- State is per-chapter (ch01 can be Urdu while ch02 is English)
- State resets on page reload (session-only)
- Toggling is instant (no API call, just re-render with different data)
- If translation is not available (API error, offline), state remains English

---

## Cache Invalidation Strategy

### TTL-Based Expiry (Automatic)

**Trigger**: Read cache entry

**Logic**:
```typescript
function isCacheValid(entry: UrduTranslation): boolean {
  const now = new Date();
  const expiryTime = new Date(entry.expiryTime);
  return expiryTime > now;
}
```

**Effect**: Entries older than 30 days are automatically re-fetched from API on next button click.

### Content-Change Detection (Manual)

**Trigger**: Developer updates chapter content

**Process**:
1. Docusaurus chapter file updated (e.g., `ch01.md` modified)
2. Next build computes new content hash
3. On user's next button click, frontend checks: `originalContentHash` in cache vs. new chapter hash
4. If hashes differ: Cache is stale (even if not 30 days old)
5. Action: Delete cache entry, re-fetch from API

**Implementation**:
```typescript
function isCacheValid(entry: UrduTranslation, currentContent: string): boolean {
  const currentHash = sha256(currentContent);
  return entry.originalContentHash === currentHash && !isExpired(entry);
}
```

### Manual Cleanup

Developer can manually delete cache entries if needed:
```typescript
function clearCache(chapterId: string) {
  localStorage.removeItem(`urdu_translation_${chapterId}`);
}
```

---

## Storage Constraints & Quotas

### Browser localStorage Limits

| Browser | Limit | Notes |
|---------|-------|-------|
| Chrome | 10 MB | Per origin (domain) |
| Firefox | 10 MB | Per origin |
| Safari | 5 MB | Per origin |
| Edge | 10 MB | Per origin |

### Capacity Planning

**Per-chapter translation**:
- Typical chapter: 2,000-5,000 words
- Urdu UTF-8: ~1 byte per character + diacritics
- Estimate: 20-25 KB per chapter (after gzip would be ~5-8 KB)

**Total for 13 chapters**:
- Uncompressed: 13 × 25 KB = 325 KB
- With metadata (JSON overhead): ~400 KB
- **Usage**: <1 MB (well under 5-10 MB limit)

**Safety Margin**: Keep usage <2 MB to allow other app data.

### Quota Exceeded Handling

**Scenario**: localStorage is full, cannot write new translation

**Fallback**:
1. Catch `QuotaExceededError` when writing
2. Log error, attempt cleanup (delete oldest 3 translations)
3. Retry write
4. If still fails: Use session memory (JavaScript object, cleared on reload)
5. Notify user: "Translation cached in session memory only. Will refresh on page reload."

---

## Validation Rules

### UrduTranslation Validation

```typescript
function validateUrduTranslation(obj: any): obj is UrduTranslation {
  return (
    typeof obj.chapterId === 'string' && /^ch\d{2}$/.test(obj.chapterId) &&
    typeof obj.originalContent === 'string' && obj.originalContent.length > 0 &&
    typeof obj.originalContentHash === 'string' && obj.originalContentHash.length === 64 && // SHA-256 hex
    typeof obj.urduTranslation === 'string' && obj.urduTranslation.length > 0 &&
    new Date(obj.timestamp).getTime() > 0 &&
    new Date(obj.expiryTime).getTime() > new Date(obj.timestamp).getTime()
  );
}
```

### LanguageState Validation

```typescript
function validateLanguageState(state: Map<string, boolean>): boolean {
  const validChapterIds = ['ch01', 'ch02', ..., 'ch13'];
  for (const [chapterId, isUrdu] of state.entries()) {
    if (!validChapterIds.includes(chapterId) || typeof isUrdu !== 'boolean') {
      return false;
    }
  }
  return true;
}
```

---

## Extensibility & Future Enhancements

### Server-Side Caching (Phase 2)

Future: Move cache to backend (Qdrant or PostgreSQL) for:
- Cross-device sync (user's translation preferences follow them)
- Analytics (track which chapters users translate)
- Shared cache (multiple users benefit from same translation)

**Migration**: Add optional `cacheSource` field to UrduTranslation:
```typescript
type CacheSource = 'localStorage' | 'server';
interface UrduTranslation {
  ...
  cacheSource: CacheSource;
}
```

### Multi-Language Support (Phase 2+)

Current design is Urdu-specific. To support other languages:
- Add `language` field to LanguageState: `Map<chapterId + language, translation>`
- Extend cache key: `translation_<chapterId>_<languageCode>` (e.g., `translation_ch01_ar` for Arabic)
- Generalize prompt template for any language

---

## Summary

| Aspect | Design | Rationale |
|--------|--------|-----------|
| **Storage** | Browser localStorage (optional backend future) | Fast, distributed, offline-capable |
| **Cache Key** | `urdu_translation_<chapterId>` | Per-chapter isolation, simple lookup |
| **TTL** | 30 days | Balances freshness (updates) with performance (fewer API calls) |
| **Language State** | Context-based, session-only | Per-chapter flexibility, no persistence overhead |
| **Fallback** | Session memory if localStorage full | Degrades gracefully, no data loss |
| **Validation** | Content hash + TTL check | Detects stale or modified chapters |

**Ready for Phase 2: Implementation Tasks** ✅
