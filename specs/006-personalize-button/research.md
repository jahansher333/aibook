# Research: Personalize Button System Implementation

## Overview
Research into the technical implementation of the personalization system for the "Personalize Button per Chapter" feature, focusing on client-side transformation with React state management.

## Decision: Client-Side Personalization Architecture

### Rationale
Client-side personalization was chosen to meet the critical requirement of "no page reload" while achieving <500ms transformation time. This approach allows content to be transformed instantly using React state updates without network round-trips for the transformation itself.

### Technical Implementation
- Content transformation occurs in the browser using JavaScript
- Profile data is cached in React Context and localStorage after initial fetch
- Transformation rules are applied using DOM manipulation or React component re-rendering
- State is preserved (scroll position, form inputs) during transformation

### Alternatives Considered
1. **Server-Side Rendering**: Would require page reloads, violating the spec requirement
2. **Static Site Generation**: Doesn't allow real-time personalization based on user profile
3. **Hybrid Approach**: Fetch personalized content from server - adds network latency and complexity

## Decision: Profile Data Management

### Rationale
User profile data must be readily available when the personalization button is clicked to meet the 500ms transformation requirement. Caching profile data in React Context with localStorage fallback ensures immediate availability.

### Implementation Strategy
- On user login, fetch profile from Better-Auth API
- Store profile in React Context for immediate access across components
- Cache profile in localStorage for persistence across sessions
- Implement fallback mechanism to fetch profile if not in cache when personalization is triggered

### Performance Considerations
- Initial profile fetch happens during login flow (not during personalization)
- Subsequent access is from memory (React Context) or localStorage
- Network calls only occur if cache is missing/expired

## Decision: Rule Configuration Format

### Rationale
To meet the maintainability requirement (SC-012), personalization rules must be defined in a configuration format rather than hardcoded in components. YAML was selected for its readability and ease of modification by non-technical team members.

### Rule Structure
Each rule follows the pattern:
```yaml
- rule_id: unique_identifier
  profile_key: user_profile_field_name
  profile_value: expected_value_or_array_of_values
  content_selector: CSS_selector_for_content
  transformation_type: show|hide|emphasize|replace_text|add_content
  parameters: optional_additional_parameters
```

### Rule Evaluation Process
1. Load rules from YAML configuration
2. Get current user profile from Context/cache
3. For each rule, check if profile matches rule conditions
4. Apply matching transformations to content
5. Update React state to trigger re-render

## Decision: Content Transformation Approach

### Rationale
The transformation must be fast, maintainable, and preserve existing functionality like Urdu translation. A CSS class-based approach with conditional rendering provides the best balance of performance and maintainability.

### Implementation Options
1. **CSS Classes**: Add/remove CSS classes to show/hide content sections
   - Pros: Fast, doesn't change DOM structure, compatible with other features
   - Cons: Content still exists in DOM even when hidden
2. **React Conditional Rendering**: Conditionally render components based on profile
   - Pros: Clean DOM, true content removal
   - Cons: Requires MDX changes to all chapters
3. **DOM Manipulation**: Directly modify DOM elements based on selectors
   - Pros: Works with existing content structure
   - Cons: Less React-like, harder to debug

### Chosen Approach: Hybrid Method
- Use React conditional rendering where possible (requires minimal MDX changes)
- Use CSS class toggling for complex content sections
- Preserve original content in state to allow toggling back to original view

## Decision: Integration with Urdu Translation

### Rationale
The system must maintain Urdu translation functionality after personalization (FR-010, SC-007). This requires that translated content be generated from the personalized version, not the original.

### Technical Implementation
1. When personalization occurs, generate a profile-specific content variant
2. Use profile_hash in cache key for Urdu translation: `chapter_id + profile_hash + "urdu"`
3. When toggling back to original content, switch to original cache key
4. Maintain separate translation caches for personalized vs original content

### Cache Key Strategy
- Original content: `ch04_original_urdu`
- Personalized content: `ch04_personalized_{profile_hash}_urdu`
- This ensures personalized content gets translated appropriately

## Decision: State Management Architecture

### Rationale
The system must preserve scroll position, form state, and URL during personalization (FR-007). Proper React state management with context ensures these requirements are met.

### State Structure
```typescript
interface PersonalizationState {
  isPersonalized: boolean;
  originalContent: string; // Cached original for toggling
  transformedContent: string; // Personalized content
  profileSummary: string; // For visual indicator
  appliedRules: string[]; // For debugging and tracking
  loading: boolean; // For UI feedback
  error: string | null; // For error handling
}
```

### State Flow
1. User clicks "Personalize this chapter"
2. Set loading state, show spinner on button
3. Apply transformation rules to content
4. Update state with transformed content
5. Preserve scroll position using scroll restoration
6. Update button state to "Show Original Content"

## Performance Optimization Strategies

### Rule Evaluation Optimization
- Pre-filter rules based on user's profile values
- Use Map/lookup structures for faster rule matching
- Cache rule evaluation results for repeated transformations

### Content Processing
- Use React.memo to prevent unnecessary re-renders
- Implement virtualization for long chapters
- Lazy-load transformation rules only when needed

### Memory Management
- Clear cached content when navigating away from chapter
- Implement cache size limits to prevent memory bloat
- Use weak references where appropriate

## Accessibility Considerations

### Keyboard Navigation
- Ensure personalization button is focusable via Tab key
- Support Enter/Space for activation
- Maintain focus position after content transformation

### Screen Reader Support
- Use ARIA attributes to announce content changes
- Provide clear labels for the personalization button
- Announce when content has been personalized

### Visual Indicators
- Clear visual feedback when content is personalized
- Maintain color contrast ratios for accessibility
- Provide text-based indicators for users with visual impairments

## Error Handling Strategy

### Profile Loading Errors
- Show fallback personalization based on default profile
- Display user-friendly error message
- Provide option to retry profile loading

### Transformation Errors
- Preserve original content if transformation fails
- Log errors for debugging while maintaining user experience
- Implement timeout for transformation operations

### Network Errors
- Use cached profile if API is unavailable
- Graceful degradation to default personalization
- Clear error messaging for users

## Security Considerations

### Content Injection Prevention
- Validate transformation rules before application
- Sanitize any content that gets added via rules
- Use safe DOM manipulation methods

### Profile Data Protection
- Profile data remains client-side, no additional exposure
- Use existing Better-Auth security measures
- Don't expose profile data to unauthorized users

## Testing Strategy

### Unit Tests
- Rule evaluation logic
- Profile matching functions
- Content transformation utilities

### Integration Tests
- End-to-end personalization flow
- Profile loading and caching
- Integration with Better-Auth

### Performance Tests
- Transformation time measurement
- React rendering performance
- Memory usage under various conditions

### Accessibility Tests
- Keyboard navigation
- Screen reader compatibility
- ARIA attribute validation

## Deployment Considerations

### Client-Side Bundling
- Personalization components add minimal bundle size
- Rules configuration can be loaded asynchronously
- Lazy-loading of components where appropriate

### Caching Strategy
- Session-based caching of personalized content
- No persistent storage of transformed content
- Clear cache on profile update or logout

### Rollback Plan
- Personalization can be disabled without affecting core functionality
- Original content remains accessible
- Simple component removal for quick rollback