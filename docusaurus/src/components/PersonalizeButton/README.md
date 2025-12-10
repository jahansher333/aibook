# PersonalizeButton Component

A React component that allows users to personalize chapter content based on their user profile. The button triggers instant content transformation without page reloads, providing a tailored learning experience for different user backgrounds.

## Features

- **Instant Content Transformation**: Click-based personalization without page reload
- **Profile-Based Rules**: Content adapts based on 7 profile dimensions
- **Visual Feedback**: Loading states and personalization indicators
- **Accessibility**: Full keyboard and screen reader support
- **Mobile Responsive**: Works on all device sizes

## Installation

The component is already integrated into all 13 chapters of the textbook.

## Usage

```tsx
import PersonalizeButton from '@site/src/components/PersonalizeButton/PersonalizeButton';

<PersonalizeButton
  chapterId="ch04"
  chapterContent="Gazebo Simulation"
/>
```

### Props

| Prop | Type | Required | Description |
|------|------|----------|-------------|
| `chapterId` | string | Yes | Unique identifier for the chapter (e.g., "ch04") |
| `chapterContent` | string | Yes | Display name for the chapter content |

## User Profiles

The personalization system works with 7 profile dimensions:

### 1. Hardware Experience
- `none`: No hardware background
- `some`: Limited hardware experience
- `proficient`: Good hardware knowledge
- `expert`: Extensive hardware expertise

### 2. GPU Access
- `none`: No GPU available
- `consumer`: Consumer-grade GPU (RTX 3070)
- `midrange`: Mid-range GPU (A100)
- `highend`: High-end GPU (H100, cluster)

### 3. ROS 2 Knowledge
- `none`: New to ROS 2
- `basic`: Basic understanding
- `intermediate`: Comfortable with ROS 2
- `advanced`: Expert-level ROS 2 knowledge

### 4. Learning Goal
- `academic`: Academic/research focus
- `hobby`: Hobby/recreational learning
- `career_transition`: Career development
- `professional`: Professional/industry focus

### 5. Python Level
- `beginner`: Learning Python
- `intermediate`: Intermediate Python skills
- `advanced`: Advanced Python
- `expert`: Expert-level Python developer

### 6. Learning Environment
- `cloud_only`: Cloud-based preferred
- `cloud_preferred`: Cloud with local backup
- `local_preferred`: Local with cloud backup
- `local_only`: Local-only setup

### 7. Timestamp
- ISO 8601 format for profile creation date

## Personalization Rules

Personalization rules are defined in `src/utils/personalizeChapter.ts` and include:

### Learning Environment Rules
- Cloud users: Hide local installation sections, emphasize AWS/cloud instructions
- Local users: Show local installation first, emphasize hardware requirements

### ROS 2 Knowledge Rules
- Beginners: Add ROS 2 primer, hide advanced concepts
- Intermediate: Show standard content with links to deeper docs
- Advanced: Show advanced patterns and research implementations

### Hardware Experience Rules
- No experience: Simplify technical specs, hide advanced customization
- Experienced: Show advanced configuration options

### Python Level Rules
- Beginners: Add Python explanations in code comments
- Intermediate: Standard code examples
- Advanced/Expert: Show Pythonic patterns and modern features

### GPU Access Rules
- No GPU: Prioritize cloud solutions, warn about local performance
- Consumer: Show both local and cloud equally
- High-end: Emphasize GPU-accelerated local setup

## Transformation Types

The system supports 5 transformation types:

1. **Hide**: Remove sections not relevant to user
   ```typescript
   transformation_type: "hide"
   content_selector: ".local-installation-section"
   ```

2. **Show**: Make optional sections visible
   ```typescript
   transformation_type: "show"
   content_selector: ".cloud-instructions"
   ```

3. **Emphasize**: Highlight important sections for user
   ```typescript
   transformation_type: "emphasize"
   content_selector: ".key-concept"
   ```

4. **Add Content**: Inject additional sections
   ```typescript
   transformation_type: "add_content"
   parameters: {
     content: "<div>...</div>",
     position: "prepend" | "append" | "after"
   }
   ```

5. **Replace Text**: Substitute terminology
   ```typescript
   transformation_type: "replace_text"
   parameters: {
     text_replacement: {
       old_text: "pattern",
       new_text: "replacement"
     }
   }
   ```

## Component State Management

The component uses React Context (`UserContext`) to manage:

- `userProfile`: Current user's profile object
- `isPersonalized`: Boolean flag for personalization state
- `isLoading`: Loading state while fetching profile
- `profileSummary`: Human-readable profile summary

## Performance Characteristics

- **Transformation Time**: Target <500ms from click to visible change
- **Render Time**: <100ms React render cycle
- **Network Impact**: Zero network requests (uses cached profile)
- **Bundle Size**: ~15KB gzipped

## Styling

The component uses CSS modules for scoped styling:

```
PersonalizeButton.module.css
├── .personalize_button_container
├── .personalize_button
├── .personalize_button.personalized
├── .personalize_button.processing
├── .spinner
└── .personalization_indicator
```

### CSS Classes for Personalization

Content can be styled with these classes:

```css
.hidden-by-personalization {
  display: none !important;
}

.personalized-emphasis {
  background-color: #fff3cd;
  padding: 0.5rem;
  border-left: 3px solid #ffc107;
}

.primer-section {
  background-color: #f8f9fa;
  border-radius: 4px;
  padding: 1rem;
}
```

## Accessibility

### Keyboard Navigation
- Tab to reach button
- Enter/Space to activate
- Visual focus indicator

### Screen Reader Support
- ARIA labels: "Button: Personalize this chapter"
- Live region announcements on state change
- Proper semantic HTML

### Mobile Support
- Minimum touch target: 44x44px
- Responsive button positioning
- Works with mobile screen readers

## Testing

See `PERSONALIZATION_TESTS.md` for comprehensive test cases covering:

1. **Cloud-Only User**: Validates cloud content emphasis
2. **Jetson Owner**: Validates local/hardware emphasis
3. **Beginner User**: Validates simplified content
4. **Expert User**: Validates advanced content

Run tests with:
```bash
npm test
cypress run
```

## Integration with Docusaurus

The component integrates seamlessly with Docusaurus:

1. **MDX Support**: Works in `.md` and `.mdx` files
2. **Theme Compatibility**: Uses Docusaurus theme colors
3. **Responsive**: Adapts to Docusaurus layout
4. **Dark Mode**: Supports light and dark themes

## Common Issues

### Button Not Showing
- Verify UserProvider wraps the entire app
- Check UserContext is accessible in the component tree
- Verify `userProfile` is not null

### Personalization Not Working
- Check browser console for errors
- Verify profile data format matches types
- Ensure CSS selectors match content elements
- Check rule evaluation logic in `personalizeChapter.ts`

### Performance Issues
- Profile size too large? Consider pagination
- Too many rules? Optimize rule evaluation
- CSS selector performance? Use more specific selectors

## Future Enhancements

1. **Persistence**: Save personalization preferences across sessions
2. **Analytics**: Track which profiles use personalization most
3. **AI-Generated**: LLM-based content rewriting (beyond show/hide)
4. **A/B Testing**: Experiment with different personalization rules
5. **Real-Time Sync**: Update across multiple tabs in real-time

## API Reference

### PersonalizeButton Props

```typescript
interface PersonalizeButtonProps {
  chapterId: string;      // Unique chapter identifier
  chapterContent: string; // Display name for the chapter
}
```

### useUserContext Hook

```typescript
const {
  userProfile,      // Current user profile or null
  isPersonalized,   // Whether content is personalized
  setIsPersonalized, // Function to toggle personalization
  setUserProfile,   // Function to update profile
  isLoading,        // Loading state
  setIsLoading,     // Function to update loading state
  profileSummary    // Human-readable profile summary
} = useUserContext();
```

### personalizeChapter Utility

```typescript
// Apply personalization rules to content
personalizeChapter(content: string, profile: UserProfile | null): string

// Get profile summary
getProfileSummary(profile: UserProfile | null): string

// Check if user has hardware
hasHardwareAccess(profile: UserProfile | null): boolean

// Check if beginner
isBeginnerUser(profile: UserProfile | null): boolean

// Check if expert
isExpertUser(profile: UserProfile | null): boolean
```

## License

This component is part of the Physical AI Textbook and is available under the same license as the main project.

## Support

For issues or questions, please refer to the project repository or contact the maintainers.
