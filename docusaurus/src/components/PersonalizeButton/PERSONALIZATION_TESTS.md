# Personalization Tests: 4 User Types

This document defines the expected behavior for the personalization button when used by 4 different user types.

## Test Setup

Each test follows this pattern:
1. Create a user with a specific profile
2. Navigate to a chapter (e.g., Chapter 4: Gazebo Simulation)
3. Click "Personalize this chapter" button
4. Verify that content transforms according to the user's profile

---

## User Type 1: No Hardware (Cloud-Only User)

### Profile
```json
{
  "hardware_experience": "none",
  "gpu_access": "none",
  "ros2_knowledge": "basic",
  "learning_goal": "hobby",
  "python_level": "beginner",
  "learning_environment": "cloud_only"
}
```

### Expected Behavior

#### Before Personalization
- All content sections visible (both local and cloud instructions)
- Chapter shows all hardware-related sections

#### After Clicking "Personalize this chapter"
1. **Cloud Instructions Section**: Made visible/emphasized
   - Shows: AWS EC2 g4dn.xlarge instance setup instructions
   - Highlights: Cloud deployment steps
   - Warns: "This setup requires internet connectivity"

2. **Local Installation Section**: Hidden or grayed out
   - Section with class `.local-installation-section` becomes hidden
   - Shows message: "Local hardware setup not available for your profile"

3. **Hardware Warning Banner**: Appears
   - Text: "⚠️ You've selected cloud-only learning. Hardware chapters will show cloud alternatives."
   - Color: Yellow/warning color
   - Dismissible: Yes

4. **Visual Indicator**: Shows at bottom
   - "Personalized for: cloud-only beginner"
   - Button changes to green: "Show Original Content"

5. **No Page Reload**
   - URL unchanged: `/docs/ch04-gazebo-simulation`
   - Scroll position preserved
   - Other page elements unaffected

### Acceptance Criteria
- ✅ Cloud instructions visible and emphasized
- ✅ Local installation section hidden
- ✅ Warning banner appears
- ✅ Visual indicator shows
- ✅ No page reload
- ✅ Button state toggles correctly

---

## User Type 2: Jetson Owner (Local Hardware User)

### Profile
```json
{
  "hardware_experience": "proficient",
  "gpu_access": "consumer",
  "ros2_knowledge": "intermediate",
  "learning_goal": "career_transition",
  "python_level": "intermediate",
  "learning_environment": "local_preferred"
}
```

### Expected Behavior

#### Before Personalization
- All content sections visible
- Mixed local and cloud content

#### After Clicking "Personalize this chapter"
1. **Local Installation Section**: Emphasized and moved to top
   - Shows: Jetson Orin Nano specific setup instructions
   - Highlights: CUDA 12.2 installation on Jetson
   - Emphasizes: Local GPU acceleration benefits
   - Color: Blue highlight box around section

2. **Cloud Instructions Section**: De-emphasized
   - Still visible but less prominent
   - Shows as: "Alternative: Cloud deployment"
   - Color: Gray/muted

3. **Hardware Performance Section**: Appears
   - Shows: GPU memory requirements (8GB VRAM)
   - Displays: Expected inference latency on Jetson
   - Links: TensorRT optimization guides

4. **Deployment Steps**: Customized
   - Shows local model deployment to Jetson Orin
   - Includes: Docker container instructions for Jetson
   - Provides: Performance monitoring commands (tegrastats)

5. **Visual Indicator**: Shows at bottom
   - "Personalized for: local-preferred proficient"
   - Button changes to green: "Show Original Content"

### Acceptance Criteria
- ✅ Local installation section emphasized and top-positioned
- ✅ Jetson-specific instructions visible
- ✅ GPU performance information shown
- ✅ Deployment customized for Jetson
- ✅ Cloud option still available but de-emphasized
- ✅ No page reload
- ✅ Button state toggles correctly

---

## User Type 3: Beginner (Simplified Content)

### Profile
```json
{
  "hardware_experience": "none",
  "gpu_access": "none",
  "ros2_knowledge": "none",
  "learning_goal": "academic",
  "python_level": "beginner",
  "learning_environment": "cloud_preferred"
}
```

### Expected Behavior

#### Before Personalization
- Complex content with advanced terminology
- Assumes ROS 2 knowledge
- Code examples without comments

#### After Clicking "Personalize this chapter"
1. **ROS 2 Primer Section**: Added at top
   - Title: "🚀 Quick ROS 2 Primer"
   - Content: Defines key terms (Node, Topic, Service, Action)
   - Includes: Links to beginner tutorials
   - Color: Light blue background

2. **Simplified Explanations**: Added to technical sections
   - Below each complex code block: "What this code does..."
   - Simplified language for concepts
   - Links to definitions and tutorials

3. **Advanced Sections**: Hidden
   - Section with class `.advanced-ros2-section` becomes hidden
   - Show message: "Advanced topics available after you learn the basics"
   - Message: "Complete foundational chapters first"

4. **Glossary Links**: Added
   - Hover over technical terms shows definitions
   - Links to relevant tutorial sections
   - Example: "ROS 2" links to "What is ROS 2?" section

5. **Learning Path**: Displayed
   - Shows: "Recommended learning order"
   - Suggests: "Chapter 2 before Chapter 4"
   - Progress indicator: "You're at step 3 of 8"

6. **Visual Indicator**: Shows at bottom
   - "Personalized for: beginner friendly"
   - Button changes to green: "Show Original Content"

### Acceptance Criteria
- ✅ ROS 2 Primer section appears
- ✅ Simplified explanations added to code
- ✅ Advanced sections hidden
- ✅ Glossary links functional
- ✅ Learning path suggested
- ✅ No page reload
- ✅ Content becomes clearer and more accessible

---

## User Type 4: Expert (Full Depth)

### Profile
```json
{
  "hardware_experience": "expert",
  "gpu_access": "highend",
  "ros2_knowledge": "advanced",
  "learning_goal": "professional",
  "python_level": "expert",
  "learning_environment": "local_only"
}
```

### Expected Behavior

#### Before Personalization
- Standard chapter content (mixed beginner-expert)

#### After Clicking "Personalize this chapter"
1. **Advanced Content Unlocked**: Additional sections appear
   - Section with class `.advanced-config` becomes visible
   - Shows: GPU optimization parameters
   - Includes: Custom kernel compilation for Jetson
   - Provides: Performance tuning options

2. **Expert-Level Code Examples**: Shown
   - Adds: Async/await patterns
   - Shows: Pythonic alternatives (list comprehensions, generators)
   - Includes: Performance benchmarking code
   - Assumes: Python 3.10+ knowledge

3. **Research Papers & References**: Linked
   - Adds: Links to relevant academic papers
   - Shows: Latest research implementations
   - Provides: GitHub links to advanced projects

4. **Performance Optimization**: Section appears
   - Shows: Memory profiling techniques
   - Includes: GPU memory management
   - Provides: Benchmarking scripts

5. **Contributing Guidelines**: Links appear
   - Points to: Project contribution repositories
   - Shows: How to extend this chapter
   - Provides: Links to development setup

6. **Beginner Content**: Hidden
   - ROS 2 Primer section hidden
   - Basic explanations hidden
   - Learning path hidden

7. **Visual Indicator**: Shows at bottom
   - "Personalized for: advanced expert"
   - Button changes to green: "Show Original Content"

### Acceptance Criteria
- ✅ Advanced sections visible
- ✅ Expert-level code examples shown
- ✅ Research papers linked
- ✅ Performance optimization details provided
- ✅ Contributing guidelines visible
- ✅ Beginner content hidden
- ✅ No page reload
- ✅ Content appropriate for advanced developers

---

## Cross-Test Criteria (All User Types)

### Performance
- ✅ Button click to content transformation: <500ms
- ✅ No browser console errors
- ✅ No network requests for content (uses local state)
- ✅ React DevTools Profiler shows <100ms render time

### Functionality
- ✅ "Show Original Content" button reverts changes instantly
- ✅ Clicking "Personalize" again re-applies transformations
- ✅ URL remains unchanged throughout
- ✅ Scroll position preserved during transformation
- ✅ Other page elements (header, footer, nav) unaffected

### Accessibility
- ✅ Button keyboard accessible (Tab, Enter/Space)
- ✅ Screen reader announces: "Button: Personalize this chapter"
- ✅ Visual indicator readable (good contrast)
- ✅ Mobile responsive (button visible on small screens)

### Browser Compatibility
- ✅ Chrome/Edge (latest)
- ✅ Firefox (latest)
- ✅ Safari (latest)
- ✅ Mobile browsers (iOS Safari, Chrome Android)

---

## Manual Testing Steps

### Test 1: Cloud-Only User
```
1. Login with cloud-only profile
2. Navigate to Chapter 4 (Gazebo Simulation)
3. Click "Personalize this chapter" button
4. Verify cloud instructions visible and emphasized
5. Verify local installation hidden
6. Verify no page reload (URL unchanged)
7. Click "Show Original Content"
8. Verify all sections reappear
```

### Test 2: Jetson Owner
```
1. Login with local-preferred + proficient hardware profile
2. Navigate to Chapter 6 (Isaac Sim)
3. Click "Personalize this chapter" button
4. Verify local instructions emphasized
5. Verify Jetson-specific steps shown
6. Verify GPU performance info visible
7. Click "Show Original Content"
8. Verify all sections reappear
```

### Test 3: Beginner
```
1. Login with beginner profile (none/none/none)
2. Navigate to Chapter 2 (ROS 2 Fundamentals)
3. Click "Personalize this chapter" button
4. Verify ROS 2 Primer section appears
5. Verify advanced sections hidden
6. Verify simplified explanations added
7. Check DevTools: verify <500ms transformation
8. Click "Show Original Content"
9. Verify all sections reappear
```

### Test 4: Expert
```
1. Login with expert profile (expert/advanced/expert)
2. Navigate to Chapter 10 (Manipulation)
3. Click "Personalize this chapter" button
4. Verify advanced content visible
5. Verify research papers linked
6. Verify beginner content hidden
7. Verify expert-level code examples shown
8. Click "Show Original Content"
9. Verify all sections reappear
```

---

## Summary

- **4 user types tested**: Cloud-only, Jetson owner, Beginner, Expert
- **13 chapters affected**: All chapters have PersonalizeButton component
- **Key success metrics**:
  - <500ms transformation time
  - No page reload
  - Correct content visibility per profile
  - Accessibility standards met
  - Cross-browser compatibility verified