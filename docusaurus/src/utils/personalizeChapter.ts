/**
 * Utility function to personalize chapter content based on user profile
 * This function transforms content based on the user's profile attributes
 */

import { UserProfile } from '../contexts/UserContext';

interface PersonalizationRule {
  rule_id: string;
  profile_key: keyof UserProfile;
  profile_value: string | string[];
  content_selector: string;
  transformation_type: 'show' | 'hide' | 'emphasize' | 'replace_text' | 'add_content';
  parameters?: Record<string, any>;
}

// Define personalization rules based on the spec
const personalizationRules: PersonalizationRule[] = [
  // Learning Environment Rules
  {
    rule_id: "env-cloud-only-hide-local",
    profile_key: "learning_environment",
    profile_value: ["cloud_only", "cloud_preferred"],
    content_selector: ".local-installation-section",
    transformation_type: "hide"
  },
  {
    rule_id: "env-local-show-local",
    profile_key: "learning_environment",
    profile_value: ["local_preferred", "local_only"],
    content_selector: ".local-installation-section",
    transformation_type: "show"
  },
  {
    rule_id: "env-cloud-show-cloud",
    profile_key: "learning_environment",
    profile_value: ["cloud_only", "cloud_preferred"],
    content_selector: ".cloud-instructions",
    transformation_type: "show"
  },

  // ROS2 Knowledge Rules
  {
    rule_id: "ros2-none-add-primer",
    profile_key: "ros2_knowledge",
    profile_value: "none",
    content_selector: "body",
    transformation_type: "add_content",
    parameters: {
      content: `
        <div class="primer-section">
          <h3>ROS2 Primer</h3>
          <p>For beginners, here are the key concepts you should understand before proceeding...</p>
        </div>
      `,
      position: "prepend"
    }
  },
  {
    rule_id: "ros2-none-hide-advanced",
    profile_key: "ros2_knowledge",
    profile_value: "none",
    content_selector: ".advanced-ros2-section",
    transformation_type: "hide"
  },

  // Hardware Experience Rules
  {
    rule_id: "hw-none-simplify",
    profile_key: "hardware_experience",
    profile_value: ["none", "some"],
    content_selector: ".technical-specs",
    transformation_type: "hide"
  },
  {
    rule_id: "hw-expert-show-advanced",
    profile_key: "hardware_experience",
    profile_value: ["proficient", "expert"],
    content_selector: ".advanced-config",
    transformation_type: "show"
  },

  // GPU Access Rules
  {
    rule_id: "gpu-none-prioritize-cloud",
    profile_key: "gpu_access",
    profile_value: "none",
    content_selector: ".cloud-alternatives",
    transformation_type: "show"
  },
  {
    rule_id: "gpu-none-hide-local-gpu",
    profile_key: "gpu_access",
    profile_value: "none",
    content_selector: ".gpu-intensive-section",
    transformation_type: "hide"
  },

  // Python Level Rules
  {
    rule_id: "python-beginner-add-explanations",
    profile_key: "python_level",
    profile_value: "beginner",
    content_selector: "code",
    transformation_type: "add_content",
    parameters: {
      content: "<div class='python-explanation'>This code example demonstrates...</div>",
      position: "after"
    }
  }
];

/**
 * Applies personalization rules to content based on user profile
 */
export const personalizeChapter = (content: string, profile: UserProfile | null): string => {
  if (!profile) {
    // If no profile, return content as is or apply default rules
    return content;
  }

  // Create a temporary DOM element to manipulate the content
  const tempDiv = document.createElement('div');
  tempDiv.innerHTML = content;

  // Apply each rule that matches the user's profile
  personalizationRules.forEach(rule => {
    // Check if the rule applies to this user's profile
    const profileValue = profile[rule.profile_key];
    const ruleValue = rule.profile_value;

    let ruleApplies = false;

    if (Array.isArray(ruleValue)) {
      ruleApplies = ruleValue.includes(profileValue as string);
    } else {
      ruleApplies = profileValue === ruleValue;
    }

    if (ruleApplies) {
      // Find elements that match the selector
      const elements = tempDiv.querySelectorAll(rule.content_selector);

      elements.forEach(element => {
        switch (rule.transformation_type) {
          case 'hide':
            // Hide the element by adding a CSS class
            element.classList.add('hidden-by-personalization');
            break;

          case 'show':
            // Show the element by removing hidden class
            element.classList.remove('hidden-by-personalization');
            break;

          case 'emphasize':
            // Add emphasis class to highlight important content
            element.classList.add('personalized-emphasis');
            break;

          case 'add_content':
            if (rule.parameters && rule.parameters.content) {
              const newContent = document.createElement('div');
              newContent.innerHTML = rule.parameters.content;

              if (rule.parameters.position === 'prepend') {
                element.insertBefore(newContent, element.firstChild);
              } else if (rule.parameters.position === 'append') {
                element.appendChild(newContent);
              } else if (rule.parameters.position === 'after') {
                element.parentNode?.insertBefore(newContent, element.nextSibling);
              }
            }
            break;

          case 'replace_text':
            // Text replacement would be handled differently
            // This is a simplified implementation
            if (rule.parameters && rule.parameters.text_replacement) {
              element.textContent = element.textContent?.replace(
                rule.parameters.text_replacement.old_text,
                rule.parameters.text_replacement.new_text
              ) || element.textContent;
            }
            break;
        }
      });
    }
  });

  // Return the modified content
  return tempDiv.innerHTML;
};

/**
 * Gets a summary of the user's profile for display purposes
 */
export const getProfileSummary = (profile: UserProfile | null): string => {
  if (!profile) {
    return 'Not logged in';
  }

  return `${profile.learning_environment.replace('_', '-')} ${profile.hardware_experience} ${profile.ros2_knowledge}`;
};

/**
 * Determines if a user has hardware access based on their profile
 */
export const hasHardwareAccess = (profile: UserProfile | null): boolean => {
  if (!profile) {
    return false;
  }

  return profile.hardware_experience !== 'none' || profile.gpu_access !== 'none';
};

/**
 * Determines if a user is a beginner based on their profile
 */
export const isBeginnerUser = (profile: UserProfile | null): boolean => {
  if (!profile) {
    return true; // Default to beginner if no profile
  }

  return profile.ros2_knowledge === 'none' || profile.python_level === 'beginner';
};

/**
 * Determines if a user is an expert based on their profile
 */
export const isExpertUser = (profile: UserProfile | null): boolean => {
  if (!profile) {
    return false;
  }

  return profile.ros2_knowledge === 'advanced' && profile.python_level === 'expert';
};