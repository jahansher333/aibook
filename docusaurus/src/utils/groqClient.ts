/**
 * Groq API client for Urdu translation
 * Uses Groq's llama-3.1-70b-instant model for fast, high-quality translations
 */

export interface TranslationResult {
  urdu: string;
  duration: number; // milliseconds
  tokensUsed?: number;
}

export interface GroqError {
  error?: {
    message: string;
    type: string;
  };
  message?: string;
  status?: number;
}

const API_URL =
  process.env.REACT_APP_GROQ_API_URL ||
  'https://api.groq.com/openai/v1/chat/completions';
const MODEL = process.env.REACT_APP_GROQ_MODEL || 'llama-3.1-70b-instant';
const API_KEY = process.env.REACT_APP_GROQ_API_KEY;

/**
 * Translate content to Urdu using Groq Llama-3.1-70B
 * @param content - Chapter HTML/Markdown content to translate
 * @returns Promise<TranslationResult> - Translated content and metadata
 * @throws Error on API failure
 */
export async function translateToUrdu(
  content: string
): Promise<TranslationResult> {
  if (!API_KEY) {
    throw new Error(
      'REACT_APP_GROQ_API_KEY environment variable not set. Get your key from https://console.groq.com'
    );
  }

  if (!content || content.trim().length === 0) {
    throw new Error('Content cannot be empty');
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
        model: MODEL,
        messages: [
          {
            role: 'system',
            content:
              'You are a professional technical translator. Translate educational robotics content to Urdu. Preserve code blocks and technical terms in English. Write for Grade 10-12 reading level.',
          },
          {
            role: 'user',
            content: prompt,
          },
        ],
        temperature: 0.3, // Lower temperature for consistency
        max_tokens: 4096,
        top_p: 1.0,
        frequency_penalty: 0.0,
        presence_penalty: 0.0,
      }),
    });

    const duration = Date.now() - startTime;

    if (!response.ok) {
      const errorData: GroqError = await response.json();
      const errorMessage =
        errorData.error?.message || errorData.message || response.statusText;

      if (response.status === 429) {
        throw new Error(`Rate limit exceeded. Please try again later. ${errorMessage}`);
      } else if (response.status >= 500) {
        throw new Error(
          `Groq API temporarily unavailable (${response.status}). Please try again in a few moments.`
        );
      } else {
        throw new Error(`Groq API error (${response.status}): ${errorMessage}`);
      }
    }

    const data = await response.json();

    if (!data.choices || !data.choices[0] || !data.choices[0].message) {
      throw new Error('Unexpected response format from Groq API');
    }

    const urdu = data.choices[0].message.content.trim();

    if (!urdu) {
      throw new Error('Empty response from Groq API');
    }

    return {
      urdu,
      duration,
      tokensUsed: data.usage?.total_tokens,
    };
  } catch (error) {
    const duration = Date.now() - startTime;

    if (error instanceof Error) {
      console.error(`Groq translation error (${duration}ms):`, error.message);
      throw error;
    }

    const unknownError = error as any;
    console.error(
      `Groq translation error (${duration}ms):`,
      unknownError
    );

    throw new Error(
      `Failed to translate content: ${unknownError?.message || 'Unknown error'}`
    );
  }
}

/**
 * Build translation prompt with best practices
 * Instructs model on how to translate while preserving technical content
 * @param content - Chapter content to translate
 * @returns Formatted prompt for Groq API
 */
export function getTranslationPrompt(content: string): string {
  return `Translate the following robotics and AI educational content into natural, easy-to-read Urdu.

IMPORTANT RULES:
1. Keep technical terms in English: ROS 2, Isaac Sim, URDF, Gazebo, TF2, Nav2, SLAM, GPT, Whisper, CLIP, MoveIt, Jetson, Ubuntu, Python, Linux, Docker, C++, etc.
2. Preserve ALL code blocks and command-line examples EXACTLY AS-IS (do NOT translate code, variable names, or keywords)
3. Preserve markdown formatting: #headers, **bold**, *italics*, - lists, [links], ![images]
4. Write for Grade 10-12 reading level (clear, simple Urdu with standard vocabulary)
5. Do not translate function/variable names, library names, or filenames
6. Keep mathematical expressions, formulas, and technical notation unchanged
7. Maintain proper Urdu grammar and punctuation (علامتیں)
8. Translate "robot" as "روبوٹ", "simulation" as "نقل", "sensor" as "حسّاس", "control" as "کنٹرول"

CONTENT TO TRANSLATE:
${content}

TRANSLATED CONTENT (Urdu only, preserve all code and technical terms unchanged):`;
}

/**
 * Validate API connection and key
 * Useful for initial setup verification
 * @returns Promise<boolean> - true if API is reachable and key is valid
 */
export async function validateApiConnection(): Promise<boolean> {
  if (!API_KEY) {
    console.warn('REACT_APP_GROQ_API_KEY not set');
    return false;
  }

  try {
    // Try a minimal API call
    const response = await fetch(API_URL, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${API_KEY}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        model: MODEL,
        messages: [
          {
            role: 'user',
            content: 'Hi',
          },
        ],
        max_tokens: 1,
      }),
    });

    return response.ok || response.status === 429; // 429 = rate limited but key is valid
  } catch (error) {
    console.error('API connection validation failed:', error);
    return false;
  }
}

/**
 * Get API configuration info (for debugging)
 * @returns Object with API configuration details
 */
export function getApiConfig(): {
  model: string;
  url: string;
  keyConfigured: boolean;
} {
  return {
    model: MODEL,
    url: API_URL,
    keyConfigured: !!API_KEY,
  };
}
