import React, { useState, useEffect, useRef } from 'react';
import styles from './styles.module.css';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  citations?: Array<{
    chapter_id: string;
    chapter_title: string;
    similarity: number;
  }>;
}

interface RagChatbotProps {
  backendUrl?: string;
}

// Get API URL - automatically detect based on environment (same pattern as UrduTranslationButton)
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

export default function RagChatbot({ backendUrl }: RagChatbotProps) {
  // Use provided backendUrl or auto-detect
  const apiUrl = backendUrl || getAPIUrl();
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isOpen, setIsOpen] = useState(false);
  const [isStreaming, setIsStreaming] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Handle text selection
  useEffect(() => {
    const handleSelectionChange = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();
      if (text && text.length > 10) {
        setSelectedText(text);
      }
    };

    document.addEventListener('selectionchange', handleSelectionChange);
    return () => document.removeEventListener('selectionchange', handleSelectionChange);
  }, []);

  // Auto-scroll to bottom
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const sendMessage = async () => {
    if (!input.trim()) return;

    const userMessage: Message = { role: 'user', content: input };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsStreaming(true);

    try {
      console.log('[Chatbot] Sending request to:', `${apiUrl}/query`);

      const response = await fetch(`${apiUrl}/query`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          message: input,
          selected_text: selectedText || undefined
        })
      });

      console.log('[Chatbot] Response status:', response.status);

      if (!response.ok) throw new Error(`API request failed with status ${response.status}`);

      const reader = response.body?.getReader();
      const decoder = new TextDecoder();
      let assistantMessage: Message = { role: 'assistant', content: '', citations: [] };
      setMessages(prev => [...prev, assistantMessage]);

      if (reader) {
        while (true) {
          const { done, value } = await reader.read();
          if (done) break;

          const chunk = decoder.decode(value);
          const lines = chunk.split('\n');

          for (const line of lines) {
            if (line.startsWith('data: ')) {
              try {
                const data = JSON.parse(line.slice(6));

                if (data.type === 'chunk') {
                  assistantMessage.content += data.content;
                  setMessages(prev => {
                    const updated = [...prev];
                    updated[updated.length - 1] = { ...assistantMessage };
                    return updated;
                  });
                } else if (data.type === 'citations') {
                  assistantMessage.citations = data.data;
                  setMessages(prev => {
                    const updated = [...prev];
                    updated[updated.length - 1] = { ...assistantMessage };
                    return updated;
                  });
                } else if (data.type === 'done') {
                  // Stream completed successfully
                } else if (data.type === 'error') {
                  console.error('[Chatbot] Server error:', data.content || data.message);
                  assistantMessage.content = data.content || data.message || 'Server error occurred';
                  setMessages(prev => {
                    const updated = [...prev];
                    updated[updated.length - 1] = { ...assistantMessage };
                    return updated;
                  });
                }
                // Ignore tool_call type - it's just a status update
              } catch (parseError) {
                // Only log if it's not an empty line
                if (line.slice(6).trim()) {
                  console.warn('[Chatbot] Could not parse SSE line:', line);
                }
              }
            }
          }
        }
      }

      setSelectedText(''); // Clear selected text after query
    } catch (error) {
      console.error('[Chatbot] Error:', error);
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: 'Sorry, there was an error processing your request. Please try again.'
      }]);
    } finally {
      setIsStreaming(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <>
      {/* Chat Button */}
      <button
        className={styles.chatButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Open AI Chatbot"
      >
        🤖
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <h3>🤖 Physical AI Chatbot</h3>
            <button onClick={() => setIsOpen(false)} className={styles.closeButton}>
              ×
            </button>
          </div>

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <p>👋 Ask me anything about:</p>
                <ul>
                  <li>ROS 2 fundamentals</li>
                  <li>Isaac Sim & Gazebo</li>
                  <li>Humanoid robots</li>
                  <li>Hardware setup</li>
                </ul>
                {selectedText && (
                  <div className={styles.contextBadge}>
                    📝 Selected text detected
                  </div>
                )}
              </div>
            )}

            {messages.map((msg, idx) => (
              <div key={idx} className={`${styles.message} ${styles[msg.role]}`}>
                <div className={styles.messageContent}>{msg.content}</div>
                {msg.citations && msg.citations.length > 0 && (
                  <div className={styles.citations}>
                    {msg.citations.map((citation, cidx) => (
                      <a
                        key={cidx}
                        href={`/docs/${citation.chapter_id}`}
                        className={styles.citationLink}
                        target="_blank"
                        rel="noopener noreferrer"
                      >
                        📖 {citation.chapter_title}
                      </a>
                    ))}
                  </div>
                )}
              </div>
            ))}

            {isStreaming && (
              <div className={styles.typingIndicator}>
                <span></span>
                <span></span>
                <span></span>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <div className={styles.inputContainer}>
            {selectedText && (
              <div className={styles.selectedTextBadge}>
                ✂️ Using selected text
                <button onClick={() => setSelectedText('')}>×</button>
              </div>
            )}
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask about ROS 2, Isaac Sim, humanoid robots..."
              className={styles.input}
              disabled={isStreaming}
            />
            <button
              onClick={sendMessage}
              disabled={!input.trim() || isStreaming}
              className={styles.sendButton}
            >
              {isStreaming ? '⏳' : '➤'}
            </button>
          </div>
        </div>
      )}
    </>
  );
}
