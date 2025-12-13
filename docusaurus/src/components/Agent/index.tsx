import React, { JSX, useState } from 'react';
import styles from './Agent.module.css';

interface AgentProps {
  name: string;
  context?: string;
  buttonText?: string;
  inline?: boolean;
}

interface AgentResponse {
  result: string;
  model?: string;
  cached?: boolean;
}

export default function Agent({
  name,
  context = '',
  buttonText,
  inline = false
}: AgentProps): JSX.Element {
  const [response, setResponse] = useState<string>('');
  const [loading, setLoading] = useState<boolean>(false);
  const [error, setError] = useState<string>('');
  const [showModal, setShowModal] = useState<boolean>(false);

  const defaultButtonText = buttonText || `Ask ${name}`;

  const invokeAgent = async () => {
    setLoading(true);
    setError('');
    setResponse('');

    try {
      const apiUrl = process.env.NODE_ENV === 'production'
        ? 'https://jahansher-aibook.hf.space/api/agent'
        : 'http://localhost:8003/api/agent';

      const res = await fetch(apiUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'application/json'
        },
        body: JSON.stringify({
          agent: name,
          context: context || `Use ${name} agent`
        })
      });

      if (!res.ok) {
        const errorData = await res.json();
        throw new Error(errorData.error || `HTTP ${res.status}: ${res.statusText}`);
      }

      const data: AgentResponse = await res.json();
      setResponse(data.result);

      if (!inline) {
        setShowModal(true);
      }
    } catch (err) {
      console.error('[Agent Error]', err);
      setError(err instanceof Error ? err.message : 'Failed to invoke agent');
    } finally {
      setLoading(false);
    }
  };

  const closeModal = () => {
    setShowModal(false);
  };

  return (
    <>
      <div className={styles.agentContainer}>
        <button
          onClick={invokeAgent}
          disabled={loading}
          className={styles.agentButton}
          aria-label={`Invoke ${name} agent`}
        >
          {loading ? (
            <>
              <span className={styles.spinner}></span>
              Loading...
            </>
          ) : (
            defaultButtonText
          )}
        </button>

        {inline && response && (
          <div className={styles.inlineResponse}>
            <div
              className={styles.responseContent}
              dangerouslySetInnerHTML={{ __html: response }}
            />
          </div>
        )}

        {inline && error && (
          <div className={styles.errorMessage}>
            <strong>Error:</strong> {error}
          </div>
        )}
      </div>

      {!inline && showModal && (
        <div className={styles.modalOverlay} onClick={closeModal}>
          <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
            <div className={styles.modalHeader}>
              <h3>{name} Response</h3>
              <button
                className={styles.closeButton}
                onClick={closeModal}
                aria-label="Close modal"
              >
                ×
              </button>
            </div>

            <div className={styles.modalBody}>
              {error ? (
                <div className={styles.errorMessage}>
                  <strong>Error:</strong> {error}
                </div>
              ) : (
                <div
                  className={styles.responseContent}
                  dangerouslySetInnerHTML={{ __html: response }}
                />
              )}
            </div>

            <div className={styles.modalFooter}>
              <button
                className={styles.closeButtonBottom}
                onClick={closeModal}
              >
                Close
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
}
