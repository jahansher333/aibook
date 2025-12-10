import React, { useState, ReactNode, JSX } from 'react';
import styles from './CollapsibleLab.module.css';

interface CollapsibleLabProps {
  /**
   * Title displayed in the collapse/expand button
   */
  title: string;

  /**
   * Content to display when expanded
   */
  children: ReactNode;

  /**
   * Whether the section should be expanded by default
   * @default false
   */
  defaultOpen?: boolean;

  /**
   * Optional CSS class for custom styling
   */
  className?: string;

  /**
   * Optional callback when toggle state changes
   */
  onToggle?: (isOpen: boolean) => void;
}

export default function CollapsibleLab({
  title,
  children,
  defaultOpen = false,
  className = '',
  onToggle,
}: CollapsibleLabProps): JSX.Element {
  const [isOpen, setIsOpen] = useState<boolean>(defaultOpen);

  const handleToggle = () => {
    const newState = !isOpen;
    setIsOpen(newState);
    if (onToggle) {
      onToggle(newState);
    }
  };

  return (
    <div className={`${styles.collapsibleContainer} ${className}`}>
      <button
        className={styles.toggleButton}
        onClick={handleToggle}
        aria-expanded={isOpen}
        aria-label={`${isOpen ? 'Collapse' : 'Expand'}: ${title}`}
      >
        <span className={styles.icon}>
          {isOpen ? (
            <svg
              className={styles.chevron}
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
            >
              <polyline points="18 15 12 9 6 15"></polyline>
            </svg>
          ) : (
            <svg
              className={styles.chevron}
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
            >
              <polyline points="6 9 12 15 18 9"></polyline>
            </svg>
          )}
        </span>
        <span className={styles.title}>{title}</span>
      </button>

      {isOpen && (
        <div className={styles.content} role="region" aria-label={`${title} content`}>
          {children}
        </div>
      )}
    </div>
  );
}
