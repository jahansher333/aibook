import React, { useState, useEffect, JSX } from 'react';
import styles from './MCQ.module.css';

interface MCQProps {
  id: string;
  question: string;
  options: string[];
  correctIndex: number;
  explanation: string;
  difficulty?: 'easy' | 'medium' | 'hard';
}

export default function MCQ({
  id,
  question,
  options,
  correctIndex,
  explanation,
  difficulty = 'medium',
}: MCQProps): JSX.Element {
  const [selected, setSelected] = useState<number | null>(null);
  const [showFeedback, setShowFeedback] = useState(false);

  // Load saved progress from localStorage on mount
  useEffect(() => {
    try {
      const progress = JSON.parse(localStorage.getItem('physicalai_user_progress') || '{}');
      const chapterId = id.split('-')[0];
      if (progress.chapters?.[chapterId]?.mcqScores?.[id]) {
        const savedAnswer = progress.chapters[chapterId].mcqScores[id];
        setSelected(savedAnswer.selectedIndex);
        setShowFeedback(true);
      }
    } catch (e) {
      console.warn('Failed to load progress from localStorage:', e);
    }
  }, [id]);

  const handleSubmit = () => {
    if (selected !== null) {
      setShowFeedback(true);
      // Save to localStorage
      try {
        const progress = JSON.parse(localStorage.getItem('physicalai_user_progress') || '{}');
        if (!progress.chapters) progress.chapters = {};
        const chapterId = id.split('-')[0];
        if (!progress.chapters[chapterId]) progress.chapters[chapterId] = { mcqScores: {} };
        progress.chapters[chapterId].mcqScores[id] = {
          selectedIndex: selected,
          correct: selected === correctIndex,
          attemptedAt: new Date().toISOString(),
        };
        progress.lastUpdated = new Date().toISOString();
        localStorage.setItem('physicalai_user_progress', JSON.stringify(progress));
      } catch (e) {
        console.warn('Failed to save progress to localStorage:', e);
      }
    }
  };

  const handleKeyDown = (event: React.KeyboardEvent) => {
    if (event.key === 'Enter' && selected !== null && !showFeedback) {
      handleSubmit();
    }
  };

  const isCorrect = selected === correctIndex;

  return (
    <div
      className={styles.mcqContainer}
      data-difficulty={difficulty}
      onKeyDown={handleKeyDown}
    >
      <h3 className={styles.question} id={`question-${id}`}>
        {question}
      </h3>
      <div
        className={styles.optionsGroup}
        role="radiogroup"
        aria-labelledby={`question-${id}`}
        aria-required="true"
      >
        {options.map((option, idx) => {
          const isSelected = selected === idx;
          const isCorrectOption = idx === correctIndex;
          const showCorrect = showFeedback && isCorrectOption;
          const showIncorrect = showFeedback && isSelected && !isCorrectOption;

          return (
            <label
              key={idx}
              className={`${styles.option} ${showCorrect ? styles.correct : ''} ${showIncorrect ? styles.incorrect : ''} ${isSelected ? styles.selected : ''}`}
            >
              <input
                type="radio"
                name={`mcq-${id}`}
                value={idx}
                checked={isSelected}
                onChange={() => setSelected(idx)}
                disabled={showFeedback}
                aria-label={`Option ${String.fromCharCode(65 + idx)}: ${option}`}
              />
              <span className={styles.optionLabel}>
                <span className={styles.optionLetter}>{String.fromCharCode(65 + idx)}.</span>
                <span className={styles.optionText}>{option}</span>
              </span>
            </label>
          );
        })}
      </div>
      <button
        className={styles.submitButton}
        onClick={handleSubmit}
        disabled={selected === null || showFeedback}
        aria-label={showFeedback ? 'Answer submitted' : 'Submit answer'}
      >
        {showFeedback ? 'Submitted' : 'Submit Answer'}
      </button>
      {showFeedback && (
        <div
          className={`${styles.feedback} ${isCorrect ? styles.feedbackCorrect : styles.feedbackIncorrect}`}
          role="alert"
          aria-live="polite"
        >
          <p className={styles.feedbackIcon}>
            {isCorrect ? '✅ Correct!' : '❌ Incorrect'}
          </p>
          <div
            className={styles.explanation}
            dangerouslySetInnerHTML={{ __html: explanation }}
          />
        </div>
      )}
    </div>
  );
}
