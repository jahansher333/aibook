import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import '@testing-library/jest-dom';
import MCQ from './index';

const defaultProps = {
  id: 'test-mcq-01',
  question: 'What is the default middleware in ROS 2 Humble?',
  options: ['DDS-RTPS', 'ZeroMQ', 'TCP/IP', 'USB'],
  correctIndex: 0,
  explanation: 'Correct! ROS 2 Humble uses <a href="https://example.com">DDS-RTPS</a> as default middleware.',
};

describe('MCQ Component', () => {
  beforeEach(() => {
    localStorage.clear();
  });

  describe('Rendering', () => {
    test('renders question and 4 options', () => {
      render(<MCQ {...defaultProps} />);

      expect(screen.getByText(defaultProps.question)).toBeInTheDocument();
      expect(screen.getByLabelText(/Option A: DDS-RTPS/i)).toBeInTheDocument();
      expect(screen.getByLabelText(/Option B: ZeroMQ/i)).toBeInTheDocument();
      expect(screen.getByLabelText(/Option C: TCP\/IP/i)).toBeInTheDocument();
      expect(screen.getByLabelText(/Option D: USB/i)).toBeInTheDocument();
    });

    test('renders with difficulty badge', () => {
      const { container } = render(<MCQ {...defaultProps} difficulty="hard" />);

      const mcqContainer = container.querySelector('[data-difficulty="hard"]');
      expect(mcqContainer).toBeInTheDocument();
    });
  });

  describe('User Interactions', () => {
    test('submit button is disabled when no option is selected', () => {
      render(<MCQ {...defaultProps} />);

      const submitButton = screen.getByRole('button', { name: /submit answer/i });
      expect(submitButton).toBeDisabled();
    });

    test('submit button is enabled when an option is selected', () => {
      render(<MCQ {...defaultProps} />);

      const optionA = screen.getByLabelText(/Option A: DDS-RTPS/i);
      fireEvent.click(optionA);

      const submitButton = screen.getByRole('button', { name: /submit answer/i });
      expect(submitButton).not.toBeDisabled();
    });

    test('shows correct feedback when correct answer is selected', () => {
      render(<MCQ {...defaultProps} />);

      const correctOption = screen.getByLabelText(/Option A: DDS-RTPS/i);
      fireEvent.click(correctOption);

      const submitButton = screen.getByRole('button', { name: /submit answer/i });
      fireEvent.click(submitButton);

      expect(screen.getByText(/✅ Correct!/i)).toBeInTheDocument();
      expect(screen.getByText(/ROS 2 Humble uses/i)).toBeInTheDocument();
    });

    test('shows incorrect feedback when wrong answer is selected', () => {
      render(<MCQ {...defaultProps} />);

      const wrongOption = screen.getByLabelText(/Option B: ZeroMQ/i);
      fireEvent.click(wrongOption);

      const submitButton = screen.getByRole('button', { name: /submit answer/i });
      fireEvent.click(submitButton);

      expect(screen.getByText(/❌ Incorrect/i)).toBeInTheDocument();
    });
  });

  describe('localStorage Integration', () => {
    test('saves answer to localStorage after submission', () => {
      render(<MCQ {...defaultProps} />);

      const optionA = screen.getByLabelText(/Option A: DDS-RTPS/i);
      fireEvent.click(optionA);

      const submitButton = screen.getByRole('button', { name: /submit answer/i });
      fireEvent.click(submitButton);

      const savedData = JSON.parse(localStorage.getItem('physicalai_user_progress') || '{}');
      expect(savedData.chapters.test.mcqScores['test-mcq-01']).toEqual({
        selectedIndex: 0,
        correct: true,
        attemptedAt: expect.any(String),
      });
    });
  });

  describe('Accessibility', () => {
    test('has proper ARIA labels', () => {
      render(<MCQ {...defaultProps} />);

      const radioGroup = screen.getByRole('radiogroup');
      expect(radioGroup).toHaveAttribute('aria-labelledby', 'question-test-mcq-01');
      expect(radioGroup).toHaveAttribute('aria-required', 'true');
    });
  });
});
