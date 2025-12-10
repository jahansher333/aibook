/**
 * Chapter Wrapper Component
 * Automatically adds Urdu Translation Button to any chapter
 * Usage: Wrap your chapter content with this component
 */

import React, { useRef, useEffect, useState } from 'react';
import UrduTranslationButton from './UrduTranslationButton';

interface ChapterWithUrduButtonProps {
  chapterId: string;
  children: React.ReactNode;
}

export const ChapterWithUrduButton: React.FC<ChapterWithUrduButtonProps> = ({
  chapterId,
  children,
}) => {
  const contentRef = useRef<HTMLDivElement>(null);
  const [chapterContent, setChapterContent] = useState<string>('');

  useEffect(() => {
    // Extract chapter content (excluding the translation button)
    if (contentRef.current) {
      const content = contentRef.current.innerHTML;
      setChapterContent(content);
    }
  }, [children]);

  return (
    <div>
      <UrduTranslationButton
        chapterId={chapterId}
        chapterContent={chapterContent}
      >
        <div ref={contentRef}>
          {children}
        </div>
      </UrduTranslationButton>
    </div>
  );
};

export default ChapterWithUrduButton;
