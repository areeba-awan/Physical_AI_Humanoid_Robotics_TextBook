import React, { useState } from 'react';
import styles from './styles.module.css';

interface TranslateButtonProps {
  chapterId: string;
}

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

export default function TranslateButton({ chapterId }: TranslateButtonProps): JSX.Element {
  const [isLoading, setIsLoading] = useState(false);
  const [currentLanguage, setCurrentLanguage] = useState<'en' | 'ur'>('en');
  const [error, setError] = useState<string | null>(null);
  const [originalContent, setOriginalContent] = useState<string | null>(null);

  const handleTranslate = async () => {
    if (isLoading) return;

    setIsLoading(true);
    setError(null);

    try {
      const contentElement = document.querySelector('.theme-doc-markdown');
      if (!contentElement) {
        throw new Error('Could not find chapter content');
      }

      if (currentLanguage === 'en') {
        // Save original content
        setOriginalContent(contentElement.innerHTML);

        // Call translation API
        const response = await fetch(`${API_URL}/api/v1/chapters/${chapterId}/translate`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            content: contentElement.innerHTML,
            targetLanguage: 'ur',
          }),
        });

        if (!response.ok) {
          throw new Error('Translation failed');
        }

        const data = await response.json();

        // Replace content with translated version
        contentElement.innerHTML = data.translatedContent;
        contentElement.setAttribute('dir', 'rtl');
        contentElement.classList.add(styles.urduText);
        setCurrentLanguage('ur');
      } else {
        // Revert to original
        if (originalContent) {
          contentElement.innerHTML = originalContent;
          contentElement.setAttribute('dir', 'ltr');
          contentElement.classList.remove(styles.urduText);
          setCurrentLanguage('en');
        }
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Translation error');
      console.error('Translation error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.container}>
      <button
        className={`${styles.button} ${currentLanguage === 'ur' ? styles.active : ''}`}
        onClick={handleTranslate}
        disabled={isLoading}
      >
        {isLoading ? (
          <>
            <span className={styles.spinner}></span>
            Translating...
          </>
        ) : currentLanguage === 'en' ? (
          <>
            <span className={styles.icon}>🌐</span>
            اردو میں پڑھیں
          </>
        ) : (
          <>
            <span className={styles.icon}>🌐</span>
            View in English
          </>
        )}
      </button>

      {currentLanguage === 'ur' && (
        <span className={styles.badge}>اردو</span>
      )}

      {error && (
        <div className={styles.error}>
          {error}
        </div>
      )}
    </div>
  );
}
