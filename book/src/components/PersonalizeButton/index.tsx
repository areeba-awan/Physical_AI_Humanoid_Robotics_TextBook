import React, { useState } from 'react';
import styles from './styles.module.css';

interface PersonalizeButtonProps {
  chapterId: string;
}

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

export default function PersonalizeButton({ chapterId }: PersonalizeButtonProps): JSX.Element {
  const [isLoading, setIsLoading] = useState(false);
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handlePersonalize = async () => {
    if (isLoading) return;

    setIsLoading(true);
    setError(null);

    try {
      // Get current chapter content
      const contentElement = document.querySelector('.theme-doc-markdown');
      if (!contentElement) {
        throw new Error('Could not find chapter content');
      }

      const currentContent = contentElement.innerHTML;

      // Call personalization API
      const response = await fetch(`${API_URL}/api/v1/chapters/${chapterId}/personalize`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          // Add auth token if available
        },
        body: JSON.stringify({
          content: currentContent,
        }),
      });

      if (!response.ok) {
        if (response.status === 401) {
          // Redirect to login
          window.location.href = '/signin?redirect=' + encodeURIComponent(window.location.pathname);
          return;
        }
        throw new Error('Failed to personalize content');
      }

      const data = await response.json();

      // Replace content with personalized version
      contentElement.innerHTML = data.personalizedContent;
      setIsPersonalized(true);

      // Show adaptations
      if (data.adaptations && data.adaptations.length > 0) {
        console.log('Content adaptations:', data.adaptations);
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
      console.error('Personalization error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  const handleRevert = () => {
    // Reload the page to get original content
    window.location.reload();
  };

  return (
    <div className={styles.container}>
      {!isPersonalized ? (
        <button
          className={styles.button}
          onClick={handlePersonalize}
          disabled={isLoading}
        >
          {isLoading ? (
            <>
              <span className={styles.spinner}></span>
              Personalizing...
            </>
          ) : (
            <>
              <span className={styles.icon}>👤</span>
              Personalize for Me
            </>
          )}
        </button>
      ) : (
        <button
          className={`${styles.button} ${styles.active}`}
          onClick={handleRevert}
        >
          <span className={styles.icon}>↩️</span>
          View Original
        </button>
      )}

      {error && (
        <div className={styles.error}>
          {error}
        </div>
      )}
    </div>
  );
}
