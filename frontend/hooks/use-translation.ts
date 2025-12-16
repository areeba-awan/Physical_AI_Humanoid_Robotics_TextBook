import { useState } from 'react';
import { contentApi } from '@/lib/api';
import { useAuth } from './use-auth';

interface TranslationState {
  loading: boolean;
  error: string | null;
  translatedContent: string | null;
}

export function useTranslation() {
  const { user } = useAuth();
  const [translationState, setTranslationState] = useState<TranslationState>({
    loading: false,
    error: null,
    translatedContent: null,
  });

  const translateContent = async (content: string, chapterId: string, targetLanguage: 'en' | 'ur' = 'ur') => {
    if (!user) {
      throw new Error('User must be authenticated to translate content');
    }

    setTranslationState({
      loading: true,
      error: null,
      translatedContent: null,
    });

    try {
      const response = await contentApi.translate(
        user.accessToken as string,
        chapterId,
        content,
        targetLanguage
      );

      setTranslationState({
        loading: false,
        error: null,
        translatedContent: response.translated_content,
      });

      return response.translated_content;
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Failed to translate content';
      setTranslationState({
        loading: false,
        error: errorMessage,
        translatedContent: null,
      });
      throw error;
    }
  };

  return {
    ...translationState,
    translateContent,
  };
}