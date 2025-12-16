import { useState, useCallback } from 'react';
import { useAuth } from './use-auth';
import { contentApi } from '@/lib/api';

export interface TranslationFunction {
  (text: string, chapterId?: string): Promise<string>;
}

export function usePageTranslation(): {
  translatePage: (pageContent: Record<string, string>, chapterId?: string) => Promise<Record<string, string>>;
  isTranslating: boolean;
  translationError: string | null;
} {
  const { user } = useAuth();
  const [isTranslating, setIsTranslating] = useState(false);
  const [translationError, setTranslationError] = useState<string | null>(null);

  const translatePage = useCallback(async (pageContent: Record<string, string>, chapterId: string = 'home') => {
    if (!user?.accessToken) {
      throw new Error('User must be authenticated to translate content');
    }

    setIsTranslating(true);
    setTranslationError(null);
    const translations: Record<string, string> = {};
    const errors: string[] = [];

    try {
      // Translate each piece of content 
      for (const [key, content] of Object.entries(pageContent)) {
        try {
          const result = await contentApi.translate(
            user.accessToken as string,
            chapterId,
            content,
            'ur' // translating to Urdu - this would be dynamic based on context
          );
          translations[key] = result.translated_content;
        } catch (error) {
          console.error(`Translation failed for key "${key}":`, error);
          translations[key] = content; // Keep original content if translation fails
          errors.push(key);
        }
      }

      if (errors.length > 0) {
        setTranslationError(`Failed to translate some content: ${errors.join(', ')}`);
      }

      return translations;
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Failed to translate page';
      setTranslationError(errorMessage);
      throw error;
    } finally {
      setIsTranslating(false);
    }
  }, [user]);

  return {
    translatePage,
    isTranslating,
    translationError,
  };
}

export function useTranslationDirection(): {
  translateToUrdu: (text: string, chapterId?: string) => Promise<string>;
  translateToEnglish: (text: string, chapterId?: string) => Promise<string>;
  isTranslating: boolean;
  translationError: string | null;
} {
  const { user } = useAuth();
  const [isTranslating, setIsTranslating] = useState(false);
  const [translationError, setTranslationError] = useState<string | null>(null);

  const translateText = useCallback(async (text: string, targetLanguage: 'en' | 'ur', chapterId: string = 'home') => {
    if (!user?.accessToken) {
      throw new Error('User must be authenticated to translate content');
    }

    setIsTranslating(true);
    setTranslationError(null);

    try {
      const result = await contentApi.translate(
        user.accessToken as string,
        chapterId,
        text,
        targetLanguage
      );
      return result.translated_content;
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Failed to translate content';
      setTranslationError(errorMessage);
      throw error;
    } finally {
      setIsTranslating(false);
    }
  }, [user]);

  const translateToUrdu = useCallback((text: string, chapterId: string = 'home') => 
    translateText(text, 'ur', chapterId), [translateText]);

  const translateToEnglish = useCallback((text: string, chapterId: string = 'home') => 
    translateText(text, 'en', chapterId), [translateText]);

  return {
    translateToUrdu,
    translateToEnglish,
    isTranslating,
    translationError,
  };
}