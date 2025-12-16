import { useState, useEffect } from 'react';
import { useLanguage } from './providers';
import { useAuth } from './hooks/use-auth';
import { contentApi } from './lib/api';

interface TranslationContextType {
  translatedContent: Record<string, string>;
  translateContent: (key: string, content: string, chapterId?: string) => Promise<void>;
  isTranslating: boolean;
  hasTranslation: (key: string) => boolean;
  getTranslatedContent: (key: string) => string;
}

export function useContentTranslation() {
  const { language } = useLanguage();
  const { user } = useAuth();
  const [translatedContent, setTranslatedContent] = useState<Record<string, string>>({});
  const [isTranslating, setIsTranslating] = useState(false);

  // When language changes, clear translations if switching back to default (en)
  useEffect(() => {
    if (language === 'en') {
      setTranslatedContent({});
    }
  }, [language]);

  const translateContent = async (key: string, content: string, chapterId: string = 'general') => {
    if (language !== 'ur' || !user) return; // Only translate to Urdu for now
    
    setIsTranslating(true);
    try {
      const result = await contentApi.translate(
        user.accessToken as string,
        chapterId,
        content,
        'ur'
      );
      setTranslatedContent(prev => ({
        ...prev,
        [key]: result.translated_content
      }));
    } catch (error) {
      console.error(`Translation failed for key "${key}":`, error);
      // Keep original content in case of error
      setTranslatedContent(prev => ({
        ...prev,
        [key]: content
      }));
    } finally {
      setIsTranslating(false);
    }
  };

  const hasTranslation = (key: string) => {
    return translatedContent.hasOwnProperty(key);
  };

  const getTranslatedContent = (key: string) => {
    return translatedContent[key] || '';
  };

  return {
    translatedContent,
    translateContent,
    isTranslating,
    hasTranslation,
    getTranslatedContent
  };
}