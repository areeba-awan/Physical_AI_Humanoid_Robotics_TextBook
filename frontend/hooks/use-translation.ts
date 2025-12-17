import { useState, useEffect } from 'react';
import { useLanguage } from '@/components/providers';
import { useAuth } from '@/hooks/use-auth';
import { agentApi } from '@/lib/api';

interface TranslationCache {
  [key: string]: {
    en: string;
    ur: string;
  };
}

export const useTranslation = () => {
  const { language } = useLanguage();
  const { user } = useAuth();
  const [cache, setCache] = useState<TranslationCache>({});
  const [loadingKeys, setLoadingKeys] = useState<Set<string>>(new Set());

  // Load cached translations from localStorage on mount
  useEffect(() => {
    const savedCache = localStorage.getItem('translationCache');
    if (savedCache) {
      try {
        setCache(JSON.parse(savedCache));
      } catch (e) {
        console.error('Failed to parse translation cache', e);
      }
    }
  }, []);

  // Save cache to localStorage when it changes
  useEffect(() => {
    localStorage.setItem('translationCache', JSON.stringify(cache));
  }, [cache]);

  const translate = async (text: string, key?: string): Promise<string> => {
    if (!user || language === 'en') {
      return text;
    }

    const cacheKey = key || text.substring(0, 20).replace(/\s+/g, '_');
    
    // Return cached translation if available
    if (cache[cacheKey]?.[language]) {
      return cache[cacheKey][language];
    }

    // Check if already loading this key
    if (loadingKeys.has(cacheKey)) {
      // Wait for the existing translation to complete
      return new Promise((resolve) => {
        const interval = setInterval(() => {
          if (cache[cacheKey]?.[language]) {
            clearInterval(interval);
            resolve(cache[cacheKey][language]);
          }
        }, 100);
      });
    }

    setLoadingKeys(prev => new Set(prev).add(cacheKey));

    try {
      const result = await agentApi.translate(user.accessToken as string, {
        content: text,
        targetLanguage: 'ur',
      });

      const translatedText = result.translated_text || result.content || text;

      setCache(prev => ({
        ...prev,
        [cacheKey]: {
          ...prev[cacheKey],
          ur: translatedText,
          en: prev[cacheKey]?.en || text
        }
      }));

      return translatedText;
    } catch (error) {
      console.error('Translation failed:', error);
      // Store original text in cache to prevent repeated failed attempts
      setCache(prev => ({
        ...prev,
        [cacheKey]: {
          ...prev[cacheKey],
          ur: text,
          en: text
        }
      }));
      return text;
    } finally {
      setLoadingKeys(prev => {
        const newSet = new Set(prev);
        newSet.delete(cacheKey);
        return newSet;
      });
    }
  };

  const t = (text: string, key?: string): string => {
    const cacheKey = key || text.substring(0, 20).replace(/\s+/g, '_');
    return (cache[cacheKey] && cache[cacheKey][language]) || text;
  };

  // Preload translations for a set of texts
  const preloadTranslations = async (texts: Array<{ text: string; key?: string }>) => {
    const promises = texts.map(({ text, key }) => translate(text, key));
    await Promise.all(promises);
  };

  return {
    t,
    translate,
    preloadTranslations,
    isTranslating: loadingKeys.size > 0,
    currentLanguage: language
  };
};