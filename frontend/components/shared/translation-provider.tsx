'use client';

import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { useLanguage } from '@/components/providers';
import { useAuth } from '@/hooks/use-auth';
import { contentApi } from '@/lib/api';

interface TranslationCache {
  [key: string]: {
    [lang: string]: string;
  };
}

interface TranslationContextType {
  translateText: (text: string, key?: string) => Promise<string>;
  isTranslating: boolean;
  getTranslatedText: (original: string, key?: string) => string;
}

const TranslationContext = createContext<TranslationContextType | undefined>(undefined);

export function TranslationProvider({ children }: { children: ReactNode }) {
  const { language } = useLanguage();
  const { user } = useAuth();
  const [translationCache, setTranslationCache] = useState<TranslationCache>({});
  const [isTranslating, setIsTranslating] = useState(false);

  // Clear cache when user logs out
  useEffect(() => {
    if (!user) {
      setTranslationCache({});
    }
  }, [user]);

  const translateText = async (text: string, key: string = ''): Promise<string> => {
    if (!user || language === 'en') {
      return text; // Return original text if English or not authenticated
    }
    
    const cacheKey = key || text.substring(0, 20).replace(/\s+/g, '_');
    
    // Check if translation is already cached
    if (translationCache[cacheKey]?.[language]) {
      return translationCache[cacheKey][language];
    }

    setIsTranslating(true);
    try {
      const result = await contentApi.translate(
        user.accessToken as string,
        'homepage',
        text,
        'ur'
      );

      // Update cache
      setTranslationCache(prev => ({
        ...prev,
        [cacheKey]: {
          ...prev[cacheKey],
          [language]: result.translated_content
        }
      }));

      return result.translated_content;
    } catch (error) {
      console.error('Translation failed:', error);
      return text; // Return original text if translation fails
    } finally {
      setIsTranslating(false);
    }
  };

  const getTranslatedText = (original: string, key?: string): string => {
    const cacheKey = key || original.substring(0, 20).replace(/\s+/g, '_');
    return translationCache[cacheKey]?.[language] || original;
  };

  return (
    <TranslationContext.Provider value={{ translateText, isTranslating, getTranslatedText }}>
      {children}
    </TranslationContext.Provider>
  );
}

export function useTranslationContext() {
  const context = useContext(TranslationContext);
  if (context === undefined) {
    throw new Error('useTranslationContext must be used within a TranslationProvider');
  }
  return context;
}

// Component to translate text content
export function TranslatedText({ 
  children, 
  id,
  className,
  element = 'span'
}: { 
  children: string; 
  id?: string;
  className?: string;
  element?: 'span' | 'div' | 'p' | 'h1' | 'h2' | 'h3' | 'h4' | 'h5' | 'h6';
}) {
  const { language } = useLanguage();
  const { getTranslatedText, isTranslating } = useTranslationContext();
  const { user } = useAuth();
  
  const translated = user && language === 'ur' ? getTranslatedText(children, id) : children;

  const Element = element;

  if (isTranslating) {
    return <Element className={className}>...</Element>;
  }

  return <Element className={className}>{translated}</Element>;
}