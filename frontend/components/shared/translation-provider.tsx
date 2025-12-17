'use client';

import React, { createContext, useContext, ReactNode } from 'react';
import { useTranslation } from '@/hooks/use-translation';
import { useLanguage } from '@/components/providers';
import { useAuth } from '@/hooks/use-auth';

interface TranslationContextType {
  translateText: (text: string, key?: string) => Promise<string>;
  isTranslating: boolean;
  getTranslatedText: (original: string, key?: string) => string;
}

const TranslationContext = createContext<TranslationContextType | undefined>(undefined);

export function TranslationProvider({ children }: { children: ReactNode }) {
  const { t, translate, isTranslating } = useTranslation();
  const { language } = useLanguage();
  const { user } = useAuth();

  const getTranslatedText = (original: string, key?: string): string => {
    if (!user || language === 'en') {
      return original;
    }
    return t(original, key);
  };

  const translateText = async (text: string, key?: string): Promise<string> => {
    if (!user || language === 'en') {
      return text;
    }
    return await translate(text, key);
  };

  const value: TranslationContextType = {
    translateText,
    isTranslating,
    getTranslatedText
  };

  return (
    <TranslationContext.Provider value={value}>
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

import { UrduText } from './urdu-text';
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

  if (language === 'ur') {
    return <UrduText element={element} className={className}>{translated}</UrduText>;
  }

  return <Element className={className}>{translated}</Element>;
}