"use client";

import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
import { ThemeProvider } from "next-themes";
import { useState, createContext, useContext, ReactNode, useEffect } from "react";
import { TranslationProvider } from "@/components/shared/translation-provider";

interface LanguageContextType {
  language: 'en' | 'ur';
  toggleLanguage: (lang: 'en' | 'ur') => void;
  dir: 'ltr' | 'rtl';
}

const LanguageContext = createContext<LanguageContextType | undefined>(undefined);

export function LanguageProvider({ children }: { children: ReactNode }) {
  const [language, setLanguage] = useState<'en' | 'ur'>('en');

  useEffect(() => {
    // Check for saved language preference in localStorage or from user profile
    const savedLang = localStorage.getItem('preferredLanguage') as 'en' | 'ur' | null;
    if (savedLang && (savedLang === 'en' || savedLang === 'ur')) {
      setLanguage(savedLang);
    } else {
      // Set the language based on the browser's language preference
      const browserLang = navigator.language.startsWith('ur') ? 'ur' : 'en';
      setLanguage(browserLang);
    }
  }, []);

  const toggleLanguage = async (lang: 'en' | 'ur') => {
    setLanguage(lang);
    localStorage.setItem('preferredLanguage', lang);
    // Update the direction for RTL support
    document.documentElement.dir = lang === 'ur' ? 'rtl' : 'ltr';
  };

  // Set the direction when language changes
  useEffect(() => {
    document.documentElement.dir = language === 'ur' ? 'rtl' : 'ltr';
  }, [language]);

  const value: LanguageContextType = {
    language,
    toggleLanguage,
    dir: language === 'ur' ? 'rtl' : 'ltr',
  };

  return (
    <LanguageContext.Provider value={value}>
      {children}
    </LanguageContext.Provider>
  );
}

export function useLanguage() {
  const context = useContext(LanguageContext);
  if (context === undefined) {
    throw new Error('useLanguage must be used within a LanguageProvider');
  }
  return context;
}

export function Providers({ children }: { children: React.ReactNode }) {
  const [queryClient] = useState(
    () =>
      new QueryClient({
        defaultOptions: {
          queries: {
            staleTime: 60 * 1000,
            refetchOnWindowFocus: false,
          },
        },
      })
  );

  return (
    <QueryClientProvider client={queryClient}>
      <LanguageProvider>
        <TranslationProvider>
          <ThemeProvider
            attribute="class"
            defaultTheme="system"
            enableSystem
            disableTransitionOnChange
          >
            {children}
          </ThemeProvider>
        </TranslationProvider>
      </LanguageProvider>
    </QueryClientProvider>
  );
}
