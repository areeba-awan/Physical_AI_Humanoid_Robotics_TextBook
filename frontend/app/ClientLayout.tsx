'use client';

import { useLanguage } from '@/components/providers';
import { ReactNode, useEffect } from 'react';

interface ClientLayoutProps {
  children: ReactNode;
}

export default function ClientLayout({ children }: ClientLayoutProps) {
  const { language } = useLanguage();
  
  useEffect(() => {
    const html = document.documentElement;
    html.lang = language;
    html.dir = language === 'ur' ? 'rtl' : 'ltr';
  }, [language]);

  return (
    <>{children}</>
  );
}