import { useLanguage } from '@/components/providers';
import { ReactNode } from 'react';

interface UrduTextProps {
  children: ReactNode;
  className?: string;
  element?: 'span' | 'div' | 'p';
}

export function UrduText({ children, className = '', element = 'span' }: UrduTextProps) {
  const { language } = useLanguage();
  
  const Element = element;
  
  if (language === 'ur') {
    return (
      <Element className={`urdu-text ${className}`}>
        {children}
      </Element>
    );
  }
  
  return <Element className={className}>{children}</Element>;
}