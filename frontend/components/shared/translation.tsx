import { TranslatedText } from './TranslatedText';

interface TranslationProps {
  children: string;
  id?: string; 
  className?: string;
  element?: 'span' | 'div' | 'p' | 'h1' | 'h2' | 'h3' | 'h4' | 'h5' | 'h6';
}

export function Translation({ children, id, className, element = 'span' }: TranslationProps) {
  return (
    <TranslatedText 
      id={id} 
      className={className}
      element={element}
    >
      {children}
    </TranslatedText>
  );
}