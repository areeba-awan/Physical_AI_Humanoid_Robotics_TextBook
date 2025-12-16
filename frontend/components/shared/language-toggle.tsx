'use client';

import { Button } from '@/components/ui/button';
import { useLanguage } from '@/components/providers';
import { Globe } from 'lucide-react';
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuTrigger,
} from '@/components/ui/dropdown-menu';
import { usePathname, useRouter } from 'next/navigation';
import { useAuth } from '@/hooks/use-auth';
import { contentApi } from '@/lib/api';
import { useToast } from '@/hooks/use-toast';

export function LanguageToggle() {
  const { language, toggleLanguage } = useLanguage();
  const pathname = usePathname();
  const router = useRouter();
  const { user } = useAuth();
  const { toast } = useToast();

  const handleLanguageChange = async (lang: 'en' | 'ur') => {
    if (!user) {
      toast({
        title: "Authentication Required",
        description: "Please sign in to use translation features.",
        variant: "destructive",
      });
      router.push('/signin');
      return;
    }

    try {
      // Toggle the UI language
      await toggleLanguage(lang);

      // For now, just refresh the page to apply language direction
      // In a real implementation, you would translate content dynamically
      if (typeof window !== 'undefined') {
        // Add a small delay to allow state updates before potential refresh
        setTimeout(() => {
          // Optionally, you could trigger a page translation here
          // For now, we'll just update the HTML direction
          document.documentElement.lang = lang;
        }, 100);
      }

      toast({
        title: "Language Changed",
        description: lang === 'ur' ? "زبان اردو میں تبدیل ہو گئی" : "Language changed to English",
      });
    } catch (error) {
      toast({
        title: "Translation Error",
        description: "Failed to change language. Please try again.",
        variant: "destructive",
      });
    }
  };

  return (
    <DropdownMenu>
      <DropdownMenuTrigger asChild>
        <Button variant="ghost" size="icon" className="relative h-8 w-8 rounded-full">
          <Globe className="h-5 w-5" />
          <span className="sr-only">Toggle language</span>
        </Button>
      </DropdownMenuTrigger>
      <DropdownMenuContent align="end" className="w-40">
        <DropdownMenuItem
          onClick={() => handleLanguageChange('en')}
          className={language === 'en' ? 'bg-accent' : ''}
        >
          <span className="mr-2">🇺🇸</span> English
        </DropdownMenuItem>
        <DropdownMenuItem
          onClick={() => handleLanguageChange('ur')}
          className={language === 'ur' ? 'bg-accent' : ''}
        >
          <span className="mr-2">🇵🇰</span> اردو
        </DropdownMenuItem>
      </DropdownMenuContent>
    </DropdownMenu>
  );
}