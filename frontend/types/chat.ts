export interface Citation {
  chapterId: string;
  section: string;
  relevance: number;
  snippet?: string;
}

export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Citation[];
  timestamp: Date;
  contextChapter?: string;
  selectedText?: string;
}

export interface ChatState {
  messages: ChatMessage[];
  isOpen: boolean;
  isLoading: boolean;
  selectedText: string | null;
  contextChapter: string | null;
}

export interface ChatRequest {
  message: string;
  contextChapter?: string;
  selectedText?: string;
}

export interface ChatResponse {
  id: string;
  role: 'assistant';
  content: string;
  sources: Citation[];
  timestamp: string;
}
