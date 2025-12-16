import { create } from 'zustand';
import { persist } from 'zustand/middleware';
import type { ChatMessage } from '@/types/chat';
import { generateId } from '@/lib/utils';

interface ChatState {
  messages: ChatMessage[];
  isOpen: boolean;
  isLoading: boolean;
  selectedText: string | null;
  contextChapter: string | null;
}

interface ChatActions {
  addMessage: (message: Omit<ChatMessage, 'id' | 'timestamp'>) => void;
  setMessages: (messages: ChatMessage[]) => void;
  toggleOpen: () => void;
  setOpen: (open: boolean) => void;
  setLoading: (loading: boolean) => void;
  setSelectedText: (text: string | null) => void;
  setContextChapter: (chapter: string | null) => void;
  clearHistory: () => void;
}

const initialState: ChatState = {
  messages: [],
  isOpen: false,
  isLoading: false,
  selectedText: null,
  contextChapter: null,
};

export const useChatStore = create<ChatState & ChatActions>()(
  persist(
    (set, get) => ({
      ...initialState,

      addMessage: (message) => {
        const newMessage: ChatMessage = {
          ...message,
          id: generateId(),
          timestamp: new Date(),
        };
        set({ messages: [...get().messages, newMessage] });
      },

      setMessages: (messages) => set({ messages }),

      toggleOpen: () => set({ isOpen: !get().isOpen }),

      setOpen: (isOpen) => set({ isOpen }),

      setLoading: (isLoading) => set({ isLoading }),

      setSelectedText: (selectedText) => set({ selectedText }),

      setContextChapter: (contextChapter) => set({ contextChapter }),

      clearHistory: () => set({ messages: [] }),
    }),
    {
      name: 'chat-storage',
      partialize: (state) => ({
        messages: state.messages.slice(-50), // Keep last 50 messages
      }),
    }
  )
);
