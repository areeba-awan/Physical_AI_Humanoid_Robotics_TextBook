// frontend/src/services/chat-service.ts

import { apiClient, QueryResponse } from './api-client';

interface ChatMessage {
  id: string;
  role: 'user' | 'assistant' | 'system';
  content: string;
  timestamp: Date;
}

class ChatService {
  /**
   * Sends a query to the chat API and returns the response
   */
  async sendMessage(
    bookId: string, 
    query: string, 
    sessionId?: string,
    context?: any
  ): Promise<QueryResponse> {
    try {
      // If we have a session ID, use the existing session
      if (sessionId) {
        return await apiClient.queryBook(bookId, { 
          query, 
          context: context || {} 
        });
      } else {
        // If no session exists, create one first
        const sessionResponse = await apiClient.createSession({
          book_id: bookId
        });
        
        return await apiClient.queryBook(bookId, { 
          query, 
          context: context || {} 
        });
      }
    } catch (error) {
      console.error('Error sending message:', error);
      throw error;
    }
  }

  /**
   * Sends a selected text query to the API and returns the response
   */
  async sendSelectedTextMessage(
    bookId: string,
    query: string,
    selectedText: string,
    sessionId?: string,
    context?: any
  ): Promise<QueryResponse> {
    try {
      // If we have a session ID, use the existing session
      if (sessionId) {
        return await apiClient.querySelectedText(bookId, { 
          query,
          selected_text: selectedText,
          context: context || {}
        });
      } else {
        // If no session exists, create one first
        const sessionResponse = await apiClient.createSession({
          book_id: bookId
        });
        
        return await apiClient.querySelectedText(bookId, { 
          query,
          selected_text: selectedText,
          context: context || {}
        });
      }
    } catch (error) {
      console.error('Error sending selected text message:', error);
      throw error;
    }
  }

  /**
   * Creates a new session for the chat
   */
  async createSession(bookId: string, userId?: string): Promise<string> {
    try {
      const response = await apiClient.createSession({
        book_id: bookId,
        user_id: userId
      });
      
      return response.id;
    } catch (error) {
      console.error('Error creating session:', error);
      throw error;
    }
  }

  /**
   * Gets session details
   */
  async getSession(sessionId: string) {
    try {
      return await apiClient.getSession(sessionId);
    } catch (error) {
      console.error('Error getting session:', error);
      throw error;
    }
  }

  /**
   * Processes user input and prepares it for sending to the API
   */
  formatMessage(message: string): string {
    // Basic formatting - could include more complex processing in the future
    return message.trim();
  }

  /**
   * Loads conversation history from local storage or session
   */
  loadConversationHistory(sessionId: string): ChatMessage[] {
    try {
      // In a real implementation, this might fetch from an API endpoint
      // For now, we'll return an empty array and implement later
      const stored = localStorage.getItem(`chat-history-${sessionId}`);
      if (stored) {
        const parsed = JSON.parse(stored);
        return parsed.map((msg: any) => ({
          ...msg,
          timestamp: new Date(msg.timestamp)
        }));
      }
      return [];
    } catch (error) {
      console.error('Error loading conversation history:', error);
      return [];
    }
  }

  /**
   * Saves conversation history to local storage
   */
  saveConversationHistory(sessionId: string, messages: ChatMessage[]): void {
    try {
      // In a real implementation, this might save to an API endpoint
      // For now, we'll save to local storage as an example
      const serialized = messages.map(msg => ({
        ...msg,
        timestamp: msg.timestamp.toISOString()
      }));
      localStorage.setItem(`chat-history-${sessionId}`, JSON.stringify(serialized));
    } catch (error) {
      console.error('Error saving conversation history:', error);
    }
  }

  /**
   * Clears conversation history
   */
  clearConversationHistory(sessionId: string): void {
    try {
      localStorage.removeItem(`chat-history-${sessionId}`);
    } catch (error) {
      console.error('Error clearing conversation history:', error);
    }
  }
}

// Export a singleton instance
export const chatService = new ChatService();

export type { ChatMessage };