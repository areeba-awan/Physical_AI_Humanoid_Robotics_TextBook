/**
 * Chat API Client for Physical AI Textbook RAG Chatbot
 *
 * Provides:
 * - Global book queries (searches entire book)
 * - Selected text queries (strict mode - only from selection)
 * - Streaming responses via SSE
 * - Chat history management
 */

const API_BASE_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';

// Types
export interface Citation {
  chapter_id: string;
  section: string;
  relevance: number;
  snippet?: string;
}

export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Citation[];
  context_chapter?: string;
  selected_text?: string;
  created_at: string;
}

export interface ChatResponse {
  id: string;
  role: 'assistant';
  content: string;
  sources: Citation[];
  timestamp: string;
}

export interface ChatHistoryResponse {
  messages: ChatMessage[];
  total: number;
  has_more: boolean;
}

export interface StreamChunk {
  type: 'content' | 'done';
  content?: string;
  sources?: Citation[];
  full_content?: string;
  is_sufficient?: boolean;
}

// API Client
export class ChatAPIClient {
  private baseUrl: string;
  private token: string;

  constructor(token: string, baseUrl?: string) {
    this.token = token;
    this.baseUrl = baseUrl || API_BASE_URL;
  }

  private getHeaders(): HeadersInit {
    return {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.token}`,
    };
  }

  /**
   * Send a global book query (searches entire book for context)
   */
  async sendMessage(
    message: string,
    contextChapter?: string
  ): Promise<ChatResponse> {
    const response = await fetch(`${this.baseUrl}/api/v1/chat`, {
      method: 'POST',
      headers: this.getHeaders(),
      body: JSON.stringify({
        message,
        context_chapter: contextChapter,
      }),
    });

    if (!response.ok) {
      throw new Error(`Chat request failed: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * Send a selected text query (STRICT mode - only uses selected text)
   *
   * If the selected text doesn't contain enough information,
   * the response will indicate insufficient context.
   */
  async sendMessageWithSelectedText(
    message: string,
    selectedText: string,
    contextChapter?: string
  ): Promise<ChatResponse> {
    const response = await fetch(`${this.baseUrl}/api/v1/chat/selected-text`, {
      method: 'POST',
      headers: this.getHeaders(),
      body: JSON.stringify({
        message,
        selected_text: selectedText,
        context_chapter: contextChapter,
      }),
    });

    if (!response.ok) {
      throw new Error(`Chat request failed: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * Stream a global book query response
   */
  async *streamMessage(
    message: string,
    contextChapter?: string
  ): AsyncGenerator<StreamChunk> {
    const response = await fetch(`${this.baseUrl}/api/v1/chat/stream`, {
      method: 'POST',
      headers: this.getHeaders(),
      body: JSON.stringify({
        message,
        context_chapter: contextChapter,
      }),
    });

    if (!response.ok) {
      throw new Error(`Stream request failed: ${response.statusText}`);
    }

    const reader = response.body?.getReader();
    if (!reader) {
      throw new Error('No response body');
    }

    const decoder = new TextDecoder();
    let buffer = '';

    while (true) {
      const { done, value } = await reader.read();
      if (done) break;

      buffer += decoder.decode(value, { stream: true });
      const lines = buffer.split('\n');
      buffer = lines.pop() || '';

      for (const line of lines) {
        if (line.startsWith('data: ')) {
          try {
            const chunk: StreamChunk = JSON.parse(line.slice(6));
            yield chunk;
          } catch {
            // Skip invalid JSON
          }
        }
      }
    }
  }

  /**
   * Stream a selected text query response (STRICT mode)
   */
  async *streamMessageWithSelectedText(
    message: string,
    selectedText: string,
    contextChapter?: string
  ): AsyncGenerator<StreamChunk> {
    const response = await fetch(`${this.baseUrl}/api/v1/chat/selected-text/stream`, {
      method: 'POST',
      headers: this.getHeaders(),
      body: JSON.stringify({
        message,
        selected_text: selectedText,
        context_chapter: contextChapter,
      }),
    });

    if (!response.ok) {
      throw new Error(`Stream request failed: ${response.statusText}`);
    }

    const reader = response.body?.getReader();
    if (!reader) {
      throw new Error('No response body');
    }

    const decoder = new TextDecoder();
    let buffer = '';

    while (true) {
      const { done, value } = await reader.read();
      if (done) break;

      buffer += decoder.decode(value, { stream: true });
      const lines = buffer.split('\n');
      buffer = lines.pop() || '';

      for (const line of lines) {
        if (line.startsWith('data: ')) {
          try {
            const chunk: StreamChunk = JSON.parse(line.slice(6));
            yield chunk;
          } catch {
            // Skip invalid JSON
          }
        }
      }
    }
  }

  /**
   * Get chat history
   */
  async getHistory(limit = 50, offset = 0): Promise<ChatHistoryResponse> {
    const response = await fetch(
      `${this.baseUrl}/api/v1/chat/history?limit=${limit}&offset=${offset}`,
      {
        method: 'GET',
        headers: this.getHeaders(),
      }
    );

    if (!response.ok) {
      throw new Error(`History request failed: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * Clear chat history
   */
  async clearHistory(): Promise<{ success: boolean; message: string }> {
    const response = await fetch(`${this.baseUrl}/api/v1/chat/history`, {
      method: 'DELETE',
      headers: this.getHeaders(),
    });

    if (!response.ok) {
      throw new Error(`Clear history failed: ${response.statusText}`);
    }

    return response.json();
  }
}

// React Hook Example
export function useChatAPI(token: string) {
  const client = new ChatAPIClient(token);

  return {
    sendMessage: client.sendMessage.bind(client),
    sendMessageWithSelectedText: client.sendMessageWithSelectedText.bind(client),
    streamMessage: client.streamMessage.bind(client),
    streamMessageWithSelectedText: client.streamMessageWithSelectedText.bind(client),
    getHistory: client.getHistory.bind(client),
    clearHistory: client.clearHistory.bind(client),
  };
}
