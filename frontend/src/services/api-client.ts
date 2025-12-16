// frontend/src/services/api-client.ts

interface QueryRequest {
  query: string;
  context?: Record<string, any>;
}

interface SelectedTextQueryRequest {
  query: string;
  selected_text: string;
}

interface CreateSessionRequest {
  book_id: string;
  user_id?: string;
  metadata?: Record<string, any>;
}

interface QueryResponse {
  response: string;
  references: ContentReference[];
  confidence: number;
  conversation_id: string;
}

interface SessionResponse {
  id: string;
  book_id: string;
  user_id: string | null;
  metadata: Record<string, any> | null;
  created_at: string;
  updated_at: string;
  expires_at: string;
}

interface ContentReference {
  content_id: string;
  text_snippet: string;
  page_number?: number;
  section_title?: string;
}

class ApiClient {
  private baseUrl: string;
  private defaultHeaders: HeadersInit;

  constructor(baseUrl?: string) {
    this.baseUrl = baseUrl || process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';
    this.defaultHeaders = {
      'Content-Type': 'application/json',
    };
  }

  /**
   * Query the book content using RAG
   */
  async queryBook(bookId: string, request: QueryRequest): Promise<QueryResponse> {
    const response = await fetch(`${this.baseUrl}/v1/book/${bookId}/query`, {
      method: 'POST',
      headers: this.defaultHeaders,
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error(`Query request failed with status ${response.status}: ${await response.text()}`);
    }

    return await response.json();
  }

  /**
   * Query only the selected text in the book
   */
  async querySelectedText(bookId: string, request: SelectedTextQueryRequest): Promise<QueryResponse> {
    const response = await fetch(`${this.baseUrl}/v1/book/${bookId}/query-selected`, {
      method: 'POST',
      headers: this.defaultHeaders,
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error(`Selected text query request failed with status ${response.status}: ${await response.text()}`);
    }

    return await response.json();
  }

  /**
   * Create a new chat session
   */
  async createSession(request: CreateSessionRequest): Promise<SessionResponse> {
    const response = await fetch(`${this.baseUrl}/v1/session`, {
      method: 'POST',
      headers: this.defaultHeaders,
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error(`Session creation failed with status ${response.status}: ${await response.text()}`);
    }

    return await response.json();
  }

  /**
   * Get session details
   */
  async getSession(sessionId: string): Promise<SessionResponse> {
    const response = await fetch(`${this.baseUrl}/v1/session/${sessionId}`, {
      method: 'GET',
      headers: this.defaultHeaders,
    });

    if (!response.ok) {
      throw new Error(`Get session failed with status ${response.status}: ${await response.text()}`);
    }

    return await response.json();
  }

  /**
   * Get health status of the API
   */
  async healthCheck(): Promise<{ status: string }> {
    const response = await fetch(`${this.baseUrl}/health`, {
      method: 'GET',
      headers: this.defaultHeaders,
    });

    if (!response.ok) {
      throw new Error(`Health check failed with status ${response.status}: ${await response.text()}`);
    }

    return await response.json();
  }
}

// Create a singleton instance of the API client
export const apiClient = new ApiClient();

// Export the types for use in other modules
export type {
  QueryRequest,
  SelectedTextQueryRequest,
  CreateSessionRequest,
  QueryResponse,
  SessionResponse,
  ContentReference,
};

export default ApiClient;