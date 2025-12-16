const API_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';

interface ApiOptions extends RequestInit {
  token?: string;
}

class ApiError extends Error {
  status: number;
  data: any;

  constructor(message: string, status: number, data?: any) {
    super(message);
    this.name = 'ApiError';
    this.status = status;
    this.data = data;
  }
}

async function apiRequest<T>(
  endpoint: string,
  options: ApiOptions = {}
): Promise<T> {
  const { token, ...fetchOptions } = options;

  const headers: HeadersInit = {
    'Content-Type': 'application/json',
    ...options.headers,
  };

  if (token) {
    headers['Authorization'] = `Bearer ${token}`;
  }

  const response = await fetch(`${API_URL}/api/v1${endpoint}`, {
    ...fetchOptions,
    headers,
    credentials: 'include',
  });

  const data = await response.json().catch(() => null);

  if (!response.ok) {
    throw new ApiError(
      data?.error?.message || 'An error occurred',
      response.status,
      data
    );
  }

  return data;
}

// Auth API
export const authApi = {
  signup: (data: { email: string; password: string; name: string }) =>
    apiRequest('/auth/signup', {
      method: 'POST',
      body: JSON.stringify(data),
    }),

  signin: (data: { email: string; password: string }) =>
    apiRequest('/auth/signin', {
      method: 'POST',
      body: JSON.stringify(data),
    }),

  signout: (token: string) =>
    apiRequest('/auth/signout', {
      method: 'POST',
      token,
    }),

  getSession: (token: string) =>
    apiRequest('/auth/session', {
      method: 'GET',
      token,
    }),

  refresh: () =>
    apiRequest('/auth/refresh', {
      method: 'POST',
    }),
};

// Profile API
export const profileApi = {
  get: (token: string) =>
    apiRequest('/profile', {
      method: 'GET',
      token,
    }),

  update: (token: string, data: any) =>
    apiRequest('/profile', {
      method: 'PUT',
      token,
      body: JSON.stringify(data),
    }),

  updateBackground: (token: string, data: any) =>
    apiRequest('/profile/background', {
      method: 'PUT',
      token,
      body: JSON.stringify(data),
    }),

  getProgress: (token: string) =>
    apiRequest('/profile/progress', {
      method: 'GET',
      token,
    }),

  updateProgress: (token: string, data: any) =>
    apiRequest('/profile/progress', {
      method: 'PUT',
      token,
      body: JSON.stringify(data),
    }),
};

// Chat API
export const chatApi = {
  send: (token: string, data: { message: string; contextChapter?: string; selectedText?: string }) =>
    apiRequest('/chat', {
      method: 'POST',
      token,
      body: JSON.stringify(data),
    }),

  sendWithContext: (token: string, data: { message: string; selectedText: string; contextChapter?: string }) =>
    apiRequest('/chat/context', {
      method: 'POST',
      token,
      body: JSON.stringify(data),
    }),

  getHistory: (token: string, limit = 50, offset = 0) =>
    apiRequest(`/chat/history?limit=${limit}&offset=${offset}`, {
      method: 'GET',
      token,
    }),

  clearHistory: (token: string) =>
    apiRequest('/chat/history', {
      method: 'DELETE',
      token,
    }),
};

// Content API
export const contentApi = {
  personalize: (token: string, chapterId: string, content: string) =>
    apiRequest(`/chapters/${chapterId}/personalize`, {
      method: 'POST',
      token,
      body: JSON.stringify({ content }),
    }),

  translate: (token: string, chapterId: string, content: string, targetLanguage = 'ur') =>
    apiRequest(`/chapters/${chapterId}/translate`, {
      method: 'POST',
      token,
      body: JSON.stringify({ content, targetLanguage }),
    }),

  getSummary: (token: string, chapterId: string) =>
    apiRequest(`/chapters/${chapterId}/summary`, {
      method: 'GET',
      token,
    }),
};

// Agent API
export const agentApi = {
  summarize: (token: string, data: { content: string; summaryType?: string }) =>
    apiRequest('/agents/summarize', {
      method: 'POST',
      token,
      body: JSON.stringify(data),
    }),

  translate: (token: string, data: { content: string; targetLanguage?: string; preserveTerms?: string[] }) =>
    apiRequest('/agents/translate', {
      method: 'POST',
      token,
      body: JSON.stringify(data),
    }),

  personalize: (token: string, data: { content: string; chapterId: string }) =>
    apiRequest('/agents/personalize', {
      method: 'POST',
      token,
      body: JSON.stringify(data),
    }),

  generate: (token: string, data: { topic: string; contentType: string; difficulty?: string; count?: number }) =>
    apiRequest('/agents/generate', {
      method: 'POST',
      token,
      body: JSON.stringify(data),
    }),
};

export { ApiError };
